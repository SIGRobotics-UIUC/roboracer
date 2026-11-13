#!/usr/bin/env bash
# Map an unseen track and plan its minimum-time raceline (docs/pipeline.md):
#   Mapping (Cartographer default, --slam-toolbox opt-in) -> save -> centerline -> TUM mintime
# Exit status as build_raceline's: 0 clean mintime raceline, 2 a fallback raceline (WARNING printed), 1 none.
#
#   ws/scripts/map_track.sh <name> [--slam-toolbox] [--wall-follow | --gap-follow] [--laps N] [follower launch args]
#
# Needs the bridge running (ros2 launch autodrive_bridge autodrive_launch.py) and the sim connected.
# Drive the lap(s) from another terminal with: ros2 run autodrive_bridge teleop_keyboard
# or pass --wall-follow (wall_follow.launch.py) or --gap-follow (disparity_gap.launch.py): the follower
# drives N laps (default 2), then the map is saved automatically.
# Writes tracks/<name>/: map.pbstream (Cartographer) or map.posegraph/map.data (SLAM Toolbox),
# map.pgm, map.yaml, trajectory.csv, centerline.csv,
# raceline.csv, preview.png. Re-plan an existing map with: ros2 run planning build_raceline <name>
set -eo pipefail

usage="usage: map_track.sh <track name> [--slam-toolbox] [--wall-follow | --gap-follow] [--laps N] [follower launch args, e.g. speed:=1.2]"
name=${1:?$usage}
shift
[[ "$name" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || { echo 'track must be a simple directory name' >&2; exit 1; }
backend=cartographer
mapping_launch=mapping.launch.py
save_command=save_map
mapping_process='cartographer_ros/cartographer_node'
follow_launch=   # the launch file that drives the laps, empty = drive them by hand
laps=2
follow_args=()
while [ $# -gt 0 ]; do
    case $1 in
        --slam-toolbox)
            backend=slam_toolbox
            mapping_launch=slam_mapping.launch.py
            save_command=save_slam_map
            mapping_process='slam_toolbox/async_slam_toolbox_node' ;;
        --wall-follow|--gap-follow)
            [ -z "$follow_launch" ] || { echo "$usage" >&2; exit 1; }
            follow_launch=$([ "$1" = --wall-follow ] && echo wall_follow.launch.py || echo disparity_gap.launch.py) ;;
        --laps) laps=${2:?$usage}; shift ;;
        *:=*) follow_args+=("$1") ;;
        *) echo "$usage" >&2; exit 1 ;;
    esac
    shift
done
[[ "$laps" =~ ^[0-9]+$ ]] || { echo '--laps must be a nonnegative integer' >&2; exit 1; }
if [ "$backend" = slam_toolbox ] && [ -n "$follow_launch" ]; then
    # Half-speed experiment after the first real map showed large SLAM corrections.
    # Explicit user launch arguments remain last and can override this default.
    follow_args=("speed:=0.5" "${follow_args[@]}")
fi
source "$(dirname "$0")/common.sh"

if [ -e "$tracks/$name" ] || [ -L "$tracks/$name" ]; then
    echo "$tracks/$name already exists; preserve it and pick another name" >&2
    exit 1
fi
# the Enter prompts read the terminal: without one, `read` fails and the mapping run is lost
if ! { : </dev/tty; } 2>/dev/null; then
    echo "map_track.sh needs a terminal (run it in an interactive shell, e.g. docker compose exec ros bash)" >&2
    exit 1
fi

# stop a background launch started under job control: SIGINT to its process group, then SIGTERM
stop_group() {
    [ -n "$1" ] || return 0
    kill -INT -- "-$1" 2>/dev/null || return 0
    for _ in $(seq 20); do kill -0 -- "-$1" 2>/dev/null || return 0; sleep 0.5; done
    kill -TERM -- "-$1" 2>/dev/null || true
}

log="$HOME/ws/log/map_track_$name.log"
follow_log="$HOME/ws/log/map_track_${name}_${follow_launch%%.*}.log"
mkdir -p "$(dirname "$log")"
# Never start a second map -> odom owner or competing controller.
active_nodes=$(ros2 node list)
if echo "$active_nodes" | grep -Eq '^/(cartographer_node|slam_toolbox|mcl|localization_gate|pure_pursuit_node|reactive_node|wall_follow_node|disparity_gap_node|teleop_keyboard)$'; then
    echo 'Stop existing mapping, localization and driving nodes before mapping' >&2
    exit 1
fi
reset_sim_car
# job control: the launch gets its own process group and keeps SIGINT (a script's background
# jobs ignore it otherwise), so stopping it stops every node it started
set -m
ros2 launch perception mapping.launch.py track:="$name" use_sim_time:="$sim_time" >"$log" 2>&1 &
mapping=$!
set +m
follower=
trap 'stop_group "$follower"; stop_group "$mapping"' EXIT

# ros2 launch stays alive when cartographer_node dies, so check the node's process (not
# `ros2 node list`, which keeps a dead node listed until its DDS lease runs out)
mapper_up() { pgrep -g "$mapping" -f "^[^ ]*$mapping_process( |$)" >/dev/null; }
sleep 3   # a node that crashes on its config dies within a second or two
mapper_up || { echo "$backend failed to start, see $log" >&2; exit 1; }
echo "Mapping '$name' with $backend (log: $log)."

if [ -n "$follow_launch" ]; then
    set -m
    ros2 launch control "$follow_launch" use_sim_time:="$sim_time" laps:="$laps" "${follow_args[@]}" >"$follow_log" 2>&1 &
    follower=$!
    set +m
    sleep 2
    kill -0 "$follower" 2>/dev/null || { echo "$follow_launch failed to start, see $follow_log" >&2; exit 1; }
    echo "$follow_launch driving $laps lap(s) (log: $follow_log). Press Enter to stop early and save the map."
    shown=0
    while kill -0 "$follower" 2>/dev/null; do
        mapper_up || { echo "$backend stopped during mapping; see $log" >&2; exit 1; }
        if read -r -t 1 _ </dev/tty; then
            break
        fi
        done_laps=$(grep -c 'Lap [0-9]*/[0-9]* complete' "$follow_log" || true)
        if [ "$done_laps" -gt "$shown" ]; then
            shown=$done_laps
            echo "  lap $shown/$laps"
        fi
    done
    finished=false
    kill -0 "$follower" 2>/dev/null || finished=true
    stop_group "$follower"      # stops the car
    follower=
    if $finished && [ "$laps" -gt 0 ] && [ "$shown" -lt "$laps" ]; then
        echo "$follow_launch stopped after $shown of $laps lap(s), see $follow_log" >&2
        read -rp "Press Enter to save the map anyway (Ctrl-C to abort)... " </dev/tty
    fi
else
    echo "Drive 1-2 clean laps in another terminal: ros2 run autodrive_bridge teleop_keyboard"
    read -rp "Stop the car, then press Enter to save the map... " </dev/tty
fi

# keep mapping running on a failed save (the session is not on disk yet); save_map can be rerun
until ros2 run perception "$save_command" "$name"; do
    mapper_up || echo "$backend is not running (see $log): the map cannot be saved" >&2
    read -rp "$save_command failed. Enter to retry, q + Enter to quit without a map... " ans </dev/tty
    # a late failure may have written map.pbstream already, which blocks this name next time
    [ "$ans" != q ] || { echo "save incomplete; preserve any partial artifacts and use a new name" >&2; exit 1; }
done
stop_group "$mapping"
trap - EXIT

# build_raceline: 0 clean mintime line, 2 a raceline but NOT a clean mintime line (fallback), 1 none
status=0
ros2 run planning build_raceline "$name" || status=$?
if [ "$status" -eq 2 ]; then
    echo "WARNING: $tracks/$name/raceline.csv is NOT a clean TUM mintime line (fallback, see the WARNING above" \
         "and its '# raceline:' header line). Check preview.png before racing." >&2
elif [ "$status" -ne 0 ]; then
    echo "build_raceline failed: no raceline for $name (map saved; rerun: ros2 run planning build_raceline $name)" >&2
    exit "$status"
fi
if [ "$backend" = slam_toolbox ]; then
    echo "Done: $tracks/$name (check preview.png). SLAM Toolbox graph is not a Cartographer pbstream."
    echo "For MCL later, reset the car to its mapping start and launch: ros2 launch control race.launch.py track:=$name localizer:=mcl speed_scale:=0.3"
else
    echo "Done: $tracks/$name (check preview.png). Race with: race $name  (scripts/race.sh)"
fi
exit "$status"
