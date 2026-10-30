#!/usr/bin/env bash
# Map an unseen track and plan its minimum-time raceline (docs/pipeline.md):
#   Cartographer mapping -> save_map -> centerline -> TUM mintime (if that fails: a fallback line, loudly)
# Exit status as build_raceline's: 0 clean mintime raceline, 2 a fallback raceline (WARNING printed), 1 none.
#
#   ws/scripts/map_track.sh <name> [--wall-follow | --gap-follow] [--laps N] [follower launch args, e.g. speed:=1.2]
#
# Needs the bridge running (ros2 launch autodrive_bridge autodrive_launch.py) and the sim connected.
# Drive the lap(s) from another terminal with: ros2 run autodrive_bridge teleop_keyboard
# or pass --wall-follow (wall_follow.launch.py) or --gap-follow (disparity_gap.launch.py): the follower
# drives N laps (default 2), then the map is saved automatically.
# Writes tracks/<name>/: map.pbstream, map.pgm, map.yaml, trajectory.csv, centerline.csv,
# raceline.csv, preview.png. Re-plan an existing map with: ros2 run planning build_raceline <name>
set -eo pipefail

usage="usage: map_track.sh <track name> [--wall-follow | --gap-follow] [--laps N] [follower launch args, e.g. speed:=1.2]"
name=${1:?$usage}
shift
follow_launch=   # the launch file that drives the laps, empty = drive them by hand
laps=2
follow_args=()
while [ $# -gt 0 ]; do
    case $1 in
        --wall-follow|--gap-follow)
            [ -z "$follow_launch" ] || { echo "$usage" >&2; exit 1; }
            follow_launch=$([ "$1" = --wall-follow ] && echo wall_follow.launch.py || echo disparity_gap.launch.py) ;;
        --laps) laps=${2:?$usage}; shift ;;
        *:=*) follow_args+=("$1") ;;
        *) echo "$usage" >&2; exit 1 ;;
    esac
    shift
done
source "$(dirname "$0")/common.sh"

if [ -e "$tracks/$name/map.pbstream" ]; then
    echo "$tracks/$name already has a map; pick another name or delete it" >&2
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
reset_sim_car
# job control: the launch gets its own process group and keeps SIGINT (a script's background
# jobs ignore it otherwise), so stopping it stops every node it started
set -m
ros2 launch perception mapping.launch.py track:="$name" >"$log" 2>&1 &
mapping=$!
set +m
follower=
trap 'stop_group "$follower"; stop_group "$mapping"' EXIT

# ros2 launch stays alive when cartographer_node dies, so check the node's process (not
# `ros2 node list`, which keeps a dead node listed until its DDS lease runs out)
cartographer_up() { pgrep -g "$mapping" -f '^[^ ]*cartographer_ros/cartographer_node( |$)' >/dev/null; }
sleep 3   # a node that crashes on its config dies within a second or two
cartographer_up || { echo "Cartographer failed to start, see $log" >&2; exit 1; }
echo "Mapping '$name' (log: $log)."

if [ -n "$follow_launch" ]; then
    set -m
    ros2 launch control "$follow_launch" laps:="$laps" "${follow_args[@]}" >"$follow_log" 2>&1 &
    follower=$!
    set +m
    sleep 2
    kill -0 "$follower" 2>/dev/null || { echo "$follow_launch failed to start, see $follow_log" >&2; exit 1; }
    echo "$follow_launch driving $laps lap(s) (log: $follow_log). Press Enter to stop early and save the map."
    shown=0
    while kill -0 "$follower" 2>/dev/null; do
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
until ros2 run perception save_map "$name"; do
    cartographer_up || echo "cartographer_node is not running (see $log): the map cannot be saved" >&2
    read -rp "save_map failed. Enter to retry, q + Enter to quit without a map... " ans </dev/tty
    # a late failure may have written map.pbstream already, which blocks this name next time
    [ "$ans" != q ] || { echo "no map saved; delete $tracks/$name before reusing the name" >&2; exit 1; }
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
echo "Done: $tracks/$name (check preview.png). Race with: race $name  (scripts/race.sh)"
exit "$status"
