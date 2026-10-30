#!/usr/bin/env bash
# Race a mapped track: localization (Cartographer, or the MCL particle filter) + pure pursuit on its minimum-time raceline.
#
#   ws/scripts/race.sh <name> [speed_scale:=0.5] [max_speed:=0.0] [pose_topic:=/localized_pose] [avoid:=false]
#                          [localizer:=mcl]    particle filter on map.yaml instead of Cartographer localization
#                          [adaptive_lookahead:=true]   lookahead capped by the raceline's curvature (default; :=false = speed term only)
#
# Needs the bridge running and tracks/<name>/ from ws/scripts/map_track.sh <name>.
# Resets the sim car first: localization starts from the pose mapping started from. The car waits
# until the localizer has localized (~2-3 s) and after every sim reset or crash. Ctrl-C stops.
set -eo pipefail

name=${1:?usage: race.sh <track name> [launch args]}
shift
source "$(dirname "$0")/common.sh"

for f in map.pbstream raceline.csv; do
    [ -e "$tracks/$name/$f" ] || { echo "$tracks/$name/$f missing; run map-track $name  (scripts/map_track.sh)" >&2; exit 1; }
done

# which line the car drives (build_raceline's provenance header line), before the launch output buries it
prov=$(grep -m1 '^# raceline: ' "$tracks/$name/raceline.csv" || true)
if [ -z "$prov" ]; then
    echo "WARNING: $tracks/$name/raceline.csv has UNKNOWN provenance (no '# raceline:' line: built before it" \
         "was recorded, or written by hand); it may not be a converged mintime line. Header:" >&2
    grep -m2 '^#' "$tracks/$name/raceline.csv" >&2 || true
elif [[ "$prov" == *'"method": "mintime", "clean": true'* ]]; then
    echo "Raceline: clean TUM mintime line. $prov"
else
    echo "WARNING: $tracks/$name/raceline.csv is NOT a clean TUM mintime line:" >&2
    echo "$prov" >&2
fi

reset_sim_car

exec ros2 launch control race.launch.py track:="$name" "$@"
