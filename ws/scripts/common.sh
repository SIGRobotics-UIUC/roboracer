# Sourced by map_track.sh and race.sh: ROS environment, tracks dir, sim reset.
tracks=${TRACKS_DIR:-$HOME/ws/tracks}
sim_time=${USE_SIM_TIME:-true}  # false for hardware / host wall-clock operation
source /opt/ros/humble/setup.bash
source "$HOME/ws/install/setup.bash"

# Teleport the sim car to its start pose, if the bridge is running (no-op on the car).
# Localization assumes the race starts where mapping started: this reset pose.
# pub waits for the bridge's subscription (a cold ros2 daemon made `topic info` miss it).
reset_sim_car() {
    if timeout 5 ros2 topic pub -w 1 --times 3 -r 10 /autodrive/reset_command std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1; then
        sleep 1
    else
        echo "*** no sim bridge on /autodrive/reset_command: sim car NOT reset (fine on the real car) ***" >&2
    fi
}
