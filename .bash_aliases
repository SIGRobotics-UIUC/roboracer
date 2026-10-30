# ==============================================================================
# ROS 2 Core & Workspace Management
# ==============================================================================
alias c="clear"

# Quickly source ROS 2 Humble base and your local workspace
alias sb="source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash"

# Build options using colcon
alias cb="cd ~/ws && colcon build --symlink-install"
alias cbp="cd ~/ws && colcon build --symlink-install --packages-select"
# cbclean removes only colcon's own logs: log/ also holds map_track logs and analysis scripts
alias cbclean="cd ~/ws && rm -rf build/ install/ log/build_* log/test_* log/latest* && cb"

# ==============================================================================
# AutoDRIVE Sim & Race Pipeline (docs/pipeline.md)
# ==============================================================================
# Bridge to the AutoDRIVE simulator (Connect the sim to 127.0.0.1:4567 after this)
alias sim="ros2 launch autodrive_bridge autodrive_launch.py"

# Keyboard teleop on /drive (W/S speed, A/D steer, X stop, R reset)
alias kb="ros2 run autodrive_bridge teleop_keyboard"

# Map a new track and plan its min-time raceline: map-track <name> [--wall-follow | --gap-follow] [--laps N]
# (a fallback raceline instead of mintime ends in a WARNING and exit status 2)
alias map-track="~/ws/scripts/map_track.sh"

# Race a mapped track: race <name> [speed_scale:=0.5]
alias race="~/ws/scripts/race.sh"

# No-map fallback: follow the gap on /scan, no map or localization: ftg [speed:=1.0]
# (stop it before race: both publish /drive)
alias ftg="ros2 launch control follow_the_gap.launch.py"

# Re-plan the raceline of an existing map: plan <name>
# (exit 0 clean mintime line, 2 fallback line with a WARNING, 1 no raceline)
alias plan="ros2 run planning build_raceline"

# Stop car instantly (publish 0 m/s speed and 0 rad steer)
alias stop-car="ros2 topic pub -1 /drive ackermann_msgs/msg/AckermannDriveStamped '{header: {frame_id: \"base_link\"}, drive: {speed: 0.0, steering_angle: 0.0}}'"

# Teleport the sim car back to its start pose
alias origin-car="ros2 topic pub --times 3 -r 10 /autodrive/reset_command std_msgs/msg/Bool '{data: true}'"

# Full reset: stops the vehicle AND teleports it back to start
alias reset-car="stop-car && origin-car"

# ==============================================================================
# Topic Inspection & Command Shortcuts
# ==============================================================================
# List all active topics
alias rtl="ros2 topic list"

# Echo LiDAR scan stream
alias echo-scan="ros2 topic echo /scan"

# Echo odometry: wheel odometry, Cartographer map pose, sim ground truth
alias echo-odom="ros2 topic echo /odom"
alias echo-pose="ros2 topic echo /tracked_pose"
alias echo-gt="ros2 topic echo /ground_truth/odom"

# Echo drive command output
alias echo-drive="ros2 topic echo /drive"

# ==============================================================================
# ROS 2 Node & Parameter Utilities
# ==============================================================================
alias rnl="ros2 node list"
alias rpl="ros2 param list"
