# Sourced by the VS Code "ros2" integrated terminal profile.
# Keep the user's normal shell config, then layer ROS 2 and this workspace on top.
[ -f ~/.bashrc ] && source ~/.bashrc

source /opt/ros/jazzy/setup.bash

# Resolve the workspace root from this file's location so it works even if the
# terminal does not open exactly at the workspace root.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi
