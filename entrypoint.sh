#!/bin/bash
set -e

# Source the global ROS 2 Humble setup file
source /opt/ros/humble/setup.bash

# Source the local workspace's setup file if it exists
if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
fi

# Execute the command passed to the Docker container
exec "$@"