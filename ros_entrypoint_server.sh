#!/bin/bash
set -e

# Build and source the ROS workspace if needed
if [ "$BUILD_NEEDED" = true ]; then
  colcon build
fi

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/home/asv_workspace/install/setup.bash"
source "/home/server_workspace/install/setup.bash"

# Welcome information
echo "SERVER ROS2 Docker Image"
echo "---------------------"
echo 'ROS distro: ' $ROS_DISTRO
echo "---------------------"    

# Run your command in the background
ros2 launch server_asv server.launch.py
exec "$@"