#!/bin/bash
# =============================================================================
# ROS2 Entrypoint Script (Real Robot)
# Sources ROS2 setup files and executes the command
# =============================================================================
set -e

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Execute the command passed to the container
exec "$@"

