#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --

if [ -f "/rero/ros_bot/app/ros2_ws/devel/setup.bash" ]; then
    source /rero/ros_bot/app/ros2_ws/devel/setup.bash
fi


exec "$@"