#!/bin/sh
set -e

FASTDDS_BUILTIN_TRANSPORTS=UDPv4

source /opt/ros/humble/setup.bash

# astra_msgs is copied into the container, so it shouldn't need to be rebuilt.
# If it does, run colcon build in the host folder and relaunch
source /usr/local/astra_msgs/install/setup.bash

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
