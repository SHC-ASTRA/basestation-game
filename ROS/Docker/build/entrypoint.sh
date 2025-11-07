#!/bin/sh
set -e

# Shouldn't need to be changed.... until the "event" occurs again (totally not every cycle)
REPO_DIR="$HOME/astra_msgs"
REPO_URL="https://github.com/SHC-ASTRA/astra_msgs.git"

# QD way to only clone if the repo already exists.
if [ ! -d "$REPO_DIR/.git" ]; then
  echo "Cloning $REPO_URL into $REPO_DIR..."
  git clone "$REPO_URL" "$REPO_DIR"
else
  echo "Repo already exists, skipping clone."
fi

source /opt/ros/humble/setup.bash

# Must build the messages before we can source them; ROS caches the revision so
# this'll only take time if we have to rebuild; and even then should be quick
cd "$REPO_DIR"
colcon build
source install/setup.bash

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
