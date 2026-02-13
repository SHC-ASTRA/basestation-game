#!/bin/sh
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

[[ -z "$BS_GAME_WS" ]] && BS_GAME_WS="$HOME/ASTRA/basestation-game"

if command -v nixos-rebuild; then
    echo "[INFO] running on NixOS"
else
    source /opt/ros/humble/setup.bash
fi

source ROS/Compiler/astra_msgs/install/setup.bash

if ! command ros2 interface list | grep astra_msgs; then
    echo "[ERROR] astra_msgs not properly included in path"
    exit
fi

cd $BS_GAME_WS && ros2 run rosbridge_server rosbridge_websocket \
--port 9090 --address "" --url_path "/" --certfile "" \
--keyfile "" --retry_startup_delay 5 --fragment_timeout 600 \
--delay_between_message 0 --max_message_size 10000000 \
--unregister_timeout 10 --call_services_in_new_thread true \
--default_call_service_timeout 1 \
--send_action_goals_in_new_thread true
