#!/usr/bin/env bash
set -e

cleanup() {
    echo "Caught SIGINT, killing process group..."
    kill -- -s SIGKILL -"$PID" 2>/dev/null
}

trap cleanup INT TERM

# Start program in a new process group
cd ROS

source astra_msgs/install/setup.bash

setsid ros2 run rosbridge_server rosbridge_websocket \
--port 9090 --address "" --url_path "/" --certfile "" \
--keyfile "" --retry_startup_delay 5 --fragment_timeout 600 \
--delay_between_message 0 --max_message_size 10000000 \
--unregister_timeout 10 --call_services_in_new_thread true \
--default_call_service_timeout 1 \
--send_action_goals_in_new_thread true &
PID=$!

wait "$PID"
