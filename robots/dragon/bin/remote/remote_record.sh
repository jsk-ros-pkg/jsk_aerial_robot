#!/bin/bash

TARGET="192.168.1.100"
PING_COUNT=2

session_name="demo"
main_w="remote"
record_w="record"

robot="dragon"

tmux new-window -t ${session_name} -n ${record_w}

if ping -c $PING_COUNT $TARGET > /dev/null 2>&1; then
    echo "$TARGET can connect"

    # remote configuration
    tmux send-keys -t ${session_name}:${record_w} 'ssh ' ${robot} ' ' C-m
fi

tmux send-keys -t ${session_name}:${record_w} 'rosrun dragon rosbag_all.sh' C-m
tmux select-window -t ${session_name}:${main_w}


















