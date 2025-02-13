#!/bin/bash

TARGET="192.168.1.100"
PING_COUNT=2

session_name="demo"
main_w="remote"

if ping -c $PING_COUNT $TARGET > /dev/null 2>&1; then
    tmux send-keys -t ${session_name}:${main_w}.1 'sudo poweroff' C-m
    sleep 5
fi

tmux kill-session -t ${session_name}
