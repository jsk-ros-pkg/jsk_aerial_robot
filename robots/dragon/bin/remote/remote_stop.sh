#!/bin/bash

session_name="demo"
main_w="remote"
sub_w="local"
record_w="record"

tmux send-keys -t ${session_name}:${main_w}.0 C-c
tmux send-keys -t ${session_name}:${sub_w}.0 C-c
tmux send-keys -t ${session_name}:${sub_w}.1 C-c
tmux send-keys -t ${session_name}:${record_w}.0 C-c













