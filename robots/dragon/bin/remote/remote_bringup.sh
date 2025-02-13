#!/bin/bash

TARGET="192.168.1.100"
PING_COUNT=2

session_name="demo"
main_w="remote"
sub_w="local"
robot="dragon"

tmux new-session -d -s ${session_name} -n ${main_w}

tmux new-window -t ${session_name} -n ${sub_w}
tmux split-window -h -t ${session_name}:${sub_w}.0

if ping -c $PING_COUNT $TARGET > /dev/null 2>&1; then
    echo "$TARGET can connect"

    # remote configuration
    tmux send-keys -t ${session_name}:${main_w}.0 'ssh ' ${robot} ' ' C-m
    tmux send-keys -t ${session_name}:${main_w}.0 'roslaunch dragon bringup.launch estimate_mode:=1' C-m

    tmux send-keys -t ${session_name}:${sub_w}.0 'rossetrobot ' ${TARGET} ' && rossetip' C-m
    tmux send-keys -t ${session_name}:${sub_w}.1 'rossetrobot ' ${TARGET} ' && rossetip' C-m

    sleep 2
else
    echo "$TARGET cannot connected. Start with local machine"

    tmux send-keys -t ${session_name}:${main_w}.0 'roslaunch dragon bringup.launch real_machine:=false simulation:=true' C-m
fi

sleep 3

tmux send-keys -t ${session_name}:${sub_w}.0 'roslaunch aerial_robot_base joy_stick.launch robot_name:=' ${robot} '' C-m
tmux send-keys -t ${session_name}:${sub_w}.1 'rosrun rviz rviz -d `rospack find ' ${robot} '`/config/rviz_config' C-m

tmux attach-session -t ${session_name}:0

















