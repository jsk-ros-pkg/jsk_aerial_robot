#!/bin/sh

session_name="demo"
main_w="remote"
sub_w="local"
robot="dragon"


tmux new-session -d -s ${session_name} -n ${main_w}

tmux new-window -t ${session_name} -n ${sub_w}
tmux split-window -h -t ${session_name}:${sub_w}.0

tmux send-keys -t ${session_name}:${main_w}.0 'ssh ' ${robot} ' ' C-m
tmux send-keys -t ${session_name}:${main_w}.0 'rossetlocal && rossetip' C-m
tmux send-keys -t ${session_name}:${main_w}.0 'roslaunch dragon bringup.launch real_machine:=false simulation:=true' C-m

sleep 3

tmux send-keys -t ${session_name}:${sub_w}.0 'rossetrobot 192.168.1.100 && rossetip' C-m
tmux send-keys -t ${session_name}:${sub_w}.0 'roslaunch aerial_robot_base joy_stick.launch robot_name:=dragon' C-m
tmux send-keys -t ${session_name}:${sub_w}.1 'rossetrobot 192.168.1.100 && rossetip' C-m
tmux send-keys -t ${session_name}:${sub_w}.1 'rosrun rviz rviz -d `rospack find dragon`/config/rviz_config' C-m

tmux attach-session -t ${session_name}:0














