#!/bin/sh                                                                                                                                                                                                  
session_name="experiment"
main_w="main"
robot1="ninja1"
robot2="ninja2"
robot3="ninja4"
# robot4="ninja4"

tmux new-session -d -s ${session_name} -n ${main_w}

tmux split-window -h -t ${session_name}
tmux split-window -v -t ${session_name}
tmux select-pane -t ${session_name}.0
tmux split-window -v -t ${session_name}

tmux send-keys -t ${session_name}:${main_w}.0 'ssh ' ${robot1} C-m
tmux send-keys -t ${session_name}:${main_w}.1 'ssh ' ${robot2} C-m
tmux send-keys -t ${session_name}:${main_w}.2 'ssh ' ${robot3} C-m
# tmux send-keys -t ${session_name}:${main_w}.3 'ssh ' ${robot4} C-m

tmux send-keys -t ${session_name}:${main_w}.0 'roscd ninja' C-m
tmux send-keys -t ${session_name}:${main_w}.1 'roscd ninja' C-m
tmux send-keys -t ${session_name}:${main_w}.2 'roscd ninja' C-m
# tmux send-keys -t ${session_name}:${main_w}.3 'roscd ninja' C-m


# tmux send-keys -t ${session_name}:${main_w}.1 'roslaunch mecanum_control bringup_remote.launch robot_name:=' ${robot1} C-m
# tmux send-keys -t ${session_name}:${sub_w}.0 'roslaunch mecanum_control bringup_remote.launch robot_name:=' ${robot2} C-m
# tmux send-keys -t ${session_name}:${sub_w}.1 'roslaunch mecanum_control bringup_remote.launch robot_name:=' ${robot3} C-m

tmux attach-session -t ${session_name}
