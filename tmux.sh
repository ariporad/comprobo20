#!/bin/sh
tmux start-server
tmux new-session -d -s ros -n roscore "roscore"
tmux new-window -t ros:1 -n rviz "sleep 3; rosrun rviz rviz"
tmux new-window -t ros:2 -n gazebo "sleep 3; roslaunch neato_gazebo neato_gauntlet_world.launch gui:=false"
tmux new-window -t ros:3 -n main
tmux new-window -t ros:4 -n alt

tmux select-window -t ros:3
tmux attach-session -t ros
