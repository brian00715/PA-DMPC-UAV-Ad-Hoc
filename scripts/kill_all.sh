#!/bin/zsh
tmux kill-server
source ./scripts/kill_gazebo.sh
source ./scripts/kill_ros.sh
kill -9 $(pgrep -f dmpc_uav_ad_hoc) 
kill -9 $(pgrep -f px4) 