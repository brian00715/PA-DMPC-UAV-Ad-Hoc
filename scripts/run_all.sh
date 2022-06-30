#!/bin/zsh
uav_num=3
run_now=false
simu_time=60
controller="nmpc"
attach=false
param_fixed=false # use fixed nmpc parameters
global_scheduler=false
local_scheduler=false
result_file_folder="default"
while getopts "u:rt:c:afglo:" opt; do
    case $opt in
    u)
        uav_num=$OPTARG
        ;;
    r)
        run_now=true
        ;;
    t)
        simu_time=$OPTARG
        ;;
    c)
        controller=$OPTARG
        ;;
    a)
        attach=$OPTARG
        ;;
    f)
        param_fixed=true
        ;;
    g)
        global_scheduler=true
        ;;
    l)
        local_scheduler=true
        ;;
    o)
        result_file_folder=$OPTARG
        ;;
    ?)
        echo "Usage: run_all.sh [-u uav_num] [-r] [-t simu_time] [-c px4/nmpc] [-a attach]"
        exit 1
        ;;
    esac
done

echo "Start a Tmux session."
tmux start-server

tmux new-session -d -s dmpc_uav_ad_hoc

tmux new-window -n debug -t dmpc_uav_ad_hoc
tmux split-window -h -t dmpc_uav_ad_hoc:debug
tmux split-window -v -t dmpc_uav_ad_hoc:debug.1
tmux send -t dmpc_uav_ad_hoc:debug.1 "rosrun plotjuggler plotjuggler -l ./config/plotjuggler_layout.xml"
tmux send -t dmpc_uav_ad_hoc:debug.2 "rosrun rqt_reconfigure rqt_reconfigure"

# > window: main
tmux new-window -n main -t dmpc_uav_ad_hoc
tmux split-window -h -t dmpc_uav_ad_hoc:main
tmux split-window -v -t dmpc_uav_ad_hoc:main.1
tmux split-window -v -t dmpc_uav_ad_hoc:main.3
tmux split-window -h -t dmpc_uav_ad_hoc:main.1
tmux split-window -v -t dmpc_uav_ad_hoc:main.1
tmux split-window -v -t dmpc_uav_ad_hoc:main.1
tmux split-window -v -t dmpc_uav_ad_hoc:main.6
tmux split-window -v -t dmpc_uav_ad_hoc:main.5
tmux send -t dmpc_uav_ad_hoc:main.1 "roscore" ENTER
tmux send -t dmpc_uav_ad_hoc:main.2 "./scripts/kill_ros.sh"
tmux send -t dmpc_uav_ad_hoc:main.3 "./scripts/kill_gazebo.sh"
time_per_uav=2.2
if $run_now; then
    tmux send -t dmpc_uav_ad_hoc:main.4 "sleep $(($time_per_uav * $uav_num)) && rosrun dmpc_uav_ad_hoc swarm_controller_switch.py on _uav_num:=$uav_num" ENTER
    tmux send -t dmpc_uav_ad_hoc:main.4 "rosrun dmpc_uav_ad_hoc swarm_controller_switch.py on _uav_num:=$uav_num"
    tmux send -t dmpc_uav_ad_hoc:main.5 "sleep 2 && roslaunch dmpc_uav_ad_hoc simu_env.launch uav_num:=$uav_num" ENTER
    tmux send -t dmpc_uav_ad_hoc:main.6 "sleep $(($time_per_uav * $uav_num)) && rosrun dmpc_uav_ad_hoc swarm_controller_launcher.py --uav_num $uav_num --rviz --controller $controller" ENTER
    # tmux send -t dmpc_uav_ad_hoc:main.7 "rosrun dmpc_uav_ad_hoc crowd_simu_node.py _uav_num:=$uav_num _crowds_num:=$uav_num"
    tmux send -t dmpc_uav_ad_hoc:main.7 "sleep $((6 + $time_per_uav * $uav_num)) && rosrun dmpc_uav_ad_hoc simu_time_ctrl.py _simu_time:=$simu_time _uav_num:=$uav_num" ENTER
    tmux send -t dmpc_uav_ad_hoc:main.8 "sleep $((3 + $time_per_uav * $uav_num)) && rosrun dmpc_uav_ad_hoc perf_analysis_node.py _uav_num:=$uav_num _controller:=$controller _result_file_folder:=$result_file_folder" ENTER
    if ! $param_fixed; then
        tmux send -t dmpc_uav_ad_hoc:main.9 "sleep $((3 + $time_per_uav * $uav_num)) && rosrun dmpc_uav_ad_hoc scheduler_node.py _uav_num:=$uav_num _global:=$global_scheduler _local:=$local_scheduler" ENTER
    fi
else
    tmux send -t dmpc_uav_ad_hoc:main.4 "rosrun dmpc_uav_ad_hoc swarm_controller_switch.py on _uav_num:=$uav_num"
    tmux send -t dmpc_uav_ad_hoc:main.5 "roslaunch dmpc_uav_ad_hoc simu_env.launch uav_num:=$uav_num"
    tmux send -t dmpc_uav_ad_hoc:main.6 "rosrun dmpc_uav_ad_hoc swarm_controller_launcher.py --uav_num $uav_num --rviz --controller $controller"
    # tmux send -t dmpc_uav_ad_hoc:main.7 "rosrun dmpc_uav_ad_hoc crowd_simu_node.py _uav_num:=$uav_num _crowds_num:=$uav_num"
    tmux send -t dmpc_uav_ad_hoc:main.7 "rosrun dmpc_uav_ad_hoc simu_time_ctrl.py _simu_time:=$simu_time _uav_num:=$uav_num"
    tmux send -t dmpc_uav_ad_hoc:main.8 "rosrun dmpc_uav_ad_hoc perf_analysis_node.py _uav_num:=$uav_num _controller:=$controller _result_file_folder:=$result_file_folder"
    tmux send -t dmpc_uav_ad_hoc:main.9 "rosrun dmpc_uav_ad_hoc scheduler_node.py _uav_num:=$uav_num _global:=$global_scheduler _local:=$local_scheduler"
fi

if $attach; then
    tmux attach-session -t dmpc_uav_ad_hoc
fi
