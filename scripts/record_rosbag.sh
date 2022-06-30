# parameters: file_path, uav_num, crowd_num
rosbag_cmd="rosbag record -O $1 /crowds/target_net_capacity"

for uav_num in "$2"; do
    uav_topic="/iris_${uav_num}/target_pose"
    rosbag_cmd="${rosbag_cmd} ${topic}"
done
for crowd_num in "$3"; do
    crowd_topic="/crowds/${crowd_num}_path}"
    rosbag_cmd="${rosbag_cmd} ${topic}"
done
echo "record topics: ${rosbag_cmd}"
eval $rosbag_cmd
