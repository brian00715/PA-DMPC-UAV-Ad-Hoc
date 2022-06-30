from socket import *
import numpy as np
from matplotlib import pyplot as plt
import rosbag
import rospy
import time
from statsmodels.tsa.stattools import adfuller
import os

if __name__ == "__main__":
    # edit hearder.frame of iris_*/target_pose to world in rosbag file
    # perserve the original file name

    # traverse all rosbag files
    folder_path = "/home/simon/wbc_ws/src/dmpc_uav_ad_hoc/data/rosbag"
    bag_files = os.listdir(folder_path)
    for bag_file in bag_files:
        if "latest" in bag_file or ".bag" not in bag_file:
            continue
        # temporarily change name to avoid overwriting
        os.rename(os.path.join(folder_path, bag_file), os.path.join(folder_path, bag_file + ".old"))
        bag_file = os.path.join(folder_path, bag_file + ".old")
        print(f"processing {bag_file}...")
        bag = rosbag.Bag(bag_file)
        edited_bag_file = bag_file[:-4]
        bag_edited = rosbag.Bag(edited_bag_file, "w")
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        total_time = end_time - start_time
        for topic, msg, t in bag.read_messages():
            if "target_pose" in topic:
                # msg.header.frame_id = "world"
                idx = topic.find("iris")
                msg.header.frame_id = f"{topic[idx:idx+6]}"
                bag_edited.write(topic, msg, t)
            else:
                bag_edited.write(topic, msg, t)
            # display process
            time_est = t.to_sec() - start_time
            print(f"\r{time_est/total_time:6.2%}\%", end="")
        print()
        bag.close()
        bag_edited.close()
        # os.remove(bag_file)
