#!/usr/bin/env python3
"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-05-13 15:17:25
 # @ Modified time: 2022-05-13 16:34:18
 # @ Description:
 """
import json
import os
import random
import sys
import time

import numpy as np
import rosbag
import rospy
import tf
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Bool
from tqdm import tqdm

from crowd_model import CrowdModel


def generate_grid_points(num, x_min, x_max, y_min, y_max, step):
    grid_points = []

    for x in range(x_min, x_max + step, step):
        for y in range(y_min, y_max + step, step):
            grid_points.append((x, y, 6, 0, 0, random.uniform(0, 2 * np.pi)))
    random_points = random.sample(grid_points, num)

    return random_points


if __name__ == "__main__":
    rospy.init_node("crowd_simu_node", anonymous=False)
    uav_num = rospy.get_param("~uav_num", 3)
    crowds_num = rospy.get_param("~crowds_num", 3)
    record2bag = rospy.get_param("~record2bag", False)
    simu_time = rospy.get_param("~simu_time", 60)  # seconds
    time_suffix = time.strftime("%m-%d-%H%M%S", time.localtime())
    pbar = tqdm(total=simu_time, desc="simu progess", unit="s")
    simu_param_dict = {
        "uav_num": uav_num,
        "crowds_num": crowds_num,
        "record2bag": record2bag,
        "simu_time": simu_time,
    }
    print(f"{'uav_num':<25}: {uav_num}")
    print(f"{'crowds_num':<25}: {crowds_num}")
    print(f"{'record2bag':<25}: {record2bag}")
    print(f"{'simu_time':<25}: {simu_time}")
    print("-" * 50)

    if record2bag:
        bag_path = os.path.join(
            sys.path[0], f"../data/rosbag/crowd_simu_u{uav_num}_c{crowds_num}_" + str(time_suffix) + ".bag"
        )
        bag_writer = rosbag.Bag(bag_path, "w")

    # pubs
    enable_simu_pub = rospy.Publisher("/enable_simu", Bool, queue_size=1, latch=True)
    path_pubs = [rospy.Publisher("crowds/" + str(i) + "_path", Path, queue_size=1) for i in range(crowds_num)]
    test_pub = rospy.Publisher("crowds/test", Float32MultiArray, queue_size=1)
    traffic_pub = rospy.Publisher("crowds/target_net_capacity", Float32MultiArray, queue_size=1)
    uav_pose_pubs = []
    for i in range(uav_num):
        uav_pose_pubs.append(rospy.Publisher(f"iris_{i}/target_pose", PoseStamped, queue_size=1))
    tf_pub = tf.broadcaster.TransformBroadcaster()

    base_rate = 10
    mobility_step_freq = 0.5  # 0.5
    traffic_step_freq = 10
    uav_cruise_height = 10

    # gauss markov parameters
    speed_alpha = 0.1  # 0.2 can be thought of as the inertia of the system
    dir_alpha = 0.7  # 0.5
    # ave_speed = 0.25 / mobility_step_rate
    ave_speed = 1.0 / mobility_step_freq
    ave_dir = np.radians(45)

    # net traffic parameters
    net_traf_ave = 25
    net_traf_std = 1.5
    net_traf_mutation_ar = np.concatenate([np.zeros(2), [5, 10, 15, 25, 30]])

    # x,y,z,roll,pitch,yaw
    init_poses = generate_grid_points(crowds_num, -5, 5, -10, 10, 1)

    crowds = []
    for i in range(crowds_num):
        init_pos = init_poses[i][:2]
        init_dir = init_poses[i][5]
        init_speed = random.uniform(ave_speed / 10, ave_speed)
        traffic_mutation_prob = random.uniform(0.01, 0.03)
        # traffic_mutation_prob = 0.05
        print(f"params for crowd_{i}:")
        print(f"{'init_pos':<25}: {init_pos}")
        print(f"{'init_speed':<25}: {init_speed:<25}")
        print(f"{'init_dir':<25}: {init_dir:<25}")
        # print(f"{'ave_speed':<25}: {ave_speed:<25}")
        # print(f"{'ave_dir':<25}: {ave_dir:<25}")
        # print(f"{'speed_alpha':<25}: {speed_alpha:<25}")
        # print(f"{'net_traf_ave':<25}: {net_traf_ave:<25}")
        # print(f"{'net_traf_std':<25}: {net_traf_std:<25}")
        print(f"{'traffic_mutation_prob':<25}: {traffic_mutation_prob:<25}")
        print("-" * 50)
        simu_param_dict[f"crowd_{i}"] = {
            "init_pos": init_pos,
            "init_speed": init_speed,
            "init_dir": init_dir,
            "ave_speed": ave_speed,
            "ave_dir": ave_dir,
            "speed_alpha": speed_alpha,
            "dir_alpha": dir_alpha,
            "net_traf_ave": net_traf_ave,
            "net_traf_std": net_traf_std,
            "traffic_mutation_prob": traffic_mutation_prob,
        }
        crowds.append(
            CrowdModel(
                init_pos,
                init_speed,
                init_dir,
                ave_speed,
                ave_dir,
                speed_alpha,
                dir_alpha,
                net_traf_ave,
                net_traf_std,
                traffic_mutation_prob,
                net_traf_mutation_ar=net_traf_mutation_ar,
            )
        )

    loop_rate = rospy.Rate(base_rate)
    paths = [Path() for i in range(crowds_num)]
    count = 0

    rospy.loginfo("crowd simulation started")
    start_time = rospy.get_time()
    while start_time < 1:
        start_time = rospy.get_time()
        loop_rate.sleep()
    enable_simu_pub.publish(True)
    while not rospy.is_shutdown():
        poses = PoseArray()
        traffics = Float32MultiArray()
        for i in range(crowds_num):
            if count % int(base_rate / mobility_step_freq) == 0:
                pose = crowds[i].step_mobility()

                pose_msg = Pose()
                pose_msg.position.x = pose[0]
                pose_msg.position.y = pose[1]
                pose_msg.position.z = 0
                yaw = crowds[i].dir
                quat = tft.quaternion_from_euler(0, 0, yaw)
                pose_msg.orientation.x = quat[0]
                pose_msg.orientation.y = quat[1]
                pose_msg.orientation.z = quat[2]
                pose_msg.orientation.w = quat[3]
                poses.poses.append(pose_msg)

                tf_pub.sendTransform(
                    (pose[0], pose[1], 0),
                    (quat[0], quat[1], quat[2], quat[3]),
                    rospy.Time.now(),
                    "crowd_" + str(i),
                    "world",
                )

                pose_stamp_msg = PoseStamped()
                pose_stamp_msg.header.frame_id = f"iris_{i}"
                pose_stamp_msg.header.stamp = rospy.Time.now()
                pose_stamp_msg.pose = pose_msg
                pose_stamp_msg.pose.position.z = uav_cruise_height + i * 0.2
                pose_stamp_msg.pose.orientation.x = 0
                pose_stamp_msg.pose.orientation.y = 0
                pose_stamp_msg.pose.orientation.z = 0
                pose_stamp_msg.pose.orientation.w = 1
                uav_pose_pubs[i].publish(pose_stamp_msg)
                if record2bag:
                    bag_writer.write(uav_pose_pubs[i].name, pose_stamp_msg)

                paths[i].header.frame_id = "world"
                paths[i].header.stamp = rospy.Time.now()
                pose_stamp_msg.pose.position.z = 0
                paths[i].poses.append(pose_stamp_msg)

            if count % int(base_rate / traffic_step_freq) == 0:
                traffic = crowds[i].step_traffic()
                traffics.data.append(traffic)

            path_pubs[i].publish(paths[i])
            if record2bag:
                bag_writer.write(path_pubs[i].name, paths[i])
        traffic_pub.publish(traffics)
        if record2bag:
            bag_writer.write(traffic_pub.name, traffics)

        # test_msg = Float32MultiArray()
        # test_msg.data = [traffic, crowds[i].speed]
        # test_pub.publish(test_msg)

        delta_time = rospy.get_time() - start_time
        pbar.update(int(delta_time) - pbar.n)
        if delta_time >= simu_time:
            pbar.close()
            enable_simu_pub.publish(False)
            rospy.loginfo("crowd simulation finished")
            if record2bag:
                json.dump(simu_param_dict, open(bag_path.replace(".bag", ".json"), "w"))
                if record2bag:
                    bag_writer.close()
                    rospy.loginfo("bag file saved to: " + bag_path)
                rospy.loginfo("json file saved to: " + bag_path.replace(".bag", ".json"))
                latest_path = os.path.join(
                    sys.path[0], f"../data/rosbag/crowd_simu_u{uav_num}_c{crowds_num}_latest.bag"
                )
                os.system(f"rm {latest_path}")
                os.system(f"ln -s {bag_path} {latest_path}")
            break

        count += 1
        loop_rate.sleep()
