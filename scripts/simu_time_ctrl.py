#!/usr/bin/env python
"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-05-26 15:22:26
 # @ Modified time: 2022-05-26 21:19:55
 # @ Description: script to control simulation time
 """


import os
import sys
import time
from multiprocessing import Process

import rospy
from std_msgs.msg import Bool
from tqdm import tqdm
import socket
import argparse
import rosbag
from rosgraph_msgs.msg import Clock

ros_time = 0


def play_rosbag(bag_path, play_rate=1):
    os.system(f"rosbag play {bag_path} -r {play_rate} > /dev/null 2>&1")


if __name__ == "__main__":
    rospy.init_node("simu_time_ctrl", anonymous=False)
    rosbag_path = rospy.get_param(
        "~rosbag_path", "/home/simon/wbc_ws/src/dmpc_uav_ad_hoc/data/crowd_simu_u1_c1_08-26-203529.bag"
    )
    simu_time = rospy.get_param("~simu_time", 300)  # s
    uav_num = rospy.get_param("~uav_num", 9)
    rosbag_path = os.path.join(sys.path[0], f"../data/rosbag/crowd_simu_u{uav_num}_c{uav_num}_latest.bag")
    # rosbag_path = os.path.join(sys.path[0], f"../data/rosbag/dynamic_formation_u{uav_num}_latest.bag")
    print(f"{'simu_time':<15}: {simu_time}")
    print(f"{'rosbag_path':<15}: {rosbag_path}")

    print(f"start testing realtime factor...")
    test_duration = 4
    factor_list = []
    for i in range(test_duration):
        ros_time1 = rospy.get_time()
        while ros_time1 < 1 and not rospy.is_shutdown():
            ros_time1 = rospy.get_time()
        time.sleep(1)
        ros_time2 = rospy.get_time()
        realtime_factor = ros_time2 - ros_time1
        factor_list.append(realtime_factor)
    realtime_factor = sum(factor_list) / len(factor_list)
    # realtime_factor = 1
    print(f"{'realtime_factor':<15}: {realtime_factor}")

    server_exist = True
    server_name = "localhost"
    server_port = 10086
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect((server_name, server_port))
        rospy.loginfo(f"connected to server {server_name}:{server_port}")
    except:
        print(f"upper server does not exist!")
        server_exist = False

    start_pub = rospy.Publisher("/enable_simu", Bool, queue_size=1, latch=True)
    start_time = rospy.get_time()
    # ROS time is published by Gazebo because setting use_sim_time:=true.
    # so we need to wait for the first /clock message to be published.
    while start_time < 1 and not rospy.is_shutdown():
        start_time = rospy.get_time()
        time.sleep(0.1)
    print(f"simulation started: {start_time}")
    start_pub.publish(True)

    process = Process(
        target=play_rosbag,
        args=(
            rosbag_path,
            realtime_factor,
        ),
    )
    process.start()

    loop_rate = rospy.Rate(10)
    pbar = tqdm(total=simu_time, desc="simu progess", unit="s")
    while not rospy.is_shutdown():
        delta_time = rospy.get_time() - start_time
        pbar.update(int(delta_time) - pbar.n)
        if server_exist:
            client_socket.send("0".encode())
        if rospy.get_time() - start_time > simu_time:
            print(f"\r\nsimulation finished: {rospy.get_time()}")
            start_pub.publish(False)
            time.sleep(0.1)
            start_pub.publish(False)
            process.kill()
            os.system("kill -9 $(pgrep -f rosbag)")
            time.sleep(0.1)
            process.close()
            if server_exist:
                client_socket.send("1".encode())
                while True:
                    try:
                        recv = str(client_socket.recv(1024), encoding="utf-8", timeout=3)
                    except:
                        rospy.logerr(f"recv timeout, resend...")
                        client_socket.send("1".encode())
                        time.sleep(0.1)
                        continue
                    if recv == "1":
                        rospy.loginfo(f"received echo from server, stop...")
                        break
                    else:
                        rospy.logerr(f"not recv 1 from server, resend...")
                        client_socket.send("1".encode())
                        time.sleep(0.1)
                        continue
            break
        loop_rate.sleep()
