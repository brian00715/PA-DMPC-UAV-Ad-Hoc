#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped, Pose
from std_msgs.msg import Float32, Float32MultiArray


def parse_array_input(input_string):
    chars = input_string.split(" ")
    arr = [float(char) for char in chars]
    return arr


def main_menu():
    print("CLI Menu:")
    print("1. Publish height to tgt_pose")
    print("2. Publish Q to nmpc_matrix_q")
    print("3. Publish R to nmpc_matrix_r")
    print("4. Publish cube poses to tgt_pose")
    print("0. Quit")


def cube_pose(idx):
    tgt_pose = {
        "0": PoseStamped(pose=Pose(position=Point(x=-50, y=-50, z=6.0)), header=rospy.Header(frame_id="world")),
        # "1": PoseStamped(pose=Pose(position=Point(x=3, y=3, z=7.0)), header=rospy.Header(frame_id="world")),
        "1": PoseStamped(pose=Pose(position=Point(x=-50, y=50, z=6.0)), header=rospy.Header(frame_id="world")),
        # "3": PoseStamped(pose=Pose(position=Point(x=3, y=-3, z=7.0)), header=rospy.Header(frame_id="world")),
    }
    return tgt_pose[str(idx % 2)]


if __name__ == "__main__":
    rospy.init_node("pub_tgt_pose")
    tgt_pose_pub = rospy.Publisher("/rviz/3d_goal", PoseStamped, queue_size=1)
    nmpc_matrix_q_pub = rospy.Publisher("/iris_0/matrix_q", Float32MultiArray, queue_size=1)
    nmpc_matrix_r_pub = rospy.Publisher("/iris_0/matrix_r", Float32MultiArray, queue_size=1)
    main_menu()
    choice = int(input("Enter your choice: "))
    idx = 0
    while not rospy.is_shutdown():
        if choice == 1:
            height = input("Enter a height: ")
            if height == "q":
                exit(0)
            print(f"entered: {height}")
            pose_msg = Pose()
            # pose_msg.position.x = float(input_value)
            # pose_msg.position.y = float(input_value)
            pose_msg.position.z = float(height)
            tgt_pose_pub.publish(pose_msg)
        elif choice == 2:
            Q = parse_array_input(input("input Q: "))
            msg = Float32MultiArray()
            msg.data = Q
            nmpc_matrix_q_pub.publish(msg)
            print(f"Q: {Q} published")
        elif choice == 3:
            R = parse_array_input(input("input R: "))
            msg = Float32MultiArray()
            msg.data = R
            nmpc_matrix_r_pub.publish(msg)
            print(f"R: {R} published")
        elif choice == 4:
            pose = cube_pose(idx)
            tgt_pose_pub.publish(pose)
            idx += 1
            print(f"next pose:{[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]}")
            input_char = input("continue? (q to quit): ")
            if input_char == "q":
                exit(0)
