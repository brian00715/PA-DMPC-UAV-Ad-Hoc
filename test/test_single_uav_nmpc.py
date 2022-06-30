#!/usr/bin/env python
'''
 # @ Author: Kenneth Simon
 # @ Create Time: 2022-05-08 22:02:27
 # @ Email: smkk00715@gmail.com
 # @ Modified time: 2022-05-10 17:50:53
 # @ Description: Test NMPC for single UAV
 '''


import rospy
import sys

import ipdb
import numpy as np
import tf
import tf.transformations as tft
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import (Point, Pose, PoseArray, PoseStamped,
                               TwistStamped, Vector3)
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
sys.path.append("/home/simon/wbc_ws/src/dmpc_uav_ad_hoc/src")

uav_pose = None
uav_vel = None
tgt_pose = PoseStamped()
motor_constant = 8.54858e-06
input_scale = 1100
Q = np.diag([10.0, 10.0, 20.0, 1.0, 1.0, 1.0])
R = np.diag([1, 1, 1, 1])


def nmpc_matrix_q_callback(msg: Float32MultiArray):
    global Q
    Q = np.diag(msg.data)
    print(f"got Q:\r\n {Q}")


def nmpc_matrix_r_callback(msg: Float32MultiArray):
    global R
    R = np.diag(msg.data)
    print(f"got R:\r\n {R}")


def model_states_callback(msg: ModelStates):
    global uav_pose, uav_vel
    uav_pose = msg.pose[2]
    uav_vel = msg.twist[2]


def pose_callback(msg: PoseStamped):
    global uav_pose
    uav_pose = msg


def vel_callback(msg: PoseStamped):
    global uav_vel
    uav_vel = msg


def tgt_pose_callback(msg: PoseStamped):
    global tgt_pose
    tgt_pose = msg
    tgt_pose.header.frame_id = "map"
    print(
        f"got pose: {msg.pose.position.x,msg.pose.position.y,msg.pose.position.z}")


def thrust2signal(thrust: float, motor_constant, input_scale):
    return np.sqrt(thrust/motor_constant/4)/input_scale


def viz_opti_traj(opti_traj: np.array, opti_u: np.array):
    marker_pub = rospy.Publisher('opti_traj', MarkerArray, queue_size=10)
    marker_array = MarkerArray()
    for i, speed in enumerate(opti_traj):
        marker = Marker()
        marker.header.frame_id = "map"  # 设置坐标系
        marker.header.stamp = rospy.Time.now()
        marker.ns = "speed_visualization"
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = Point(speed[0], speed[1], speed[2])

        if i < len(opti_u):
            quat = tft.quaternion_from_euler(
                opti_u[i][1], opti_u[i][2], opti_u[i][3])
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]

        # marker.scale = Vector3(speed[3], speed[4], speed[5])  # 设置箭头的大小
        marker.scale = Vector3(0.1, 0.01, 0.01)
        marker.color.a = 1.0-(i/len(opti_traj))
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)
    marker_pub.publish(marker_array)

    # pose_pub = rospy.Publisher('opti_traj', PoseArray, queue_size=1)
    # poses = PoseArray()
    # poses.header.frame_id = "iris_0"
    # for i, speed in enumerate(opti_traj):
    #     pose = Pose()
    #     pose.orientation.w = 1.0
    #     pose.position = Point(speed[0], speed[1], speed[2])
    #     poses.poses.append(pose)
    # pose_pub.publish(poses)


def inertial2NED(state_in_inertial):
    """transfer pose in intertial frame to NED frame"""
    return np.array([state_in_inertial[1], state_in_inertial[0], -state_in_inertial[2],
                     state_in_inertial[4], state_in_inertial[3], -state_in_inertial[5]])


def NED2inertial(state_in_NED):
    """transfer pose in NED frame to intertial frame"""
    return np.array([state_in_NED[1], state_in_NED[0], -state_in_NED[2],
                     state_in_NED[4], state_in_NED[3], -state_in_NED[5]])


if __name__ == "__main__":
    rospy.init_node("test_nmpc")

    # subs
    # pose_sub = rospy.Subscriber(
    #     "/iris_0/mavros/local_position/pose", PoseStamped, pose_callback)
    # speed_sub = rospy.Subscriber(
    #     "/iris_0/mavros/local_position/velocity", TwistStamped, vel_callback)
    model_states_sub = rospy.Subscriber(
        "/gazebo/model_states", ModelStates, model_states_callback, queue_size=1)
    tgt_pose_sub = rospy.Subscriber(
        "/rviz/3d_goal", PoseStamped, tgt_pose_callback, queue_size=3)
    npmc_matrix_q_sub = rospy.Subscriber("/nmpc_matrix/Q", Float32MultiArray, nmpc_matrix_q_callback, queue_size=1)
    npmc_matrix_r_sub = rospy.Subscriber("/nmpc_matrix/R", Float32MultiArray, nmpc_matrix_r_callback, queue_size=1)

    # pubs
    attitude_pub = rospy.Publisher('/iris_0/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    xtdrone_cmd_pub = rospy.Publisher('/xtdrone/iris_0/cmd', String, queue_size=3)
    test_data_pub = rospy.Publisher("test_data", Float32MultiArray, queue_size=1)
    nmpc_ctrl_pub = rospy.Publisher("nmpc_ctrl", Float32MultiArray, queue_size=1)
    tf_pub = tf.broadcaster.TransformBroadcaster()

    # srvs
    set_mode_proxy = rospy.ServiceProxy('/iris_0/mavros/set_mode', SetMode)
    arm_proxy = rospy.ServiceProxy('/iris_0/mavros/cmd/arming', CommandBool)

    rate = 30
    loop_rate = rospy.Rate(rate)

    from nmpc_controller import NMPCC
    from pid_controller import PIDC
    mpcc = NMPCC(np.array([0, 0, 0, 0, 0, 0]), 35,np.radians(30),
                 N=20, T=1/rate*2, Q=np.diag([10.0, 10.0, 20.0, 1.0, 1.0, 1.0]), R=np.diag([1, 1, 1, 1]))
    pidc = PIDC(0.2, 1, 10, 0.3, 0.4)

    # initial pose
    tgt_pose.pose.position.x = 0
    tgt_pose.pose.position.y = 0
    tgt_pose.pose.position.z = 2.0
    tgt_pose.pose.orientation.w = 1
    u_ref = np.zeros((mpcc.N, 4))

    attitude_msg = AttitudeTarget()

    while uav_pose == None or uav_vel == None:
        continue

    arm_proxy(value=True)
    xtdrone_cmd_pub.publish("OFFBOARD")
    while not rospy.is_shutdown():
        set_mode_proxy(custom_mode="OFFBOARD")
        tf_pub.sendTransform((uav_pose.position.x, uav_pose.position.y, uav_pose.position.z), (uav_pose.orientation.x,
                             uav_pose.orientation.y, uav_pose.orientation.z, uav_pose.orientation.w), rospy.Time.now(), "iris_0", "map")
        x_ref = np.array([[tgt_pose.pose.position.x, tgt_pose.pose.position.y,
                         tgt_pose.pose.position.z, 0, 0, 0]]*(mpcc.N+1))
        # x_ref = generate_circle_points(1,5,mpcc.N+1)
        x_ref_delta = pidc.control(
            tgt_pose.pose.position.z, uav_pose.position.z)  # altitude compensate
        x_ref[:, 2] += x_ref_delta
        for i in range(len(x_ref)):
            x_ref[i] = inertial2NED(x_ref[i])
        x_curr = np.array([uav_pose.position.x, uav_pose.position.y, uav_pose.position.z,
                          uav_vel.linear.x, uav_vel.linear.y, uav_vel.linear.z])

        opt_u_raw, opt_x = mpcc.solve(inertial2NED(
            x_curr), x_ref, u_ref, return_first_u=False, Q=Q)  # XXX: core step
        opt_u = opt_u_raw[0]
        opt_u = [opt_u[0], opt_u[2], opt_u[1], -opt_u[3]]
        thrust = opt_u[0]
        throttle = thrust2signal(thrust, motor_constant, input_scale)

        att_quad = tft.quaternion_from_euler(opt_u[1], opt_u[2], opt_u[3])
        attitude_msg.orientation.x = att_quad[0]
        attitude_msg.orientation.y = att_quad[1]
        attitude_msg.orientation.z = att_quad[2]
        attitude_msg.orientation.w = att_quad[3]
        attitude_msg.thrust = throttle
        attitude_pub.publish(attitude_msg)

        for i in range(len(opt_x)):
            opt_x[i] = NED2inertial(opt_x[i])
        nmpc_ctrl_pub.publish(Float32MultiArray(data=opt_u))
        viz_opti_traj(opt_x, opt_u_raw)
        # print(
        #     f"opt_u:{thrust:.2f},throttle: {throttle:.2f},{opt_u[1]:.2f},{opt_u[2]:.2f},{opt_u[3]:.2f} pose: {x_curr[0]:.2f},{x_curr[1]:.2f},{x_curr[2]:.2f}")
        loop_rate.sleep()
    xtdrone_cmd_pub.publish("HOVER")
