#!/usr/bin/env python
import math
import time

import tf
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from gazebo_msgs.msg import ModelStates

import rospy

local_pose = PoseStamped()
gazebo_pose = PoseStamped()
tf_broadcaster = tf.TransformBroadcaster()


def local_pose_callback(msg: PoseStamped):
    global local_pose, tf_broadcaster
    local_pose = msg
    tf_broadcaster.sendTransform(
        (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w),
        rospy.Time.now(),
        "local_pose",
        "map"
    )


def model_states_callback(msg: ModelStates):
    global tf_broadcaster, gazebo_pose
    # 获取模型位姿列表
    model_poses = msg.pose
    gazebo_pose = model_poses[2]

    # 发布模型位姿到 tf
    tf_broadcaster.sendTransform(
        (model_poses[2].position.x, model_poses[2].position.y,
         model_poses[2].position.z),
        (model_poses[2].orientation.x, model_poses[2].orientation.y,
         model_poses[2].orientation.z, model_poses[2].orientation.w),
        rospy.Time.now(),
        "gazebo_pose",
        "map"
    )


def control_attitude():
    global local_pose, tf_broadcaster, gazebo_pose
    rospy.init_node('control_attitude', anonymous=True)
    pub = rospy.Publisher(
        '/iris_0/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    model_states_sub = rospy.Subscriber(
        "/gazebo/model_states", ModelStates, model_states_callback)
    # pub = rospy.Publisher('/iris_0/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=1)
    real_pose_sub = rospy.Subscriber(
        '/iris_0/mavros/local_position/pose', PoseStamped, queue_size=1, callback=local_pose_callback)
    set_mode_proxy = rospy.ServiceProxy('/iris_0/mavros/set_mode', SetMode)
    arm_proxy = rospy.ServiceProxy('/iris_0/mavros/cmd/arming', CommandBool)
    rate = rospy.Rate(50)  # 10hz

    thrust_duration = 2  # 推力保持的时间（秒）
    thrust_start_time = time.time()  # 推力保持的起始时间

    arm_proxy(value=True)
    target_euler = [math.radians(5), math.radians(5), 0]
    revert_start_time = time.time()
    while not rospy.is_shutdown():
        set_mode_proxy(custom_mode="OFFBOARD")
        now = time.time()
        attitude_msg = AttitudeTarget()
        attitude_msg.header.stamp = rospy.Time.now()
        if (now - thrust_start_time) >= thrust_duration:
            attitude_msg.thrust = 0.59
            if now - revert_start_time > 4:
                target_euler[1] = - target_euler[1]
                revert_start_time = time.time()
            target_quat = tft.quaternion_from_euler(
                target_euler[0], target_euler[1], target_euler[2])
            tf_broadcaster.sendTransform((gazebo_pose.position.x, gazebo_pose.position.y, gazebo_pose.position.z),
                                         (target_quat[0], target_quat[1],
                                          target_quat[2], target_quat[3]),
                                         rospy.Time.now(),
                                         "target_quat", "map")
            attitude_msg.orientation.x = target_quat[0]
            attitude_msg.orientation.y = target_quat[1]
            attitude_msg.orientation.z = target_quat[2]
            attitude_msg.orientation.w = target_quat[3]
            rospy.loginfo_throttle(
                1.0, f"adjusting, pose:{[x for x in target_quat]}")

        else:
            # rospy.loginfo_throttle(1.0,"lifting")
            attitude_msg.thrust = 0.6
            attitude_msg.orientation.x = 0.0
            attitude_msg.orientation.y = 0.0
            attitude_msg.orientation.z = 0.0
            attitude_msg.orientation.w = 0.0
        pub.publish(attitude_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        control_attitude()
    except rospy.ROSInterruptException:
        pass
