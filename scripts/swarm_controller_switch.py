#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

all_models_ready = False


def all_models_ready_callback(msg):
    global all_models_ready
    if msg.data:
        rospy.loginfo("All models are ready")
        all_models_ready = True
    else:
        rospy.loginfo("All models are not ready")
        all_models_ready = False


def enable_ctrl(switch=False, uav_num=3):
    for i in range(uav_num):
        service_name = f"/iris_{i}/enable_ctrl"
        try:
            rospy.wait_for_service(service_name, timeout=10)
            enable_ctrl_service = rospy.ServiceProxy(service_name, SetBool)
            response = enable_ctrl_service(switch)
            if response.success:
                rospy.loginfo(f"Service call successful for {service_name}: {switch}")
            else:
                rospy.logwarn(f"Service call failed for {service_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to {service_name} failed: {e}")


def return_home(uav_num=3):
    for i in range(uav_num):
        service_name = f"/iris_{i}/return_home"
        try:
            rospy.wait_for_service(service_name, timeout=10)
            return_home_service = rospy.ServiceProxy(service_name, Trigger)
            response = return_home_service()
            if response.success:
                rospy.loginfo(f"Service call successful for {service_name}")
            else:
                rospy.logwarn(f"Service call failed for {service_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to {service_name} failed: {e}")


if __name__ == "__main__":
    rospy.init_node("control_iris_services")
    uav_num = rospy.get_param("~uav_num", 3)

    if len(rospy.myargv()) != 2:
        rospy.logerr("Invalid argument. Usage: rosrun your_package_name control_iris_services.py [on|off]")
        exit(1)

    all_models_ready_sub = rospy.Subscriber("/all_models_ready", Bool, all_models_ready_callback, queue_size=1)

    rate = rospy.Rate(10)
    while not all_models_ready:
        rospy.loginfo("Waiting for all models to be ready")
        rate.sleep()

    arg = rospy.myargv()[1]
    if arg == "on":
        enable_ctrl(True, uav_num)
    elif arg == "off":
        enable_ctrl(False, uav_num)
    elif arg == "return_home":
        return_home(uav_num)
    else:
        rospy.logerr("Invalid argument. Usage: rosrun your_package_name control_iris_services.py [on|off]")
        exit(1)
