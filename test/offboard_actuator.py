#!/usr/bin/env python
import rospy
from mavros_msgs.msg import ActuatorControl



if __name__ == '__main__':
    try:
        rospy.init_node('control_motor_speed', anonymous=True)
        pub = rospy.Publisher('/iris_0/mavros/actuator_control', ActuatorControl, queue_size=10)
        actuator_control_msg = ActuatorControl()
        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            # 发布修改后的控制指令
            actuator_control_msg.controls[0]=0.6
            pub.publish(actuator_control_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass