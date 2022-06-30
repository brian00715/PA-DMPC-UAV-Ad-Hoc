#!/usr/bin/env python
"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-05-17 17:45:35
 # @ Modified time: 2022-05-17 20:44:43
 # @ Description:
 """


import time
import xml.etree.ElementTree as ET

import roslaunch
import rospy

fcu_base_port1 = 24540
fcu_base_port2 = 34580
base_mavlink_tcp_port = 4560
base_mavlink_udp_port = 18570


def launch_uav(
    uav_id,
    vehicle,
    x,
    y,
    z,
    R,
    P,
    Y,
):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Launch single_vehicle_spawn_xtd.launch for UAV spawn
    mavlink_udp_port = base_mavlink_udp_port + uav_id
    mavlink_tcp_port = base_mavlink_tcp_port + uav_id
    fcu_port1 = fcu_base_port1 + uav_id
    fcu_port2 = fcu_base_port2 + uav_id
    fcu_url = "udp://:{}@localhost:{}".format(fcu_port1, fcu_port2)

    spawn_cli_args = [
        "dmpc_uav_ad_hoc",
        "single_uav_spawn.launch",
        "x:={}".format(x),
        "y:={}".format(y),
        "z:={}".format(z),
        "R:={}".format(R),
        "P:={}".format(P),
        "Y:={}".format(Y),
        f"vehicle:={vehicle}",
        "sdf:=iris",
        "mavlink_udp_port:={}".format(mavlink_udp_port),
        "mavlink_tcp_port:={}".format(mavlink_tcp_port),
        "ID:={}".format(uav_id),
        "ID_in_group:={}".format(uav_id),
        f"namespace:={vehicle}_{uav_id}",
        f"fcu_url:={fcu_url}",
    ]
    spawn_args = spawn_cli_args[2:]
    spawn_launch_file = roslaunch.rlutil.resolve_launch_arguments(spawn_cli_args)[
        0
    ]  # get the complete path of the launch file

    launch_files = [(spawn_launch_file, spawn_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files, verbose=True)
    parent.start()


if __name__ == "__main__":
    rospy.init_node("launch_uavs", anonymous=True)

    uav_num = rospy.get_param("~uav_num", 1)

    positions = [
        (0, 3, 0.5),
        (3, 3, 0.5),
        (6, 3, 0.5),
        (0, 6, 0.5),
        (3, 6, 0.5),
        (6, 6, 0.5),
        (0, 9, 0.5),
        (3, 9, 0.5),
        (6, 9, 0.5),
    ]

    postures = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]

    for i in range(uav_num):
        uav_id = i
        x, y, z = positions[i]
        R, P, Y = postures[i]
        launch_uav(uav_id, "iris", x, y, z, R, P, Y)
    rospy.spin()
