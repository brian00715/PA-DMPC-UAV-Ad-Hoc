#!/usr/bin/env python3
"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-05-13 16:00:48
 # @ Modified time: 2022-05-13 16:33:43
 # @ Description: Simultaneously activate controllers for all UAVs. 
 # You can indicate the UAV number and whether to use rviz using commandline arguments. 
 
 # If you intend to set a target pose via rviz's "3D Nav Goal" plugin, 
 # you will need to remap the topic. Simply append "--remap_target_topic" to the commandline arguments.
 """


from argparse import ArgumentParser

import roslaunch
import rospy

positions = [
    (0, 3, 2),
    (3, 3, 2),
    (6, 3, 2),
    (0, 6, 2),
    (3, 6, 2),
    (6, 6, 2),
    (0, 9, 2),
    (3, 9, 2),
    (6, 9, 2),
]


def launch_uav_controller_node(uav_idx, remap_target_topic=False, controller="nmpc", ctrl_rate=20,follow_mode="waypoint"):
    global positions
    package = "dmpc_uav_ad_hoc"
    executable = "uav_controller_node.py"
    node_name = f"uav_controller_node"
    node = roslaunch.core.Node(
        package,
        executable,
        name=node_name,
        namespace=f"iris_{uav_idx}",
        output="screen",
        args=f"""_uav_idx:={uav_idx} _x:={positions[uav_idx][0]} _y:={positions[uav_idx][1]} _z:={positions[uav_idx][2]}
        _controller:={controller} _ctrl_rate:={ctrl_rate} _follow_mode:={follow_mode}""",
        # if use rviz to specify target pose
        remap_args=[("/iris_0/target_pose", "/rviz/3d_goal")] if remap_target_topic else None,
    )
    return node


def launch_rviz_node():
    package = "rviz"
    executable = "rviz"
    node_name = f"rviz_node"
    node = roslaunch.core.Node(
        package,
        executable,
        name=node_name,
        output="screen",
        args=f"-d $(find dmpc_uav_ad_hoc)/rviz/swarm_controller.rviz",
    )
    return node


if __name__ == "__main__":
    arg_parser = ArgumentParser()
    arg_parser.add_argument("--rviz", action="store_true")
    arg_parser.add_argument("--remap_target_topic", action="store_true")
    arg_parser.add_argument("--uav_num", type=int, default=3)
    arg_parser.add_argument("--controller", type=str, default="nmpc", choices=["nmpc", "px4"])
    arg_parser.add_argument("--follow_mode", type=str, default="waypoint", choices=["path", "waypoint"])
    # arg_parser.add_argument("--ctrl_rate", type=float, default=20)
    args = arg_parser.parse_args()

    rospy.init_node("swarm_controller_node", anonymous=False)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    for uav_idx in range(args.uav_num):
        node = launch_uav_controller_node(
            uav_idx,
            args.remap_target_topic,
            controller=args.controller,
            follow_mode=args.follow_mode,
        )
        launch.launch(node)
    if args.rviz:
        node = launch_rviz_node()
        launch.launch(node)
    rospy.spin()
