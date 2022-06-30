#!/usr/bin/env python3
"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-05-07 20:29:42
 # @ Modified time: 2022-05-19 10:37:34
 # @ Description: Generate desired trajectory for UAVs
 """


import argparse
import datetime
import math
import os
import sys
import time
from typing import List

import rosbag
import rospy
from geometry_msgs.msg import Point32, PolygonStamped, Pose, PoseStamped
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray
from tf.transformations import *

from crowd_model import CrowdModel
from utils import *

iris0_real_pose = None


def real_pose_cb(msg: PoseStamped):
    global iris0_real_pose
    iris0_real_pose = msg


def heart_curve_polar(r, theta, a, b, c):
    return (
        -(r**5) * np.sin(theta) ** 3 * np.cos(theta) ** 2
        + (a * r**2 * np.cos(theta) ** 2 + b * r**2 * np.sin(theta) ** 2 - c) ** 3
    )


def heart_curve_discrete_solve(theta, a, b, c, simu_step=0.01):
    """given theta, find the most approximate solution of r by discrete searching"""
    rs = np.arange(0, 2 * c, simu_step)
    sol = -1
    min_value = np.inf
    for r in rs:
        func_value = abs(heart_curve_polar(r, theta, a, b, c))  # values that are close to 0 are better
        if func_value < min_value:
            min_value = func_value
            sol = r
    return sol, min_value


def get_heart_com_curve(a, b, c, step=0.1):
    """get complete heart curve from 0 to 2pi"""
    theta = np.arange(0, 2 * np.pi, step)
    r = []
    min_value = []
    for i in theta:
        r_, min_value_ = heart_curve_discrete_solve(i, 1, 1, 1, step / 10)
        r.append(r_)
        min_value.append(min_value_)
    pass
    x = []
    y = []
    yaw = []
    for i in range(len(theta)):
        x.append(r[i] * np.cos(theta[i]))
        y.append(r[i] * np.sin(theta[i]))
        yaw.append(np.arctan2(y[i], x[i]))
    return x, y, yaw


def get_heart_curve(a, b, c, theta, simu_step=0.1):
    r, _ = heart_curve_discrete_solve(theta, a, b, c, simu_step)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    yaw = np.arctan2(y, x)
    return x, y, yaw


def gen_traj(shape, scale, height, step) -> Path:
    """generate trajectory

    Args:
        shape (str): shape of trajectory, option: circle, sin, square, eight
        scale (float): scale factor of trajectory. circle: radius, sin: amplitude
        step (float): step size of trajectory

    Returns:
        Path: trajectory in Path format
    """
    T = step
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()
    sim_time = 30
    iter = 0

    time2theta = lambda t: 2 * math.pi / 30 * t
    shapes = {
        # x,y,yaw
        "circle": lambda t, scale: (
            scale * math.cos(time2theta(t)),
            scale * math.sin(time2theta(t)),
            time2theta(t) + math.pi / 2,
        ),
        "sin": lambda t, scale: (
            1 / 3 * t,
            -scale * math.cos(time2theta(t)),
            math.atan2(2 * math.pi / 30 * scale * math.sin(time2theta(t)), 1 / 3),
        ),
        "square": lambda t, scale: (
            scale * math.cos(time2theta(t)),
            scale * math.sin(time2theta(t)),
            time2theta(t),
        ),
        "eight": lambda t, scale: (
            scale * math.cos(time2theta(t)),
            scale * math.sin(4 * math.pi / 30 * t),
            time2theta(t),
        ),
        "heart": lambda t, scale: get_heart_curve(1, 1, scale, time2theta(t), 0.05),
    }
    while sim_time - iter * T >= 0:
        t_predict = T * iter

        x_ref, y_ref, q_ref = shapes[shape](t_predict, scale)

        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x_ref
        pose.pose.position.y = y_ref
        pose.pose.position.z = height
        quat = quaternion_from_euler(0, 0, q_ref)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        path.poses.append(pose)

        iter += 1
    return path


def get_nearest_point_idx(path: Path, curr_pos: Pose, shape="circle") -> int:
    min_distance = float("inf")
    nearest_index = 0
    path = path.poses
    if shape != "circle":
        for i, point in enumerate(path):
            distance = euclidean_distance(curr_pos, point)
            if distance < min_distance:
                min_distance = distance
                nearest_index = i
    else:
        # use centre to find the nearest point
        point1 = path[0].pose
        point2 = path[len(path) // 2].pose
        centre = Pose()
        centre.position.x = (point1.position.x + point2.position.x) / 2
        centre.position.y = (point1.position.y + point2.position.y) / 2
        vec = np.array([curr_pos.position.x - centre.position.x, curr_pos.position.y - centre.position.y])
        theta = math.atan2(vec[1], vec[0])
        if theta < 0:
            theta += 2 * math.pi
        nearest_index = int(theta / (2 * math.pi) * len(path))
    return nearest_index


def get_ref_path(path: Path, curr_pos: PoseStamped, T: float, dt: float) -> List[Pose]:
    """get MPC ref trajectory from the whole path

    Args:
        curr_pos (Pose): use current position to find the nearest point
        T (float): MPC horizon length
        dt (float): MPC time step
    """
    nearest_index = get_nearest_point_idx(path, curr_pos.pose, shape="circle")
    ref_path = Path()
    for t in np.arange(0, T + dt, dt):
        future_index = nearest_index + int(t / dt)
        if future_index < len(path.poses):
            ref_path.poses.append(path.poses[future_index])
        else:
            # ref_path.poses.append(path.poses[-1])
            idx = future_index % len(path.poses)
            ref_path.poses.append(path.poses[idx])
    return ref_path


def draw_path(path: Path, color: str = "r"):
    """draw 3d path using matplotlib

    Args:
        path (List[Pose]): path to draw
        color (str, optional): color of path. Defaults to "r".
    """
    fig = plt.figure()
    path = path.poses
    ax = fig.add_subplot(111, projection="3d")
    x = []
    y = []
    z = []
    for pose in path:
        x.append(pose.pose.position.x)
        y.append(pose.pose.position.y)
        z.append(pose.pose.position.z)
    ax.scatter(x, y, z, color=color, s=0.5)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()


def add_path2fig(ax: Axes3D, path: Path, color: str = "r", size=0.5):
    x = []
    y = []
    z = []
    path = path.poses
    for pose in path:
        x.append(pose.pose.position.x)
        y.append(pose.pose.position.y)
        z.append(pose.pose.position.z)
    ax.scatter(x, y, z, color=color, s=size)


def get_crowd_model_from_path(
    path: Path,
    net_traf_ave,
    net_traf_std,
    traffic_mutation_prob,
    net_traf_mutation_ar,
):
    """set target network capacity for each point in path"""
    crowd_model = CrowdModel(
        net_traf_ave=net_traf_ave,
        net_traf_std=net_traf_std,
        traffic_mutation_prob=traffic_mutation_prob,
        net_traf_mutation_ar=net_traf_mutation_ar,
    )
    net_cap = []
    for pose in path.poses:
        net_cap.append(crowd_model.step_traffic())
    return path, net_cap


def gen_regular_polygon_vertices(num_vertices, area_size):
    radius = math.sqrt((4 * area_size) / (num_vertices * math.sin(2 * math.pi / num_vertices)))

    vertices = []
    for i in range(num_vertices):
        angle = 2 * math.pi * i / num_vertices
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        vertices.append((x, y))

    return vertices


def cal_polugon_area(vertices):
    """calculate area of a irregular polygon using shoelace formula"""
    area = 0
    for i in range(len(vertices)):
        area += vertices[i][0] * vertices[(i + 1) % len(vertices)][1]
        area -= vertices[i][1] * vertices[(i + 1) % len(vertices)][0]
    area = abs(area) / 2
    return area


if __name__ == "__main__":
    argparse = argparse.ArgumentParser()
    argparse.add_argument("--mode", type=str, default="dynamic_formation", help="traj, waypoint")
    argparse.add_argument("--shape", type=str, default="circle", help="circle, sin, square, eight")
    argparse.add_argument("--scale", type=float, default=20, help="scale factor of trajectory")
    argparse.add_argument("--height", type=float, default=5, help="height of trajectory")
    argparse.add_argument("--step", type=float, default=3, help="step size of trajectory")
    args = argparse.parse_args()
    mode = args.mode
    print("mode: %s" % mode)

    if 0:  # test basic path generation function
        start_time = time.time()
        path = gen_traj("heart", 5, 5, 0.01)
        local_ref_path = get_ref_path(path, path.poses[0], 10, 0.01)
        print("delta time: %.2f" % (time.time() - start_time))
        # draw_path(path, "r")
        fig = plt.figure()
        ax = plt.subplot(111, projection="3d")
        add_path2fig(ax, path, "r", size=0.1)
        add_path2fig(ax, local_ref_path, "b", size=1)
        plt.show()

    if 0:  # test local path generation
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        path = gen_traj("circle", 4, 0.1)
        add_path2fig(ax, path.poses, size=0.5)
        curr_pose = PoseStamped()
        curr_pose.pose.position.x = 2
        curr_pose.pose.position.y = 5
        add_path2fig(ax, [curr_pose], "b", size=10)

        idx = get_nearest_point_idx(path, curr_pose.pose, shape="circle")
        nearest_pose = path.poses[idx]
        add_path2fig(ax, [nearest_pose], "g", size=10)

        ref_path = get_ref_path(path, curr_pose, 10, 0.1)
        add_path2fig(ax, ref_path.poses, "y", size=10)

        plt.show()

    if mode == "traj":  # publish trajectory
        rospy.init_node("traj_node", anonymous=False)
        rate = 10
        waypoint_pub = rospy.Publisher("/iris_0/target_traj", Path, queue_size=1)
        r = rospy.Rate(rate)
        # path = path_generator("circle", 15, 5, 0.01)
        path = gen_traj("heart", 30, 5, 0.1)
        while not rospy.is_shutdown():
            waypoint_pub.publish(path)
            r.sleep()

    if mode == "waypoint":  # publish waypoints
        rospy.init_node("traj_node", anonymous=False)
        rate = 10
        path_pub_rate = 0.5
        crowd_model = CrowdModel(
            init_pos=[0, 0],
            net_traf_ave=40,
            net_traf_std=0.5,
            net_traf_max=100,
            net_traf_min=0.1,
            net_traf_mutation_prob=0.1,
            net_traf_mutation_ar=np.concatenate((np.zeros(2), [10, 20, 30])),
        )
        path = gen_traj(
            args.shape,
            args.scale,
            args.height,
            args.step,
        )
        # draw_path(path, "r")
        print(f"{'mode':<25}: {args.mode}")
        print(f"{'shape':<25}: {args.shape}")
        print(f"{'scale':<25}: {args.scale}")
        print(f"{'height':<25}: {args.height}")
        print(f"{'step':<25}: {args.step}")
        print(f"{'net_traf_ave':<25}: {crowd_model.net_traf_ave}")
        print(f"{'net_traf_std':<25}: {crowd_model.net_traf_std}")
        print(f"{'net_traf_mutation_prob':<25}: {crowd_model.net_traf_mutation_prob}")
        print(f"{'net_traf_mutation_ar':<25}: {crowd_model.net_traf_mutation_ar}")
        waypoint_pub = rospy.Publisher("/iris_0/target_pose", PoseStamped, queue_size=1, latch=True)
        tgt_cap_pub = rospy.Publisher("/crowds/target_net_capacity", Float32MultiArray, queue_size=1, latch=True)
        real_pose_sub = rospy.Subscriber("/iris_0/real_pose", PoseStamped, real_pose_cb)
        while iris0_real_pose is None:
            rospy.loginfo("Waiting for real pose...")
            rospy.sleep(0.1)
        nearest_index = get_nearest_point_idx(path, iris0_real_pose.pose, shape="circle")  # find nearest point
        rospy.loginfo("Nearest point index: %d" % nearest_index)
        r = rospy.Rate(rate)
        idx = nearest_index
        cnt = 0
        rospy.loginfo("Start publishing waypoints...")
        next_pose = path.poses[idx]
        while not rospy.is_shutdown():
            dist2next_pose = euclidean_distance(iris0_real_pose.pose, next_pose.pose)
            if dist2next_pose < 3:
                idx = (idx + 1) % len(path.poses)
                next_pose = path.poses[idx]
                next_pose.header.stamp = rospy.Time.now()
                set_pose_msg_orien(next_pose.pose, [0, 0, 0, 1])
            waypoint_pub.publish(next_pose)
            tgt_cap = crowd_model.step_traffic()
            tgt_cap_pub.publish(Float32MultiArray(data=[tgt_cap]))
            cnt += 1
            r.sleep()

    if mode == "dynamic_formation":
        rospy.init_node("traj_node", anonymous=False)
        sim_time = 300
        write2bag = True
        uav_num = 4
        base_freq = 10
        mobility_step_freq = 0.5
        net_cap_step_freq = 10
        area = 25
        height = 5
        dirs = np.arange(0, 2 * np.pi, 2 * np.pi / uav_num)
        init_poses = gen_regular_polygon_vertices(uav_num, area)
        vertices = []
        alphas = [0.8, 0.5, 0.1]  # randomly select alpha from this list according to alpha_mut_probs
        alpha_mut_probs = 0.2  # alpha mutation probability
        dir_mut_prob = 0.75  # reverse direction probability
        only_scale_prob = 0.01
        only_scale_flag = False
        if write2bag:
            time_suffix = datetime.datetime.now().strftime("%m-%d-%H%M%S")
            bagfile = os.path.join(sys.path[0], f"../data/rosbag/dynamic_formation_u{uav_num}_{time_suffix}.bag")
            bagfile_alia = os.path.join(sys.path[0], f"../data/rosbag/dynamic_formation_u{uav_num}_latest.bag")
            bag = rosbag.Bag(bagfile, "w")
        print(f"{'uav_num':<25}: {uav_num}")
        print(f"{'base_freq':<25}: {base_freq}")
        print(f"{'mobility_step_freq':<25}: {mobility_step_freq}")
        print(f"{'net_cap_step_freq':<25}: {net_cap_step_freq}")
        print(f"{'write2bag':<25}: {write2bag}")
        print(f"{'sim_time':<25}: {sim_time}")
        print(f"{'area':<25}: {area}")
        print(f"{'height':<25}: {height}")
        print(f"{'dirs':<25}: {dirs}")
        print(f"{'alphas':<25}: {alphas}")
        print(f"{'alpha_mut_probs':<25}: {alpha_mut_probs}")
        print(f"{'dir_mut_prob':<25}: {dir_mut_prob}")
        print(f"{'only_scale_prob':<25}: {only_scale_prob}")

        if 0:  # plot initial formation
            plt.scatter([init_poses[i][0] for i in range(uav_num)], [init_poses[i][1] for i in range(uav_num)], s=10)
            # plot dirs as arrows
            for i in range(uav_num):
                plt.arrow(
                    0,
                    0,
                    2 * np.cos(dirs[i]),
                    2 * np.sin(dirs[i]),
                    head_width=0.5,
                    head_length=1,
                    fc="k",
                    ec="k",
                )
            plt.show()

        for i in range(uav_num):
            vertices.append(
                CrowdModel(
                    init_pos=init_poses[i],
                    init_speed=0.1,
                    init_dir=dirs[i],
                    ave_speed=1,
                    speed_alpha=0.1,  # 0.1
                    ave_dir=dirs[i],
                    dir_alpha=0.5,  # 0.8
                    net_traf_ave=40,
                    net_traf_std=0.5,
                    net_traf_max=100,
                    net_traf_min=0.1,
                    net_traf_mutation_prob=0.1,
                    net_traf_mutation_ar=np.concatenate((np.zeros(2), [10, 20, 30])),
                )
            )

        rate = rospy.Rate(base_freq)
        # pubs
        form_pub = rospy.Publisher("/crowds/formation", PolygonStamped, queue_size=1, latch=True)
        cap_pub = rospy.Publisher("/crowds/target_net_capacity", Float32MultiArray, queue_size=1, latch=True)
        uav_tgt_pose_pubs = []
        for i in range(uav_num):
            uav_tgt_pose_pubs.append(rospy.Publisher(f"/iris_{i}/target_pose", PoseStamped, queue_size=1, latch=True))
        enable_ctrl_pub = rospy.Publisher("/enable_simu", Bool, queue_size=1, latch=True)

        np.random.seed(int(time.time()))
        start_time = time.time()
        enable_ctrl_pub.publish(Bool(data=True))
        rospy.loginfo("Start publishing formaiton...")
        cnt = 0
        while not rospy.is_shutdown() and time.time() - start_time < sim_time:
            print(f"\rtime passed: {time.time() - start_time:.2f}", end="")
            polygen_msg = PolygonStamped()
            formation = []
            capacity = []
            if np.random.rand() < only_scale_prob:
                only_scale_flag = not only_scale_flag
                # print(f"[{time.time():<14}] mutated to only scaling") if only_scale_flag else print(
                #     f"[{time.time():<14}] mutated to not only scaling"
                # )
            for i in range(uav_num):
                if cnt % int(base_freq / mobility_step_freq) == 0:
                    if only_scale_flag:
                        vertices[i].dir_alpha = 1
                        vertices[i].speed_alpha = 1
                        vertices[i].speed = 0.2
                    else:
                        vertices[i].dir_alpha = 0.8
                        vertices[i].speed_alpha = 0.1
                        vertices[i].ave_speed = 1
                        vertices[i].dir *= np.random.choice([-1, 1])
                    pose = vertices[i].step_mobility()
                    formation.append(pose)
                if cnt % int(base_freq / net_cap_step_freq) == 0:
                    cap = vertices[i].step_traffic()
                    capacity.append(cap)

            if cnt % int(base_freq / mobility_step_freq) == 0:
                formation = np.array(formation)
                # formation = formation - np.mean(formation, axis=0)  # move to origin
                for i, pose in enumerate(formation):
                    polygen_msg.header.stamp = rospy.Time.now()
                    polygen_msg.header.frame_id = "world"
                    polygen_msg.polygon.points.append(Point32(x=pose[0], y=pose[1], z=height))

                    uav_pose_msg = PoseStamped()
                    uav_pose_msg.header.stamp = rospy.Time.now()
                    uav_pose_msg.header.frame_id = f"iris_{i}"
                    set_pose_msg(uav_pose_msg.pose, [pose[0], pose[1], height], [0, 0, 0, 1])
                    uav_tgt_pose_pubs[i].publish(uav_pose_msg)
                    if write2bag:
                        bag.write(f"/iris_{i}/target_pose", uav_pose_msg)
                form_pub.publish(polygen_msg)
            if cnt % int(base_freq / net_cap_step_freq) == 0:
                cap_pub.publish(Float32MultiArray(data=capacity))
                if write2bag:
                    bag.write("/crowds/target_net_capacity", Float32MultiArray(data=capacity))
            cnt += 1
            rate.sleep()
        if write2bag:
            bag.close()
            print(f"\nBag file saved to {bagfile}")
            os.system(f"rm {bagfile_alia}")
            os.system(f"ln -s {bagfile} {bagfile_alia}")
