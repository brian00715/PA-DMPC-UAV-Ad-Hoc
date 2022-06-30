#!/usr/bin/env python

"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-05-16 17:44:09
 # @ Modified time: 2022-05-16 17:44:46
 # @ Description: Scheduling module.
 """

import os
import sys
import threading

import matplotlib.pyplot as plt
import numpy as np
import rosparam
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from statsmodels.tsa.stattools import adfuller
from std_msgs.msg import Bool, Float32, Float32MultiArray

from dmpc_uav_ad_hoc.srv import GetMPCMatrix, SetController, SetMPCMatrix


class LowPassFilter:
    def __init__(self, alpha=0.5) -> None:
        self.x_filtered = None
        self.alpha = alpha

    def step(self, x_new):
        if self.x_filtered is None:
            self.x_filtered = x_new
            return self.x_filtered
        self.x_filtered = self.x_filtered * self.alpha + (1 - self.alpha) * x_new
        return self.x_filtered


class GlobalScheduler:
    """Scheduler for optimizing the MPC execution frequency and whether to use VFC algorithm"""

    def __init__(self, uav_num=3, loop_freq=10) -> None:
        self.uav_num = uav_num
        self.loop_freq = loop_freq
        self.uav_poses = np.zeros((self.uav_num, 3))  # current formation
        self.uav_poses_des = np.zeros((self.uav_num, 3))  # desired formation
        self.uav_poses_des_last = np.zeros((self.uav_num, 3))  # last desired formation
        # adjancency matrix
        self.matrix_adj = np.zeros((self.uav_num, self.uav_num))
        self.matrix_adj_des = np.zeros((self.uav_num, self.uav_num))
        # degree matrix
        self.matrix_deg = np.zeros(self.uav_num)
        self.matrix_deg_des = np.zeros(self.uav_num)
        # Symmetric Normalized Laplacian
        self.matrix_snl = np.zeros((self.uav_num, self.uav_num))
        self.matrix_snl_des = np.zeros((self.uav_num, self.uav_num))

        # params
        param_file = os.path.join(sys.path[0], "../config/scheduler_params.yaml")
        param_dict = rosparam.load_file(param_file)[0][0]["global_scheduler"]
        self.form_sim = 0  # instant formation similarity
        self.form_sim_ar = []
        self.form_sim_int_len = param_dict[
            "form_sim_int_len"
        ]  #  [seconds] window length for formation similarity integral
        self.form_sim_int_th = param_dict["form_sim_int_th"]  # formation similarity integral threshold
        self.tgt_pose_updated_flags = [False for _ in range(self.uav_num)]
        self.tgt_pose_pub_rate = param_dict["tgt_pose_pub_rate"]  # [hz]
        self.enable_simu_flag = False

        # subs
        self.uav_tgt_pose_subs = []
        self.real_pose_subs = []
        self.real_twist_subs = []
        self.uav_controller_srvs = []
        for i in range(uav_num):
            self.uav_tgt_pose_subs.append(
                rospy.Subscriber(f"/iris_{i}/target_pose", PoseStamped, callback=self.uav_tgt_pose_cb, queue_size=1)
            )
            self.real_pose_subs.append(
                rospy.Subscriber(f"/iris_{i}/real_pose", PoseStamped, self.real_pose_cb, queue_size=1)
            )
            rospy.wait_for_service(f"/iris_{i}/set_controller", rospy.Duration(3))
            self.uav_controller_srvs.append(rospy.ServiceProxy(f"/iris_{i}/set_controller", SetController))
        self.enable_simu_sub = rospy.Subscriber("/enable_simu", Bool, self.enable_simu_cb)
        # pubs
        self.form_sim_pub = rospy.Publisher("/global_scheduler/formation_similarity", Float32, queue_size=1)
        self.form_sim_int_pub = rospy.Publisher(
            "/global_scheduler/formation_similarity_integral", Float32, queue_size=1
        )
        self.form_polygon_pub = rospy.Publisher("/global_scheduler/formation_polygon", PolygonStamped, queue_size=1)
        self.form_polygon_des_pub = rospy.Publisher(
            "/global_scheduler/formation_polygon_des", PolygonStamped, queue_size=1
        )
        rospy.loginfo("Global scheduler initialized.")

    def enable_simu_cb(self, enable_msg: Bool):
        self.enable_simu_flag = enable_msg.data

    def real_pose_cb(self, pose_msg: PoseStamped):
        uav_name = pose_msg.header.frame_id
        idx_in_name = int(uav_name.find("iris_")) + 5
        idx = int(uav_name[idx_in_name])
        self.uav_poses[idx] = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]
        # print(f"{uav_name} pose: {self.uav_poses[idx]}")

    def uav_tgt_pose_cb(self, pose_msg: PoseStamped):
        uav_name = pose_msg.header.frame_id
        idx_in_name = int(uav_name.find("iris_")) + 5
        idx = int(uav_name[idx_in_name])
        self.uav_poses_des[idx] = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]
        self.tgt_pose_updated_flags[idx] = True

    def model_status_cb(self, model_states_msg: ModelStates):
        for i in range(self.uav_num):
            idx = model_states_msg.name.index(f"iris_{i}")
            self.uav_poses[i] = [
                model_states_msg.pose[idx].position.x,
                model_states_msg.pose[idx].position.y,
                model_states_msg.pose[idx].position.z,
            ]

    def calcu_sim(self):
        """calculate the similarity between current formation and desired formation"""
        self.matrix_deg = np.zeros(self.uav_num)
        self.matrix_deg_des = np.zeros(self.uav_num)
        for i in range(self.uav_num):
            for j in range(self.uav_num):
                self.matrix_adj[i][j] = (
                    np.sqrt(
                        (self.uav_poses[i][0] - self.uav_poses[j][0]) ** 2
                        + (self.uav_poses[i][1] - self.uav_poses[j][1]) ** 2
                        + (self.uav_poses[i][2] - self.uav_poses[j][2]) ** 2
                    )
                    + 1e-10
                )
                self.matrix_adj_des[i][j] = np.sqrt(
                    (self.uav_poses_des[i][0] - self.uav_poses_des[j][0]) ** 2
                    + (self.uav_poses_des[i][1] - self.uav_poses_des[j][1]) ** 2
                    + (self.uav_poses_des[i][2] - self.uav_poses_des[j][2]) ** 2
                )
                self.matrix_deg[i] += self.matrix_adj[i][j]
                self.matrix_deg_des[i] += self.matrix_adj_des[i][j]

        # Symmetric Normalized Laplace Matrix
        self.matrix_snl = np.eye(self.uav_num) - np.diag(self.matrix_deg ** (-1 / 2)) @ self.matrix_adj @ np.diag(
            self.matrix_deg ** (-1 / 2)
        )
        self.matrix_snl_des = np.eye(self.uav_num) - np.diag(
            self.matrix_deg_des ** (-1 / 2)
        ) @ self.matrix_adj_des @ np.diag(self.matrix_deg_des ** (-1 / 2))
        temp = self.matrix_snl - self.matrix_snl_des
        self.form_sim = np.trace(temp.T @ temp)

        return self.form_sim

    def run(self):
        rate = rospy.Rate(self.loop_freq)
        while not self.enable_simu_flag:
            # print(f"[{rospy.get_time():<20}] waiting for simulation to start...", end="\r")
            rate.sleep()
        rospy.loginfo("Global scheduler started.")
        uav_controller = "nmpc"
        while not rospy.is_shutdown() and self.enable_simu_flag:
            if np.count_nonzero(self.tgt_pose_updated_flags) == self.uav_num:  # ensure all uav's tgt pose are updated
                self.tgt_pose_updated_flags = [False for _ in range(self.uav_num)]  # reset
                # self.uav_poses = (
                #     self.uav_poses_des_last.copy()
                # )  # FIXME: for now, i think use differnece between current and last desired formation is more reasonable
                polygon_msg = PolygonStamped()
                polygon_msg.header.stamp = rospy.Time.now()
                polygon_msg.header.frame_id = "world"
                polygon_msg.polygon.points = [Point32(x=x, y=y, z=z) for x, y, z in self.uav_poses_des]
                self.form_polygon_des_pub.publish(polygon_msg)

                # clculate formation similarity
                sim_value = self.calcu_sim()
                self.form_sim_ar.append(sim_value)
                if len(self.form_sim_ar) > self.form_sim_int_len * self.tgt_pose_pub_rate:
                    self.form_sim_ar.pop(0)
                else:
                    rospy.loginfo(
                        f"collecting formation similarity data ({len(self.form_sim_ar)/self.form_sim_int_len/self.tgt_pose_pub_rate*100:.2f}%)..."
                    )
                    continue

                # controller decision
                if np.sum(self.form_sim_ar) < self.form_sim_int_th:  # stationary formation
                    rospy.loginfo(f"Stationary formation detected. similarity: {sim_value:.2f}")
                    if uav_controller != "px4":
                        for i in range(self.uav_num):
                            resp = self.uav_controller_srvs[i](controller="px4")
                            if not resp.success:
                                rospy.logerr(f"Failed to call service /iris_{i}/set_controller")
                    uav_controller = "px4"
                else:
                    rospy.loginfo(f"Moving formation detected. similarity: {sim_value:.2f}")
                    if uav_controller != "nmpc":
                        for i in range(self.uav_num):
                            resp = self.uav_controller_srvs[i](controller="nmpc")
                            if not resp.success:
                                rospy.logerr(f"Failed to call service /iris_{i}/set_controller")
                    uav_controller = "nmpc"

                self.form_sim_pub.publish(Float32(data=self.form_sim))
                self.form_sim_int_pub.publish(Float32(data=np.sum(self.form_sim_ar)))
                self.uav_poses_des_last = self.uav_poses_des.copy()
            polygon_msg = PolygonStamped()
            polygon_msg.header.stamp = rospy.Time.now()
            polygon_msg.header.frame_id = "world"
            polygon_msg.polygon.points = [Point32(x=x, y=y, z=z) for x, y, z in self.uav_poses]
            self.form_polygon_pub.publish(polygon_msg)

            rate.sleep()
        rospy.loginfo("Global scheduler stopped.")


class LocalScheduler:
    """Scheduler for optimizing a single UAV's MPC matrix"""

    def __init__(self, uav_num=1, t_size=5, t_size_polyfit=1, t_size_adf=4, loop_freq=10) -> None:
        self.uav_num = uav_num
        self.loop_freq = loop_freq
        self.enable_simu_flag = False
        self.crowds_cap = []  # network capacity requirements of all crowds. [time_idx, crowd_idx]
        self.crowds_cap_filtered = []
        self.crowds_data_prepared = False
        self.cap_pub_freq = 10  # (hz) publish frequency of crowd_simu_node.py

        param_file = os.path.join(sys.path[0], "../config/scheduler_params.yaml")
        param_dict = rosparam.load_file(param_file)[0][0]["local_scheduler"]
        self.t_size = t_size  # (s). time window size
        self.t_size_polyfit = t_size_polyfit  # (s). time window size for polyfit
        self.t_size_adf = t_size_adf  # (s). time window size for ADF
        self.p_value_th = param_dict["p_value_th"]  # 0.2 p-value threshold for ADF
        self.low_pass_filter = [
            LowPassFilter(alpha=param_dict["low_pass_filter_alpha"]) for _ in range(uav_num)
        ]  # low pass filter for crowds capacity
        self.rising_edge_flag = [False for _ in range(uav_num)]
        self.falling_edge_flag = [False for _ in range(uav_num)]
        # self.edge_flag_th = 4.5
        self.max_gradient = param_dict["max_gradient"]
        self.max_gradient_in_win = [0 for _ in range(uav_num)]
        self.mpc_matrices_raw = [None for _ in range(uav_num)]  # raw MPC matrices of UAV
        self.mpc_matrices_adjusted = [None for _ in range(uav_num)]
        self.max_atti_ang_raw = [None for _ in range(uav_num)]  # degree
        self.max_lin_acc_raw = [None for _ in range(uav_num)]
        self.max_atti_ang_adjusted = [None for _ in range(uav_num)]
        self.matrix_q_max_delta = np.array(param_dict["matrix_q_max_delta"])
        self.matrix_r_max_delta = np.array(param_dict["matrix_r_max_delta"])
        self.max_atti_ang_delta = param_dict["max_atti_ang_delta"]  # degree
        self.max_lin_acc_delta = param_dict["max_lin_acc_delta"]  # m/s^2
        self.max_lin_acc_adjusted = [None for _ in range(uav_num)]
        self.cap_grad = [0 for _ in range(uav_num)]

        # proxies
        self.get_mpc_matrix_proxy = []
        self.set_mpc_matrix_proxy = []
        for i in range(uav_num):
            rospy.wait_for_service(f"/iris_{i}/get_mpc_matrix", rospy.Duration(5))
            rospy.wait_for_service(f"/iris_{i}/set_mpc_matrix", rospy.Duration(5))
            self.get_mpc_matrix_proxy.append(rospy.ServiceProxy(f"/iris_{i}/get_mpc_matrix", GetMPCMatrix))
            self.set_mpc_matrix_proxy.append(rospy.ServiceProxy(f"/iris_{i}/set_mpc_matrix", SetMPCMatrix))
            try:
                response = self.get_mpc_matrix_proxy[i].call()
            except:
                rospy.logerr(f"Failed to call service /iris_{i}/get_mpc_matrix")
                return
            self.mpc_matrices_raw[i] = [response.Q, response.R]
            self.mpc_matrices_adjusted[i] = [response.Q, response.R]
            self.max_atti_ang_adjusted[i] = 0

            uav_param_file = os.path.join(sys.path[0], f"../config/uav_params.yaml")
            param_dict = rosparam.load_file(uav_param_file)[0][0]["nmpcc"]["power_saving"]
            self.max_atti_ang_raw[i] = param_dict["max_atti_ang"]
            self.max_atti_ang_adjusted[i] = param_dict["max_atti_ang"]
            self.max_lin_acc_raw[i] = param_dict["max_lin_acc"]
            self.max_lin_acc_adjusted[i] = param_dict["max_lin_acc"]

        print(f"{'uav_num':<20}: {self.uav_num:<20}")
        print(f"{'t_size':<20}: {self.t_size:<20}")
        print(f"{'t_size_polyfit':<20}: {self.t_size_polyfit:<20}")
        print(f"{'t_size_adf':<20}: {self.t_size_adf:<20}")
        print(f"{'p_value_th':<20}: {self.p_value_th:<20}")
        print(f"{'max_gradient':<20}: {self.max_gradient:<20}")
        print(f"{'matrix_q_raw':<20}: {self.mpc_matrices_raw[0][0]}")
        print(f"{'matrix_q_max_delta':<20}: {self.matrix_q_max_delta}")
        print(f"{'matrix_r_raw':<20}: {self.mpc_matrices_raw[0][1]}")
        print(f"{'matrix_r_max_delta':<20}: {self.matrix_r_max_delta}")
        print(f"{'max_atti_ang_raw':<20}: {self.max_atti_ang_raw[0]}")
        print(f"{'max_atti_ang_delta':<20}: {self.max_atti_ang_delta}")
        print("-" * 50)

        # subs
        self.crowds_cap_sub = rospy.Subscriber("/crowds/target_net_capacity", Float32MultiArray, self.crowds_cap_cb)
        self.enable_simu_sub = rospy.Subscriber("/enable_simu", Bool, self.enable_simu_cb)

        # pubs
        self.matrix_q_pubs = []
        self.matrix_r_pubs = []
        self.max_atti_ang_pubs = []
        self.max_lin_acc_pubs = []
        for i in range(uav_num):
            self.matrix_q_pubs.append(rospy.Publisher(f"/iris_{i}/matrix_q", Float32MultiArray, queue_size=1))
            self.matrix_r_pubs.append(rospy.Publisher(f"/iris_{i}/matrix_r", Float32MultiArray, queue_size=1))
            self.max_atti_ang_pubs.append(rospy.Publisher(f"/iris_{i}/max_atti_ang", Float32, queue_size=1))
            self.max_lin_acc_pubs.append(rospy.Publisher(f"/iris_{i}/max_lin_acc", Float32, queue_size=1))
        self.test_pub = rospy.Publisher("/local_scheduler/test", Float32MultiArray, queue_size=1)
        self.adf_pub = rospy.Publisher("/local_scheduler/adf", Float32MultiArray, queue_size=1)
        # self.cap_grad_pub = rospy.Publisher("/local_scheduler/cap_grad", Float32MultiArray, queue_size=1)
        self.scale_factor_pub = rospy.Publisher("/local_scheduler/scale_factor", Float32MultiArray, queue_size=1)
        self.cap_tgt_grad_pub = rospy.Publisher("/local_scheduler/cap_tgt_grad", Float32MultiArray, queue_size=1)
        self.cap_tgt_filtered_pub = rospy.Publisher(
            "/local_scheduler/cap_tgt_filtered", Float32MultiArray, queue_size=1
        )
        self.grad_pub = rospy.Publisher("/local_scheduler/grad", Float32MultiArray, queue_size=1)

        rospy.loginfo("Local scheduler initialized.")

    def enable_simu_cb(self, enable_msg: Bool):
        self.enable_simu_flag = enable_msg.data

    def crowds_cap_cb(self, crowds_msg: Float32MultiArray):
        self.crowds_cap.append(crowds_msg.data)
        self.crowds_cap_filtered.append([self.low_pass_filter[i].step(crowds_msg.data[i]) for i in range(self.uav_num)])
        if len(self.crowds_cap) < (self.t_size * self.cap_pub_freq) + 1:
            rospy.loginfo_throttle(
                1,
                f"collecting crowds net capacity data ({len(self.crowds_cap)/(self.t_size*self.cap_pub_freq)*100:.2f}%)...",
            )
            self.crowds_data_prepared = False
        else:
            self.crowds_data_prepared = True

    def run(self):
        rate = rospy.Rate(self.loop_freq)
        while not self.enable_simu_flag:
            rate.sleep()
            print(f"[{rospy.get_time():<20}] waiting for simulation to start...", end="\r")
            continue
        print(f"LocalScheduler started.")
        while not self.crowds_data_prepared:
            rate.sleep()
            continue
        last_scale_factor = np.zeros(self.uav_num)
        while not rospy.is_shutdown():
            if not self.enable_simu_flag:
                break
            test_msg = Float32MultiArray()
            adf_msg = Float32MultiArray()
            scale_factor_msg = Float32MultiArray()
            grad_msg = Float32MultiArray()
            for i in range(self.uav_num):
                data4adf = np.array(self.crowds_cap_filtered)[-self.t_size_adf * self.cap_pub_freq :, i]
                test_msg.data.append(data4adf[-1])
                adf_result = adfuller(data4adf)
                adf_msg.data.append(adf_result[1])
                if adf_result[1] > self.p_value_th:  # non-stationary
                    idx_offset = self.cap_pub_freq * self.t_size_polyfit
                    gradient = np.polyfit(np.arange(0, idx_offset), np.array(self.crowds_cap)[-idx_offset:, i], 1)[0]
                    grad_msg.data.append(gradient)
                    self.cap_grad[i] = gradient
                    if gradient > 0:  # rising edge
                        if gradient > self.max_gradient_in_win[i]:
                            self.max_gradient_in_win[i] = gradient
                        scale_factor = min(self.max_gradient_in_win[i] / self.max_gradient, 1)
                        r_delta = self.matrix_r_max_delta * scale_factor
                        q_delta = self.matrix_q_max_delta * scale_factor
                        atti_ang_delta = self.max_atti_ang_delta * scale_factor
                        lin_acc_delta = self.max_lin_acc_delta * scale_factor
                        self.mpc_matrices_adjusted[i][0] = self.mpc_matrices_raw[i][0] + q_delta
                        self.mpc_matrices_adjusted[i][1] = self.mpc_matrices_raw[i][1] + r_delta
                        self.max_atti_ang_adjusted[i] = self.max_atti_ang_raw[i] + atti_ang_delta
                        self.max_lin_acc_adjusted[i] = self.max_lin_acc_raw[i] + lin_acc_delta

                        scale_factor_msg.data.append(scale_factor)
                        last_scale_factor[i] = scale_factor
                    else:  # falling edge. don't change the MPC matrices
                        scale_factor_msg.data.append(last_scale_factor[i])
                        pass

                else:
                    self.mpc_matrices_adjusted[i][0] = self.mpc_matrices_raw[i][0]
                    self.mpc_matrices_adjusted[i][1] = self.mpc_matrices_raw[i][1]
                    self.max_atti_ang_adjusted[i] = self.max_atti_ang_raw[i]
                    self.max_lin_acc_adjusted[i] = self.max_lin_acc_raw[i]
                    self.max_gradient_in_win[i] = 0
                    scale_factor_msg.data.append(0)
                    last_scale_factor[i] = 0
                    grad_msg.data.append(0)

                # if not self.set_mpc_matrix_proxy[i](self.mpc_matrices_adjusted[i][0], self.mpc_matrices_adjusted[i][1]):
                #     rospy.logerr(f"Failed to call service /iris_{i}/set_mpc_matrix")
                # considering calling delay, use topics instead
                self.matrix_q_pubs[i].publish(Float32MultiArray(data=self.mpc_matrices_adjusted[i][0]))
                self.matrix_r_pubs[i].publish(Float32MultiArray(data=self.mpc_matrices_adjusted[i][1]))
                self.max_atti_ang_pubs[i].publish(Float32(data=np.deg2rad(self.max_atti_ang_adjusted[i])))
                self.max_lin_acc_pubs[i].publish(Float32(data=self.max_lin_acc_adjusted[i]))

                # cap_diff = self.crowds_cap[-1][i] - self.crowds_cap[-2][i]
                # if cap_diff > self.edge_flag_th:
                #     self.rising_edge_flag[i] = True
                #     # rospy.loginfo("rising edge detected.")
                # elif cap_diff > -self.edge_flag_th:
                #     self.rising_edge_flag[i] = False
                # if cap_diff < -self.edge_flag_th:
                #     self.falling_edge_flag[i] = True
                #     # rospy.loginfo("falling edge detected.")
                # elif cap_diff < self.edge_flag_th:
                #     self.falling_edge_flag[i] = False
            self.cap_tgt_grad_pub.publish(Float32MultiArray(data=self.cap_grad))
            self.test_pub.publish(test_msg)
            self.adf_pub.publish(adf_msg)
            self.grad_pub.publish(grad_msg)
            self.scale_factor_pub.publish(scale_factor_msg)
            self.cap_tgt_filtered_pub.publish(Float32MultiArray(data=self.crowds_cap_filtered[-1]))
            rate.sleep()
        rospy.loginfo("Local scheduler stopped.")


if __name__ == "__main__":
    rospy.init_node("scheduler", anonymous=False)
    uav_num = rospy.get_param("~uav_num", 3)
    enable_global_scheduler = rospy.get_param("~global", True)
    enable_local_scheduler = rospy.get_param("~local", True)
    print(f"{'uav_num':<20}: {uav_num}")
    print(f"{'enable_global_scheduler':<20}: {enable_global_scheduler}")
    print(f"{'enable_local_scheduler':<20}: {enable_local_scheduler}")

    if enable_local_scheduler:
        local_scheduler = LocalScheduler(uav_num=uav_num, loop_freq=20)
        local_thread = threading.Thread(target=local_scheduler.run)
        local_thread.start()
    if enable_global_scheduler:
        global_scheduler = GlobalScheduler(uav_num=uav_num, loop_freq=20)
        global_thread = threading.Thread(target=global_scheduler.run)
        global_thread.start()

    rospy.spin()
