#! /usr/bin/env python3
"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-05-14 15:46:24
 # @ Modified time: 2022-05-14 16:01:51
 # @ Description: 
 """


import csv
import datetime
import json
import os
import sys
import time

import ipdb
import numpy as np
import psutil
import rosparam
import rospy
import sympy
import tf.transformations as tft
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from matplotlib import pyplot as plt
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Float32MultiArray

from utils import *


class UAVStateMonitor:
    """get the realtime state of the uav"""

    def __init__(self, uav_name) -> None:
        self.uav_name = uav_name
        self.real_pose_sub = rospy.Subscriber(
            f"/{self.uav_name}/real_pose", PoseStamped, self.real_pose_cb, queue_size=1
        )
        self.real_twist_sub = rospy.Subscriber(
            f"/{self.uav_name}/real_twist", TwistStamped, self.real_twist_cb, queue_size=1
        )
        self.curr_pose = PoseStamped()
        self.curr_twist = TwistStamped()

    def real_pose_cb(self, real_pose_msg: PoseStamped):
        self.curr_pose = real_pose_msg

    def real_twist_cb(self, real_twist_msg: TwistStamped):
        self.curr_twist = real_twist_msg


class PowerAnalyzer:
    """PowerAnalyzer for one uav.

    Refer to:
    Z. Liu, R. Sengupta and A. Kurzhanskiy, "A power consumption model for multi-rotor small
    unmanned aircraft systems," 2017 International Conference on Unmanned Aircraft Systems (ICUAS),
    Miami, FL, USA, 2017, pp. 310-315, doi: 10.1109/ICUAS.2017.7991310.
    """

    # Parameters for iris quadrotor
    N = 2  # totol number of blades in a single propeller
    A_quad = 0.03891  # cross-sectional area of the uav when against wind (cm^2)
    c = 1  # blade chord width
    c_d = 0.5  # air drag coefficient of the blade. a normal car's drag coefficient is 0.28-0.4
    k_1 = 0.8554
    k_2 = 0.3051  # kg/m^(1/2)
    k_3 = 8.54858e-06  # motor constant (N/(rad/s)^2)
    c_2 = 0.3177  # m/kg^(1/2)
    c_3 = 0  # negligible, approx 0
    c_4 = 0.0296  # kg/m
    c_5 = 0.0279  # Ns/m
    c_6 = 0  # negligible, approx 0
    g = 9.8
    pho = 1.225  # kg/m^3 air density
    """
    Size: 10 cm in height, 55 cm motor-to-motor, 0.3891m in side length
    Propellers: (2) 10 x 4.7 normal-rotation, (2) 10 x 4.7 reverse-rotation
    Motors: AC 2830, 850 kV
    Battery: 3-cell 11.1 V 3.5 Ah lithium polymer with XT-60 type connector. Weight: 262 grams
    Weight (with battery): 1282 grams
    Motor to motor dimension: 550mm
    """

    def __init__(self, uav_name="iris_0", state_monitor=None) -> None:
        self.uav_name = uav_name
        self.thrust_est = 0  # estimated thrust
        self.instant_power = 0
        self.energy_total = 0
        self.induced_power = 0
        self.profile_power = 0
        self.parasite_power = 0
        self.total_induced_power = 0
        self.total_profile_power = 0
        self.total_parasite_power = 0
        self.total_instant_power = 0
        self.sampling_rate = 10  # (hz)

        # subs
        self.thrust_est_sub = rospy.Subscriber(f"/{self.uav_name}/thrust_est", Float32, self.thrust_est_cb)
        if state_monitor is None:
            self.state_monitor = UAVStateMonitor(uav_name=self.uav_name)
        else:
            self.state_monitor = state_monitor
        # self.model_state_sub = rospy.Subscriber(f"/gazebo/model_states", ModelStates, self.model_state_cb)
        # self.imu_sub = rospy.Subscriber(f"/{self.uav_name}/imu/data", Imu, self.imu_cb)

        # pubs
        self.induced_power_pub = rospy.Publisher(f"/{self.uav_name}/perf_analyzer/induced_power", Float32, queue_size=1)
        self.profile_power_pub = rospy.Publisher(f"/{self.uav_name}/perf_analyzer/profile_power", Float32, queue_size=1)
        self.parasite_power_pub = rospy.Publisher(
            f"/{self.uav_name}/perf_analyzer/parasite_power", Float32, queue_size=1
        )
        self.instant_power_pub = rospy.Publisher(f"/{self.uav_name}/perf_analyzer/instant_power", Float32, queue_size=1)

    def calcu_induced_power(self, T, V_vert):
        """
        Calculate the induced power.

        Parameters:
        - T: Total thrust.
        - V_vert: Vertical speed at each boundary.

        Returns:
        Induced power based on given thrust and vertical speed.
        """
        return PowerAnalyzer.k_1 * T * (V_vert / 2 + np.sqrt((V_vert / 2) ** 2 + T / PowerAnalyzer.k_2**2))

    def calcu_profile_power(self, T, V_air, alpha):
        """Parameters:
        - T: Total thrust.
        - V_air: Horizontal airspeed.
        - alpha: angle of attack for each propeller disk. alpha = pi/2 - pitch

        """
        return PowerAnalyzer.c_2 * T ** (3 / 2) + PowerAnalyzer.c_3 * (V_air * np.cos(alpha)) ** 2 * T ** (1 / 2)

    def calcu_parasite_power(self, C_d, rho, A_quad, V_air):
        """
        Parameters:
        - C_d: Drag coefficient of the vehicle body.
        - rho: Density of air.
        - A_quad: Cross-sectional area of the vehicle when against wind.
        - V_air: Horizontal airspeed.
        """
        return 0.5 * C_d * rho * A_quad * V_air**3

    def calculate_T(m, V_air, alpha):
        """
        the theoretical thrust given the airspeed and angle of attack.

        - alpha: Angle of attack of the propeller disk
        """
        T = sympy.symbols("T")
        f = (
            np.sqrt(
                (m * PowerAnalyzer.g - (PowerAnalyzer.c_5 * (V_air * np.cos(alpha)) ** 2 + PowerAnalyzer.c_6 * T)) ** 2
                + (PowerAnalyzer.c_4 * V_air**2) ** 2
            )
            - T
        )
        result = sympy.solve(f, T)
        return result

    def test_power_model(self):
        m = 1.5
        ax1 = plt.subplot(311)
        ax2 = plt.subplot(312)
        ax3 = plt.subplot(313)

        # induced power
        T = np.arange(0, 10, 0.1)
        # V_vert = np.arange(0, 10, 0.1)
        self.induced_power = self.calcu_induced_power(T, 0)
        ax1.plot(T, self.induced_power)
        ax1.set_xlabel("Thrust (N)")
        ax1.set_ylabel("Induced power (W)")
        ax1.set_title("Induced power vs thrust")
        ax1.legend()

        # profile power
        T = np.arange(0, 10, 0.1)
        V_air = 10
        alpha = np.radians(10)
        self.profile_power = self.calcu_profile_power(T, V_air, alpha)
        ax2.plot(
            T,
            self.profile_power,
        )
        ax2.set_xlabel("Thrust (N)")
        ax2.set_ylabel("Profile power (W)")
        ax2.set_title("Profile power vs thrust and v_air")
        ax2.legend()

        # # parasite power
        V_air = np.arange(0, 10, 0.1)
        self.parasite_power = self.calcu_parasite_power(self.c_d, self.pho, self.A_quad, V_air)
        ax3.plot(V_air, self.parasite_power)
        ax3.set_xlabel("V_air (m/s)")
        ax3.set_ylabel("Parasite power (W)")
        ax3.set_title("Parasite power vs V_air")
        ax3.legend()

        plt.legend()
        plt.tight_layout()
        plt.show()

    def thrust_est_cb(self, thrust_est_msg: Float32):
        self.thrust_est = thrust_est_msg.data

    def model_state_cb(self, model_state_msg: ModelStates):
        model_idx = model_state_msg.name.index(f"{self.uav_name}")
        self.twist = model_state_msg.twist[model_idx]
        self.pose = model_state_msg.pose[model_idx]

    def run(self):
        loop_rate = rospy.Rate(self.sampling_rate)
        rospy.loginfo(f"Power analyzer for uav {self.uav_id} started.")
        while not rospy.is_shutdown():
            self.pose = self.state_monitor.curr_pose.pose
            self.twist = self.state_monitor.curr_twist.twist
            [r, p, y] = tft.euler_from_quaternion(
                [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
            )
            [r, p, y, _, _, _] = ENU2NED_Position([r, p, y, 0, 0, 0])
            speed_in_body = inertial2body([self.twist.linear.x, self.twist.linear.y, self.twist.linear.z], self.pose)

            self.induced_power = self.calcu_induced_power(self, self.thrust_est, abs(speed_in_body[2]))
            self.profile_power = self.calcu_profile_power(self.thrust_est, abs(speed_in_body[0]), p)
            self.parasite_power = self.calcu_parasite_power(self.c_d, self.pho, self.A_quad, abs(speed_in_body[0]))
            self.instant_power = self.induced_power + self.profile_power + self.parasite_power
            self.energy_total += self.instant_power / self.sampling_rate

            self.induced_power_pub.publish(self.induced_power)
            self.profile_power_pub.publish(self.profile_power)
            self.parasite_power_pub.publish(self.parasite_power)
            self.instant_power_pub.publish(self.instant_power)
            loop_rate.sleep()

    def run_in_ext(self, sampling_duty):
        """run in external loop"""
        self.pose = self.state_monitor.curr_pose.pose
        self.twist = self.state_monitor.curr_twist.twist
        [r, p, y] = tft.euler_from_quaternion(
            [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        )
        [r, p, y, _, _, _] = ENU2NED_Position([r, p, y, 0, 0, 0])
        speed_in_body = inertial2body([self.twist.linear.x, self.twist.linear.y, self.twist.linear.z], self.pose)

        if isinstance(self.thrust_est, complex) or np.isnan(self.thrust_est):
            rospy.logerr(f"thrust_est: {self.thrust_est}")
            return
        if isinstance(speed_in_body[2], complex) or np.isnan(speed_in_body[2]):
            rospy.logerr(f"speed_in_body[2]: {speed_in_body[2]}")
            return
        self.induced_power = self.calcu_induced_power(self.thrust_est, abs(speed_in_body[2]))
        if isinstance(self.induced_power, complex) or np.isnan(self.induced_power):
            rospy.logerr(
                f"induced_power: {self.induced_power} thrust_est: {self.thrust_est} speed_in_body[2]: {speed_in_body[2]}"
            )
            return
        self.profile_power = self.calcu_profile_power(self.thrust_est, abs(speed_in_body[0]), p)
        self.parasite_power = self.calcu_parasite_power(self.c_d, self.pho, self.A_quad, abs(speed_in_body[0]))
        self.total_induced_power += self.induced_power
        self.total_profile_power += self.profile_power
        self.total_parasite_power += self.parasite_power
        self.instant_power = self.induced_power + self.profile_power + self.parasite_power
        self.total_instant_power += self.instant_power
        self.energy_total += self.instant_power * sampling_duty

        self.induced_power_pub.publish(self.induced_power)
        self.profile_power_pub.publish(self.profile_power)
        self.parasite_power_pub.publish(self.parasite_power)
        self.instant_power_pub.publish(self.instant_power)


class NetworkAnalyzer:
    def __init__(self, uav_name="iris_0", state_monitor=None) -> None:
        self.uav_name = uav_name
        self.tgt_pose = PoseStamped()
        self.net_cap_th = 3  # (mbps) network capacity reach threshold
        self.net_cap_curr = 0  # (mbps) current network capacity
        self.net_cap_tgt = 0
        self.total_traffic = 0  # mb
        self.cap_resp_time = None  # s reponse time from the timestamp not meeting the capacity to meeting
        self.cap_resp_time_total = 0  # s
        self.cap_resp_time_total_weighted = 0  # s
        self.cap_resp_time_ave = 0  # s average time to meet the capacity
        self.timestamp_not_meet_cap = None  # s timestamp when not meeting the capacity requirement
        self.cap_meet_flag = False
        self.start_time = None
        self.cap_grad_curr = 0
        self.cap_grad_in_win = 0  # traffic gradient in a time window

        # params for wifi channel
        self.band_width = 2.4e9  # 2.4Ghz
        self.transmit_power = dbm2watt(20)  # dbm
        self.noise_power = dbm2watt(25)

        if state_monitor is None:
            self.state_monitor = UAVStateMonitor(uav_name=self.uav_name)
        else:
            self.state_monitor = state_monitor

        # subs
        self.tgt_pose_sub = rospy.Subscriber(
            f"/{self.uav_name}/target_pose", PoseStamped, self.tgt_pose_cb, queue_size=1
        )
        self.cap_tgt_sub = rospy.Subscriber(f"/crowds/target_net_capacity", Float32MultiArray, self.cap_tgt_cb)
        self.cap_tgt_grad_sub = rospy.Subscriber(f"/scheduler/cap_tgt_grad", Float32MultiArray, self.cap_tgt_grad_cb)
        # pubs
        self.curr_cap_pub = rospy.Publisher(f"/{self.uav_name}/perf_analyzer/curr_net_cap", Float32, queue_size=1)
        self.test_data_pub = rospy.Publisher(f"/{self.uav_name}/perf_analyzer/test", Float32MultiArray, queue_size=1)

    def cap_tgt_grad_cb(self, msg: Float32MultiArray):
        idx = int(self.uav_name.split("_")[-1])
        self.cap_grad_curr = msg.data[idx]

    def tgt_pose_cb(self, tgt_pose_msg: PoseStamped):
        self.tgt_pose = tgt_pose_msg

    def cap_tgt_cb(self, tgt_cap_msg: Float32):
        idx = int(self.uav_name.split("_")[-1])
        self.net_cap_tgt = tgt_cap_msg.data[idx]

    def run_in_ext(self, samping_duty):
        dist = np.linalg.norm(
            [
                self.tgt_pose.pose.position.x - self.state_monitor.curr_pose.pose.position.x,
                self.tgt_pose.pose.position.y - self.state_monitor.curr_pose.pose.position.y,
                0 - self.state_monitor.curr_pose.pose.position.z,
            ]
        )
        path_loss = two_ray_ground_reflection(dist, 1, self.state_monitor.curr_pose.pose.position.z, 1, 1)
        gain = 1 / path_loss
        self.net_cap_curr = calcu_capacity(self.band_width, self.transmit_power, gain, self.noise_power) / 1e6
        if self.net_cap_curr - self.net_cap_tgt >= -self.net_cap_th:  # meet the capacity requirement
            if (
                not self.cap_meet_flag and self.timestamp_not_meet_cap != None
            ):  # first time meet the capacity requirement
                self.cap_resp_time = rospy.get_time() - self.timestamp_not_meet_cap
                self.cap_resp_time_total += self.cap_resp_time
                # if self.uav_name == "iris_0":
                #     print(f"cap_resp_time: {self.cap_resp_time}")
                #     print(f"cap_resp_time_total: {self.cap_resp_time_total}")

                cap_resp_time_weighted = self.cap_resp_time * self.cap_grad_in_win
                self.cap_resp_time_total_weighted += cap_resp_time_weighted  # weighted by traffic gradient
                # if self.uav_name == "iris_0":
                #     print(f"traffic_grad_in_win: {self.cap_grad_in_win}")
                #     print(f"cap_resp_time_weighted: {cap_resp_time_weighted}")
                #     print(f"cap_resp_time_total_weighted: {self.cap_resp_time_total_weighted}")
                #     print("-" * 50)
                self.cap_grad_in_win = 0
            self.cap_meet_flag = True
        else:
            if self.timestamp_not_meet_cap is None:  # initial timestamp
                self.timestamp_not_meet_cap = rospy.get_time()
            if self.cap_meet_flag:  # change-point: last loop meet while this loop not meet
                self.timestamp_not_meet_cap = rospy.get_time()
            self.cap_meet_flag = False
            if self.cap_grad_curr > 0:
                self.cap_grad_in_win += self.cap_grad_curr
        self.total_traffic += self.net_cap_curr * samping_duty

        self.curr_cap_pub.publish(self.net_cap_curr)
        self.test_data_pub.publish(Float32MultiArray(data=[self.cap_grad_in_win]))


class PerformanceAnalyzer:
    def __init__(self, uav_num=3, samping_freq=10, controller="nmpc", mode="both", result_file_folder=None) -> None:
        self.uav_num = uav_num
        self.samping_freq = samping_freq  # (hz)
        self.uav_state_monitors = []  # shared monitors for memory efficiency
        self.power_analyzers = []
        self.network_analyzers = []
        self.enable_simu_flag = False
        self.controller = controller
        self.csv_writers = []
        self.time_suffix = datetime.datetime.now().strftime("%m-%d-%H%M%S")
        self.ave_curr_net_cap = 0  # average current network capacity of all uavs
        self.ave_tgt_net_cap = 0
        self.ave_instant_power = 0
        self.perf_data_write_freq = 2  # hz
        self.cpu_usage = 0
        self.cpu_usage_sum = 0
        self.cpu_usage_ave = 0
        if result_file_folder is None:
            self.result_file_folder = os.path.join(sys.path[0], f"../data/perf_anaysis")
        else:
            self.result_file_folder = result_file_folder
        if mode == "both":
            self.mode = 3
        elif mode == "power":
            self.mode = 1
            self.enable_simu_flag = True
        elif mode == "net":
            self.mode = 2
        else:
            raise ValueError("mode must be both, power or net")
        print(f"{'uav_num':<20}: {self.uav_num}")
        print(f"{'controller':<20}: {self.controller}")
        print(f"{'mode':<20}: {self.mode}")
        print(f"{'sampling_freq':<20}: {self.samping_freq}")
        print(f"{'result_file_folder':<20}: {self.result_file_folder}")

        if not os.path.exists(self.result_file_folder):
            os.makedirs(self.result_file_folder)
        for i in range(uav_num):
            self.uav_state_monitors.append(UAVStateMonitor(uav_name=f"iris_{i}"))
            if self.mode != 2:  # power or both
                self.power_analyzers.append(
                    PowerAnalyzer(uav_name=f"iris_{i}", state_monitor=self.uav_state_monitors[i])
                )
            if self.mode >= 2:  # net or both
                self.network_analyzers.append(
                    NetworkAnalyzer(uav_name=f"iris_{i}", state_monitor=self.uav_state_monitors[i])
                )
            self.csv_writers.append(
                csv.writer(
                    open(
                        os.path.join(
                            sys.path[0],
                            self.result_file_folder,
                            f"perf_data_{controller}_u{uav_num}_iris_{i}_{self.time_suffix}.csv",
                        ),
                        "w",
                    )
                )
            )
            self.csv_writers[i].writerow(  # header
                [
                    "time",
                    "induced_power",
                    "profile_power",
                    "parasite_power",
                    "instant_power",
                    "energy_total",
                    "net_cap_curr",
                    "net_cap_tgt",
                    "total_traffic",
                    "cap_resp_time",
                    "cap_resp_time_total_weighted",
                ]
            )

        self.enable_simu_sub = rospy.Subscriber("/enable_simu", Bool, self.start_simu_cb, queue_size=1)

        # pubs
        self.ave_curr_net_cap_pub = rospy.Publisher(f"/perf_analyzer/ave_curr_net_cap", Float32, queue_size=1)
        self.ave_tgt_net_cap_pub = rospy.Publisher(f"/perf_analyzer/ave_tgt_net_cap", Float32, queue_size=1)
        self.ave_instant_power_pub = rospy.Publisher(f"/perf_analyzer/ave_instant_power", Float32, queue_size=1)
        self.cpu_usage_pub = rospy.Publisher(f"/perf_analyzer/cpu_usage", Float32, queue_size=1)
        self.test_pub = rospy.Publisher(f"/perf_analyzer/test", Float32MultiArray, queue_size=1)

    def start_simu_cb(self, start_simu_msg: Bool):
        self.enable_simu_flag = start_simu_msg.data

    def run(self):
        loop_rate = rospy.Rate(self.samping_freq)
        while not self.enable_simu_flag and not rospy.is_shutdown():
            loop_rate.sleep()
            print(f"[{rospy.get_time():<20}] waiting for simulation to start...", end="\r")
            continue
        rospy.loginfo("Performance analyzer started.")
        start_time = rospy.get_time()
        cnt = 0
        while not rospy.is_shutdown():
            if not self.enable_simu_flag:
                break
            for i in range(self.uav_num):
                if self.mode != 2:
                    self.power_analyzers[i].run_in_ext(1 / self.samping_freq)
                if self.mode >= 2:
                    self.network_analyzers[i].run_in_ext(1 / self.samping_freq)
                if cnt % (int(self.samping_freq / self.perf_data_write_freq)) == 0:
                    if self.mode == 1:
                        self.curr_perf_data = [
                            rospy.get_time(),
                            self.power_analyzers[i].induced_power,
                            self.power_analyzers[i].profile_power,
                            self.power_analyzers[i].parasite_power,
                            self.power_analyzers[i].instant_power,
                            self.power_analyzers[i].energy_total,
                            0,
                            0,
                            0,
                            0,
                            0,
                        ]
                    elif self.mode == 2:
                        self.curr_perf_data = [
                            0,
                            0,
                            0,
                            0,
                            0,
                            self.network_analyzers[i].net_cap_curr,
                            self.network_analyzers[i].net_cap_tgt,
                            self.network_analyzers[i].total_traffic,
                            self.network_analyzers[i].cap_resp_time,
                            self.network_analyzers[i].cap_resp_time_total_weighted,
                        ]
                        self.curr_perf_data.insert(0, rospy.get_time())
                    elif self.mode == 3:
                        self.curr_perf_data = [
                            rospy.get_time(),
                            self.power_analyzers[i].induced_power,
                            self.power_analyzers[i].profile_power,
                            self.power_analyzers[i].parasite_power,
                            self.power_analyzers[i].instant_power,
                            self.power_analyzers[i].energy_total,
                            self.network_analyzers[i].net_cap_curr,
                            self.network_analyzers[i].net_cap_tgt,
                            self.network_analyzers[i].total_traffic,
                            self.network_analyzers[i].cap_resp_time,
                            self.network_analyzers[i].cap_resp_time_total_weighted,
                        ]
                for data in self.curr_perf_data:
                    if data != None:
                        data = round(data, 4)
                self.csv_writers[i].writerow(self.curr_perf_data)
                self.ave_curr_net_cap += self.network_analyzers[i].net_cap_curr
                self.ave_tgt_net_cap += self.network_analyzers[i].net_cap_tgt
                self.ave_instant_power += self.power_analyzers[i].instant_power
            self.ave_curr_net_cap /= self.uav_num
            self.ave_tgt_net_cap /= self.uav_num
            self.ave_instant_power /= self.uav_num
            self.ave_curr_net_cap_pub.publish(self.ave_curr_net_cap)
            self.ave_tgt_net_cap_pub.publish(self.ave_tgt_net_cap)
            self.ave_instant_power_pub.publish(self.ave_instant_power)
            self.ave_curr_net_cap = 0
            self.ave_tgt_net_cap = 0
            self.ave_instant_power = 0
            self.cpu_usage = psutil.cpu_percent()
            self.cpu_usage_sum += self.cpu_usage
            self.cpu_usage_ave = self.cpu_usage_sum / (cnt + 1)
            self.cpu_usage_pub.publish(self.cpu_usage)
            test_msg = Float32MultiArray()
            test_msg.data = [self.cpu_usage_sum, self.cpu_usage_ave]  # FIXME
            self.test_pub.publish(test_msg)
            cnt += 1
            loop_rate.sleep()

        rospy.loginfo("Performance analyzer stopped.")
        end_time = rospy.get_time()
        file_path = os.path.join(
            sys.path[0],
            self.result_file_folder,
            f"perf_anaysis_result_{self.controller}_u{self.uav_num}_" + self.time_suffix + ".json",
        )
        file_obj = open(file_path, "w")
        result = {
            "cpu_usage_ave": self.cpu_usage_ave,
            "cpu_usage_sum": self.cpu_usage_sum,
        }
        for i in range(self.uav_num):
            # network performance
            net_traf_total = self.network_analyzers[i].total_traffic
            cap_resp_time_ave = self.network_analyzers[i].cap_resp_time_total / (end_time - start_time)
            cap_resp_time_total = self.network_analyzers[i].cap_resp_time_total
            cap_resp_time_total_weighted = self.network_analyzers[i].cap_resp_time_total_weighted
            cap_resp_time_ave_weighted = cap_resp_time_total_weighted / (end_time - start_time)
            # power performance
            induced_power_ave = self.power_analyzers[i].total_induced_power / (end_time - start_time)
            parasite_power_ave = self.power_analyzers[i].total_parasite_power / (end_time - start_time)
            profile_power_ave = self.power_analyzers[i].total_profile_power / (end_time - start_time)
            instant_power_ave = self.power_analyzers[i].total_instant_power / (end_time - start_time)
            energy_total = self.power_analyzers[i].energy_total

            result_dict = {
                "simu_time": end_time - start_time,
                "net_traf_total": net_traf_total,
                "cap_resp_time_ave": cap_resp_time_ave,
                "cap_resp_time_total": cap_resp_time_total,
                "cap_resp_time_total_weighted": cap_resp_time_total_weighted,
                "cap_resp_time_ave_weighted": cap_resp_time_ave_weighted,
                "induced_power_ave": induced_power_ave,
                "parasite_power_ave": parasite_power_ave,
                "profile_power_ave": profile_power_ave,
                "instant_power_ave": instant_power_ave,
                "energy_total": energy_total,
            }
            result[f"iris_{i}"] = result_dict
            uav_param_file = os.path.join(sys.path[0], f"../config/uav_params.yaml")
            uav_param_dict = rosparam.load_file(uav_param_file)[0][0]
            result[f"iris_{i}"]["param"] = uav_param_dict["nmpcc"]["power_saving"]

        json.dump(result, file_obj, indent=4)
        rospy.loginfo(f"result file saved to {file_path}")


if __name__ == "__main__":
    rospy.init_node("performace_analysis_node", anonymous=False)
    uav_num = rospy.get_param("~uav_num", 1)
    controller = rospy.get_param("~controller", "nmpc")
    mode = rospy.get_param("~mode", "both")
    result_file_folder = rospy.get_param("~result_file_folder", None)
    if result_file_folder == "default":
        result_file_folder = None

    # test PowerAnalyzer
    # analyzers = [PowerAnalyzer(uav_id=i) for i in range(uav_num)]
    # for i in range(uav_num):
    #     Thread(target=analyzers[i].run).start()

    # test NetworkAnalyzer
    # traffic_anas = [TrafficAnalyzer(uav_name=f"iris_{i}") for i in range(uav_num)]
    # freq = 30
    # loop_rate = rospy.Rate(freq)
    # while not rospy.is_shutdown():
    #     for i in range(uav_num):
    #         traffic_anas[i].run_in_ext(freq)
    #     loop_rate.sleep()
    # rospy.spin()

    perf_ana = PerformanceAnalyzer(
        uav_num=uav_num, controller=controller, mode=mode, result_file_folder=result_file_folder
    )
    perf_ana.run()
