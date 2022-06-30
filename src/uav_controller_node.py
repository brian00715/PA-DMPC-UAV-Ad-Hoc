#!/usr/bin/env python
"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-05-12 18:59:56
 # @ Modified time: 2022-05-12 19:49:54
 # @ Description: ROS wrapped controller node for single UAV.
 """

import os
import sys

import numpy as np
import rosparam
import rospy
import tf.transformations as tft
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, TwistStamped, Vector3
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from dmpc_uav_ad_hoc.cfg import ControllerConfig
from dmpc_uav_ad_hoc.srv import (
    GetControllerParam,
    GetControllerParamResponse,
    GetMPCMatrix,
    GetMPCMatrixResponse,
    SetController,
    SetControllerResponse,
    SetMPCMatrix,
    SetMPCMatrixResponse,
)
from nmpc_controller import NMPCC
from pid_controller import PIDC
from traj_generator import get_nearest_point_idx
from utils import *


class UAVState:
    def __init__(self) -> None:
        self.pose_now = PoseStamped()  # pose in ENU coordinate
        self.pose_now.pose.orientation.w = 1.0
        self.vel_now = TwistStamped()


class UAVControllerNode:
    def __init__(
        self,
        uav_idx=0,
        mass=1.5,
        init_pose=np.zeros(6),
        nmpcc_params: {} = None,
        h_pidc_params: {} = None,
        ctrl_rate=30,
        controller="nmpc",
        follow_mode="waypoint",
        h_pidc_enable=False,
        lin_interp_enable=False,
        pub_opti_traj_flag=True,
        pub_prev_traj_flag=True,
        pub_lin_interp_traj_flag=True,
        pub_local_ref_traj_flag=True,
        prev_traj_msg_len=1000,
    ):
        """

        Args:
            uav_idx (int, optional): index. Defaults to 0.
            mass (float, optional): [kg] mass of uav. Defaults to 1.5.
            init_pose (_type_, optional): initial pose [x,y,z,roll,pitch,yaw]. Defaults to np.zeros(6).
            ctrl_rate (int, optional): main loop rate. Defaults to 30.
            controller (str, optional): nmpc or px4. Defaults to "nmpc".
            follow_mode (str, optional): waypoint or traj. Defaults to "waypoint".
            h_pidc_enable (bool, optional): whether to use pid compensate for height. Defaults to False.
            lin_interp_enable (bool, optional): whether to use linear interpolation to generate a trajectory from current pose to target pose. Defaults to False.
        """
        print("-" * 50)
        rospy.loginfo(f"starting controller for uav_{uav_idx}...")
        self.uav_idx = uav_idx
        self.ns = f"/iris_{uav_idx}"  # namespace
        self.mass = mass
        self.init_pose = init_pose
        self.motor_constant = 8.54858e-06
        self.input_scale = 1100
        self.controller = controller  # px4, nmpc
        self.ctrl_rate = ctrl_rate
        self.loop_rate = rospy.Rate(self.ctrl_rate)

        # visualization related
        self.pub_opti_traj_flag = pub_opti_traj_flag
        self.pub_prev_traj_flag = pub_prev_traj_flag
        self.pub_lin_interp_traj_flag = pub_lin_interp_traj_flag
        self.prev_traj_msg = None
        if self.pub_prev_traj_flag:
            self.prev_traj_msg = Path()
        self.prev_traj_msg_len = prev_traj_msg_len
        self.pub_local_ref_traj_flag = pub_local_ref_traj_flag

        # optinal features
        self.h_pidc_enable = h_pidc_enable
        self.lin_interp_enable = lin_interp_enable
        if self.lin_interp_enable and self.pub_lin_interp_traj_flag:
            self.lin_interp_msg = PoseArray()

        # state variables
        self.state = UAVState()
        self.thrust_est = 0  # estimated thrust from IMU
        self.enable_ctrl = False
        self.tgt_pose = PoseStamped()
        set_pose_msg(self.tgt_pose.pose, init_pose[:3], tft.quaternion_from_euler(*init_pose[3:]))
        self.tgt_traj = Path()
        self.tgt_traj.poses = [self.tgt_pose] * (nmpcc_params["N"] + 1)
        self.tgt_speed = Twist()
        self.mavros_state = State()
        self.follow_mode = follow_mode  # path, waypoint

        print("Controller Node Parameters".ljust(50, "-"))
        print(f"{'namespace':<25}: {self.ns}")
        print(f"{'controller':<25}: {self.controller}")
        print(f"{'ctrl_rate':<25}: {self.ctrl_rate}")
        print(f"{'h_pidc_enable':<25}: {self.h_pidc_enable}")
        print(f"{'lin_interp_enable':<25}: {self.lin_interp_enable}")
        print(f"{'follow_mode':<25}: {self.follow_mode}")

        # tunable params
        self.nmpc_q: np.ndarray = nmpcc_params["Q"]  # diag matrix, not 1d-array
        self.nmpc_r: np.ndarray = nmpcc_params["R"]
        self.nmpc_qf: np.ndarray = nmpcc_params["Qf"]
        self.max_atti_ang = nmpcc_params["max_atti_ang"]
        self.max_atti_ang_rate = nmpcc_params["max_atti_ang_rate"]
        self.max_lin_acc = nmpcc_params["max_lin_acc"]

        self.nmpcc = NMPCC(
            T=1 / self.ctrl_rate * 2,
            N=nmpcc_params["N"],
            Q=nmpcc_params["Q"],
            Qf=nmpcc_params["Qf"],
            R=nmpcc_params["R"],
            prob_params={
                "control_dim": 4,
                "state_dim": 6,
                "init_pose": init_pose,
                "mass": self.mass,
                "max_atti_ang": nmpcc_params["max_atti_ang"],
                "max_atti_ang_rate": nmpcc_params["max_atti_ang_rate"],
                "max_thrust": nmpcc_params["max_thrust"],
                "max_thrust_rate": nmpcc_params["max_thrust_rate"],
                "max_lin_acc": nmpcc_params["max_lin_acc"],
            },
        )
        self.h_pidc = PIDC(
            h_pidc_params["kp"],
            h_pidc_params["ki"],
            h_pidc_params["kd"],
            h_pidc_params["int_clamp"],
            h_pidc_params["int_err_th"],
        )

        # pubs
        self.opti_traj_pub = rospy.Publisher(self.ns + "/opti_traj", MarkerArray, queue_size=10)
        self.px4_atti_cmd_pub = rospy.Publisher(self.ns + "/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)
        self.px4_pos_cmd_pub = rospy.Publisher(self.ns + "/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.test_data_pub = rospy.Publisher(self.ns + "/test", Float32MultiArray, queue_size=1)
        self.mpc_ctrl_pub = rospy.Publisher(self.ns + "/opti_input", Float32MultiArray, queue_size=1)
        self.thrust_est_pub = rospy.Publisher(self.ns + "/thrust_est", Float32, queue_size=1)
        if self.pub_prev_traj_flag:
            self.prev_traj_pub = rospy.Publisher(self.ns + "/traj", Path, queue_size=1)
        if self.pub_lin_interp_traj_flag:
            self.lin_interp_traj_pub = rospy.Publisher(self.ns + "/lin_interp_traj", PoseArray, queue_size=1)
        if self.pub_local_ref_traj_flag:
            self.local_ref_traj_pub = rospy.Publisher(self.ns + "/local_ref_traj", Path, queue_size=1)

        # subs
        # self.model_states_sub = rospy.Subscriber(
        #     "/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1
        # )
        if self.follow_mode == "traj":
            self.tgt_traj_sub = rospy.Subscriber(self.ns + "/target_traj", Path, self.tgt_traj_cb, queue_size=1)
        elif self.follow_mode == "waypoint":
            self.tgt_pose_sub = rospy.Subscriber(self.ns + "/target_pose", PoseStamped, self.tgt_pose_cb, queue_size=1)
        self.mavros_state_sub = rospy.Subscriber(self.ns + "/mavros/state", State, self.mavros_state_cb, queue_size=1)
        self.matrix_q_sub = rospy.Subscriber(self.ns + "/matrix_q", Float32MultiArray, self.matrix_q_cb, queue_size=1)
        self.matrix_r_sub = rospy.Subscriber(self.ns + "/matrix_r", Float32MultiArray, self.matrix_r_cb, queue_size=1)
        self.imu_sub = rospy.Subscriber(self.ns + "/mavros/imu/data", Imu, self.imu_cb, queue_size=1)
        self.real_pose_sub = rospy.Subscriber(self.ns + "/real_pose", PoseStamped, self.real_pose_cb, queue_size=1)
        self.real_twist_sub = rospy.Subscriber(self.ns + "/real_twist", TwistStamped, self.real_twist_cb, queue_size=1)
        self.max_atti_ang_sub = rospy.Subscriber(self.ns + "/max_atti_ang", Float32, self.max_atti_ang_cb, queue_size=1)
        self.max_lin_acc_sub = rospy.Subscriber(self.ns + "/max_lin_acc", Float32, self.max_lin_acc_cb, queue_size=1)

        # srvs
        rospy.wait_for_service(self.ns + "/mavros/set_mode", rospy.Duration(3))
        self.set_mode_proxy = rospy.ServiceProxy(self.ns + "/mavros/set_mode", SetMode)
        self.arm_proxy = rospy.ServiceProxy(self.ns + "/mavros/cmd/arming", CommandBool)
        self.enable_ctrl_srv = rospy.Service(self.ns + "/enable_ctrl", SetBool, self.enable_ctrl_cb)
        self.get_mpc_matrix_srv = rospy.Service(self.ns + "/get_mpc_matrix", GetMPCMatrix, self.get_mpc_matrix_cb)
        self.set_mpc_matrix_srv = rospy.Service(self.ns + "/set_mpc_matrix", SetMPCMatrix, self.set_mpc_matrix_cb)
        self.init_dyn_cfg_flag = True
        self.dyn_cfg_srv = DynamicReconfigureServer(ControllerConfig, self.dyncfg_cb)
        self.get_param_srv = rospy.Service(self.ns + "/get_param", GetControllerParam, self.get_param_cb)
        self.set_controller_srv = rospy.Service(self.ns + "/set_controller", SetController, self.set_controller_cb)
        self.ret_home_srv = rospy.Service(self.ns + "/return_home", Trigger, self.return_home_cb)

        rospy.loginfo(f"controller for iris_{self.uav_idx} is ready.")

    def run(self):
        self.arm_proxy(value=True)
        attitude_msg = AttitudeTarget()
        while not rospy.is_shutdown():
            if not self.enable_ctrl:
                self.loop_rate.sleep()
                continue
            if not self.mavros_state.armed:
                self.arm_proxy(value=True)
            self.set_mode_proxy(custom_mode="OFFBOARD")

            if self.controller == "px4":
                # NOTE: the following transformation is only used when px4 uses GPS as the global positioning source
                if 0:
                    if self.init_pose is None:
                        continue
                    T_world2local = tft.inverse_matrix(
                        pose_msg_to_matrix(self.init_pose)
                    )  # transformation from world to px4's local origin
                    cmd_pose_local = apply_transformation_pose(matrix_to_pose_msg(T_world2local), self.tgt_pose.pose)
                pose_cmd_msg = PoseStamped()
                pose_cmd_msg.header.frame_id = "world"
                pose_cmd_msg.header.stamp = rospy.Time.now()
                if self.follow_mode == "waypoint":
                    cmd_pose_local = self.tgt_pose.pose
                elif self.follow_mode == "traj":
                    offset_idx = 3  # NOTE: use offset to provide forward looking
                    cmd_pose_local = self.tgt_traj.poses[0 + offset_idx]
                pose_cmd_msg.pose = cmd_pose_local
                self.px4_pos_cmd_pub.publish(pose_cmd_msg)

            elif self.controller == "nmpc":
                # preparing x and u
                u_ref = np.zeros((self.nmpcc.N, 4))
                if self.follow_mode == "waypoint":
                    if self.lin_interp_enable:
                        pose_interp = linear_interpolation(
                            self.state.pose_now.pose, self.tgt_pose.pose, num_steps=self.nmpcc.N + 1, max_distance=5
                        )[: self.nmpcc.N + 1]
                        x_ref = np.array(
                            [
                                [
                                    pose.position.x,
                                    pose.position.y,
                                    pose.position.z,
                                    self.tgt_speed.linear.x,
                                    self.tgt_speed.linear.y,
                                    self.tgt_speed.linear.z,
                                ]
                                for pose in pose_interp
                            ]
                        )
                        u_ref[:, 3] = [
                            -tft.euler_from_quaternion(pose_msg2ar(pose)[3:])[2] for pose in pose_interp[:-1]
                        ]
                        if self.pub_lin_interp_traj_flag:
                            self.lin_interp_msg.poses = pose_interp
                            self.lin_interp_msg.header.frame_id = "world"
                            self.lin_interp_msg.header.stamp = rospy.Time.now()
                    else:
                        euler = tft.euler_from_quaternion(pose_msg2ar(self.tgt_pose.pose)[3:])
                        u_ref[:, 3] = -euler[2]  # yaw command
                        x_ref = np.array(
                            [
                                [
                                    self.tgt_pose.pose.position.x,
                                    self.tgt_pose.pose.position.y,
                                    self.tgt_pose.pose.position.z,
                                    self.tgt_speed.linear.x,
                                    self.tgt_speed.linear.y,
                                    self.tgt_speed.linear.z,
                                ]
                            ]
                            * (self.nmpcc.N + 1),
                            dtype=np.float64,
                        )
                elif self.follow_mode == "traj":
                    local_ref_traj = self.get_local_ref_traj()
                    # print(local_ref_traj)
                    # print('-'*50)
                    x_ref = np.zeros((self.nmpcc.N + 1, 6))
                    for i in range(self.nmpcc.N + 1):
                        x_ref[i] = [
                            local_ref_traj.poses[i].pose.position.x,
                            local_ref_traj.poses[i].pose.position.y,
                            local_ref_traj.poses[i].pose.position.z,
                            self.tgt_speed.linear.x,
                            self.tgt_speed.linear.y,
                            self.tgt_speed.linear.z,
                        ]

                if self.h_pidc_enable:
                    x_ref_delta = self.h_pidc.control(
                        self.tgt_pose.pose.position.z, self.state.pose_now.pose.position.z
                    )
                    x_ref[:, 2] += x_ref_delta  # altitude compensate
                for i in range(len(x_ref)):
                    x_ref[i] = ENU2NED_Position(x_ref[i])
                x_curr = np.array(
                    [
                        self.state.pose_now.pose.position.x,
                        self.state.pose_now.pose.position.y,
                        self.state.pose_now.pose.position.z,
                        self.state.vel_now.linear.x,
                        self.state.vel_now.linear.y,
                        self.state.vel_now.linear.z,
                    ]
                )
                x_curr = ENU2NED_Position(x_curr)

                # solve MPC
                self.nmpcc.set_param("Q", self.nmpc_q)
                self.nmpcc.set_param("Qf", self.nmpc_qf)
                self.nmpcc.set_param("R", self.nmpc_r)
                self.nmpcc.set_param("mass", self.mass)
                self.nmpcc.set_param("max_atti_ang", self.max_atti_ang)
                self.nmpcc.set_param("max_atti_ang_rate", self.max_atti_ang_rate)
                self.nmpcc.set_param("max_lin_acc", self.max_lin_acc)
                opt_u_raw, opt_x = self.nmpcc.solve(x_curr, x_ref, u_ref, return_first_u=False)  # XXX: core step
                opt_u = opt_u_raw[0]

                # set command msg
                opt_u = [opt_u[0], opt_u[2], opt_u[1], -opt_u[3]]  # NED to ENU
                thrust = opt_u[0]
                throttle = thrust2signal(thrust, self.motor_constant, self.input_scale)
                att_quad = tft.quaternion_from_euler(opt_u[1], opt_u[2], opt_u[3])
                attitude_msg.orientation.x = att_quad[0]
                attitude_msg.orientation.y = att_quad[1]
                attitude_msg.orientation.z = att_quad[2]
                attitude_msg.orientation.w = att_quad[3]
                attitude_msg.thrust = throttle
                self.px4_atti_cmd_pub.publish(attitude_msg)

                for i in range(len(opt_x)):
                    opt_x[i] = NED2ENU_Position(opt_x[i])
                self.mpc_ctrl_pub.publish(Float32MultiArray(data=opt_u))
                if self.pub_opti_traj_flag:
                    self.pub_opti_traj(opt_x, opt_u_raw)

            if self.pub_lin_interp_traj_flag and self.lin_interp_enable:
                self.lin_interp_traj_pub.publish(self.lin_interp_msg)
            if self.pub_prev_traj_flag:
                self.pub_prev_traj()

            self.loop_rate.sleep()

    def return_home_cb(self, req):
        set_pose_msg(self.tgt_pose.pose, self.init_pose[:3], tft.quaternion_from_euler(*self.init_pose[3:]))
        tgt_pose_pub = rospy.Publisher(self.ns + "/target_pose", PoseStamped, queue_size=1, latch=True)
        self.tgt_pose.header.frame_id = f"iris_{self.uav_idx}"
        tgt_pose_pub.publish(self.tgt_pose)
        tgt_pose_pub.unregister()
        response = TriggerResponse()
        response.success = True
        response.message = "return home"
        return response

    def set_controller_cb(self, req):
        response = SetControllerResponse()
        if req.controller not in ["px4", "nmpc"]:
            rospy.logerr(f"controller {req.controller} does not exist!")
            response.success = False
            return response
        self.controller = req.controller
        rospy.loginfo(f"[uav_{self.uav_idx}] set controller to {self.controller}")
        response = SetControllerResponse()
        response.success = True
        return response

    def max_lin_acc_cb(self, msg: Float32):
        self.max_lin_acc = msg.data
        # rospy.loginfo(f"max_lin_acc={self.max_lin_acc}")

    def tgt_traj_cb(self, msg: Path):
        self.tgt_traj = msg

    def get_param_cb(self, req):
        response = GetControllerParamResponse()
        try:
            response.param_value = self.__dict__[req.param_name]
            response.success = True
        except:
            rospy.logerr(f"param {req.param_name} does not exist!")
            response.param_value = -1
            response.success = False
        return response

    def max_atti_ang_cb(self, msg: Float32):
        if msg.data > np.deg2rad(50) or msg.data < 0:
            rospy.logerr(f"max_atti_ang={np.rad2deg(msg.data)} exceeds allowd range[0,50]!")
        else:
            self.max_atti_ang = msg.data

    def mavros_state_cb(self, msg: State):
        self.mavros_state = msg

    def enable_ctrl_cb(self, req: SetBoolRequest):
        self.enable_ctrl = req.data
        response = SetBoolResponse()
        response.success = True
        response.message = "Set enable_ctrl to " + str(self.enable_ctrl)
        return response

    def model_states_cb(self, msg: ModelStates):
        idx_in_names = msg.name.index("iris_" + str(self.uav_idx))
        self.state.pose_now.pose = msg.pose[idx_in_names]
        self.state.vel_now = msg.twist[idx_in_names]

    def real_pose_cb(self, msg: PoseStamped):
        self.state.pose_now.pose = msg.pose

    def real_twist_cb(self, msg: TwistStamped):
        self.state.vel_now = msg.twist

    def tgt_pose_cb(self, msg: PoseStamped):
        self.tgt_pose = msg
        self.tgt_pose.header.frame_id = "world"
        # rospy.loginfo(f"got pose: {msg.pose.position.x,msg.pose.position.y,msg.pose.position.z}")

    def matrix_q_cb(self, msg: Float32MultiArray):
        self.nmpc_q = np.diag(msg.data)
        # rospy.loginfo(f"got matrix_q:\r\n{self.mpc_q}")

    def matrix_r_cb(self, msg: Float32MultiArray):
        self.nmpc_r = np.diag(msg.data)
        # rospy.loginfo(f"got matrix_r:\r\n{self.mpc_r}")

    def dyncfg_cb(self, config: ControllerConfig, level):
        keys_q = ["q_x", "q_y", "q_z", "q_vx", "q_vy", "q_vz"]
        keys_qf = ["qf_x", "qf_y", "qf_z", "qf_vx", "qf_vy", "qf_vz"]
        keys_r = ["r_thrust", "r_roll", "r_pitch", "r_yaw"]
        if self.init_dyn_cfg_flag:
            self.init_dyn_cfg_flag = False
            config["mass"] = self.mass
            for i, key in enumerate(keys_q):
                config[key] = float(self.nmpc_q[i, i])
            for i, key in enumerate(keys_qf):
                config[key] = float(self.nmpc_qf[i, i])
            for i, key in enumerate(keys_r):
                config[key] = float(self.nmpc_r[i, i])
            config["h_pidc_kp"] = self.h_pidc.kp
            config["h_pidc_ki"] = self.h_pidc.ki
            config["h_pidc_kd"] = self.h_pidc.kd
            config["h_pidc_int_clamp"] = self.h_pidc.int_clamp
            config["h_pidc_int_err_th"] = self.h_pidc.int_err_th
            config["h_pidc_enable"] = self.h_pidc_enable
            config["max_atti_ang"] = float(np.rad2deg(self.max_atti_ang))
            config["max_lin_acc"] = float(self.max_lin_acc)
            config["max_atti_ang_rate"] = float(np.rad2deg(self.max_atti_ang_rate))
            return config
        self.nmpc_r = np.diag([config[key] for key in keys_r])  # set values in order
        self.nmpc_q = np.diag([config[key] for key in keys_q])
        self.nmpc_qf = np.diag([config[key] for key in keys_qf])
        self.mass = config["mass"]
        self.h_pidc.kp = config["h_pidc_kp"]
        self.h_pidc.ki = config["h_pidc_ki"]
        self.h_pidc.kd = config["h_pidc_kd"]
        self.h_pidc.int_clamp = config["h_pidc_int_clamp"]
        self.h_pidc.int_err_th = config["h_pidc_int_err_th"]
        self.h_pidc_enable = config["h_pidc_enable"]
        self.max_lin_acc = config["max_lin_acc"]
        self.max_atti_ang = np.deg2rad(config["max_atti_ang"])
        self.max_atti_ang_rate = np.deg2rad(config["max_atti_ang_rate"])
        # print(f"{'Q':<5}: \r\n{self.nmpc_q}")
        # print(f"{'R':<5}: \r\n{self.nmpc_r}")
        # print(f"{'Qf':<5}: \r\n{self.nmpc_qf}")
        # print(f"{'mass':<5}: {self.mass}")
        return config

    def get_mpc_matrix_cb(self, req):
        response = GetMPCMatrixResponse()
        response.Q = np.diag(self.nmpc_q)
        response.R = np.diag(self.nmpc_r)
        return response

    def set_mpc_matrix_cb(self, req):
        self.nmpc_q = np.diag(req.Q)
        self.nmpc_r = np.diag(req.R)
        response = SetMPCMatrixResponse()
        response.success = True
        # rospy.loginfo(f"set matrix Q:\r\n{self.nmpc_q}")
        # rospy.loginfo(f"set matrix R:\r\n{self.nmpc_r}")
        return response

    def pub_opti_traj(self, opti_traj: np.array, opti_u: np.array, viz_step=3):
        marker_array = MarkerArray()
        for i in range(len(opti_traj)):
            if i % viz_step != 0 or i < viz_step:
                continue
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "traj_viz"
            marker.id = i
            marker.action = Marker.ADD

            marker.pose.position = Point(opti_traj[i][0], opti_traj[i][1], opti_traj[i][2])
            if i < len(opti_u):
                quat = tft.quaternion_from_euler(opti_u[i][1], opti_u[i][2], opti_u[i][3])
            else:
                quat = tft.quaternion_from_euler(opti_u[i - 1][1], opti_u[i - 1][2], opti_u[i - 1][3])
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]

            marker.color.a = (1 - (i / len(opti_traj))) * 0.4
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = "package://px4/Tools/sitl_gazebo/models/iris/meshes/iris.stl"
            marker.scale = Vector3(1, 1, 1)

            # marker.type = Marker.ARROW
            # marker.scale = Vector3(0.1*opti_u[0], 0.01, 0.01)
            marker_array.markers.append(marker)
        self.opti_traj_pub.publish(marker_array)

    def imu_cb(self, msg: Imu):
        self.imu = msg
        self.estimate_thrust()

    def estimate_thrust(self):
        """estimate thrust from IMU according to the following equation:
        T = abs(abs(9.8 - az) * m / (cos(phi) * cos(theta)))"""
        [phi, theta, psi] = tft.euler_from_quaternion(
            pose_msg2ar(self.state.pose_now.pose)[3:]
        )  # euler angle in inertial frame
        [phi, theta, psi, _, _, _] = ENU2NED_Position([phi, theta, psi, 0, 0, 0])
        rotate_matrix = tft.euler_matrix(phi, theta, psi)
        acc_in_inertial = rotate_matrix @ np.array(
            [[self.imu.linear_acceleration.x], [self.imu.linear_acceleration.y], [self.imu.linear_acceleration.z], [1]]
        )
        az = acc_in_inertial[2]
        T = abs(abs(9.8 - az) * self.mass / (np.cos(phi) * np.cos(theta)))
        self.thrust_est = T[0]
        self.thrust_est_pub.publish(self.thrust_est)

    def pub_prev_traj(self):
        pose_stamp_msg = PoseStamped()
        pose_stamp_msg.pose = self.state.pose_now.pose
        pose_stamp_msg.header.stamp = rospy.Time.now()
        pose_stamp_msg.header.frame_id = "world"
        self.prev_traj_msg.poses.append(pose_stamp_msg)
        self.prev_traj_msg.header.frame_id = "world"
        self.prev_traj_msg.header.stamp = rospy.Time.now()
        if len(self.prev_traj_msg.poses) >= self.prev_traj_msg_len:
            self.prev_traj_msg.poses.pop(0)
        self.prev_traj_pub.publish(self.prev_traj_msg)

    def get_local_ref_traj(
        self,
    ) -> Path:
        """get MPC ref trajectory from the whole path"""
        # TODO: use linear interpolation to get the ref path given a refernce speed or distance
        nearest_idx = get_nearest_point_idx(self.tgt_traj, self.state.pose_now.pose, shape="circle")
        ref_traj = Path()
        ref_traj.header.frame_id = "world"
        ref_traj.header.stamp = rospy.Time.now()
        N = self.nmpcc.N
        dt = self.nmpcc.T
        for t in np.arange(0, N + dt, dt):
            future_index = nearest_idx + int(t / dt)
            if future_index < len(self.tgt_traj.poses):
                ref_traj.poses.append(self.tgt_traj.poses[future_index])
            else:
                ref_traj.poses.append(self.tgt_traj.poses[-1])
                # future_index %= len(self.tgt_traj.poses)
        if self.pub_local_ref_traj_flag:
            self.local_ref_traj_pub.publish(ref_traj)
        return ref_traj


if __name__ == "__main__":
    rospy.init_node(f"uav_controller_node", anonymous=False)
    uav_idx = rospy.get_param("~uav_idx", 0)
    x = rospy.get_param("~x", 0)
    y = rospy.get_param("~y", 0)
    z = rospy.get_param("~z", 2)
    roll = rospy.get_param("~raw", 0)
    pitch = rospy.get_param("~pitch", 0)
    yaw = rospy.get_param("~yaw", 0)
    controller = rospy.get_param("~controller", "nmpc")
    h_pidc_enable = rospy.get_param("~h_pidc_enable", False)
    lin_interp_enable = rospy.get_param("~lin_interp_enable", False)
    follow_mode = rospy.get_param("~follow_mode", "waypoint")

    dyn_mode = "power_saving"  # performance, moderate, power_saving
    param_file = os.path.join(sys.path[0], "../config/uav_params.yaml")
    param_dict = rosparam.load_file(param_file, f"iris_{uav_idx}")[0][0]
    ctrl_rate = param_dict["nmpcc"]["ctrl_rate"]
    nmpcc_params = param_dict["nmpcc"][dyn_mode]
    nmpcc_params["max_atti_ang"] = np.deg2rad(nmpcc_params["max_atti_ang"])
    nmpcc_params["max_atti_ang_rate"] = np.deg2rad(nmpcc_params["max_atti_ang_rate"])
    nmpcc_params["Q"] = np.diag(nmpcc_params["Q"])
    nmpcc_params["Qf"] = np.diag(nmpcc_params["Qf"])
    nmpcc_params["R"] = np.diag(nmpcc_params["R"])
    nmpcc_params["max_thrust"] = nmpcc_params["max_thrust"]
    mass = param_dict["physics"]["mass"]
    h_pidc_params = param_dict["pidc"]  # params of height PID controller for the outer loop
    init_pose = np.array([x, y, z, roll, pitch, yaw])
    uav_controller = UAVControllerNode(
        uav_idx,
        mass,
        init_pose,
        nmpcc_params,
        h_pidc_params,
        ctrl_rate=ctrl_rate,
        controller=controller,
        h_pidc_enable=h_pidc_enable,
        lin_interp_enable=lin_interp_enable,
        follow_mode=follow_mode,
        prev_traj_msg_len=param_dict["prev_traj_msg_len"],
    )
    uav_controller.run()
