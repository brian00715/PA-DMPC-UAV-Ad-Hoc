"""
 # @ Author: Kenneth Simon
 # @ Create Time: 2022-05-07 20:29:25
 # @ Modified time: 2022-05-08 21:43:56
 # @ Description: NMPC controller based on CasADi. 
 # Use dynamics model, 4th runge kutta integrator and direct multiple shooting method.
 # x: [x, y, z, dx, dy, dz]
 # u: [thrust, phi, theta, psi], where psi(yaw) is actually a target state too.
 """


import casadi as ca
import numpy as np
import ipdb


class NMPCC:
    """Nonlinear Model Predictive Controler"""

    def __init__(
        self,
        T=0.01,
        N=30,
        Q=np.diag([10, 10, 10, 0, 0, 0]),
        R=np.diag([0, 0, 0, 0]),
        Qf=None,
        solver_params: dict = None,
        prob_params: dict = None,  # optimal control problem parameters
        integrator="euler",
    ):
        """

        Args:
            T (float, optional): Prediction time step.
            N (int, optional): Prediction length (steps).
            Q (np.ndarray, optional):  States weight matrix.
            R (np.ndarray, optional):  Input weight matrix.
            Qf (np.ndarray, optional): Final weight matrix.
            casadi_params (dict, optional): casadi solver parameters
            prob_params (dict, optional): optimal control problem parameters like mass, max_thrust, max_atti_ang, max_thrust_rate, max_atti_ang_rate
        """

        """TODO: split system formulation"""
        self.T = T  # time step (s)
        self.N = N  # horizon length
        g = 9.8
        self.integrator = integrator

        self.opti = ca.Opti()
        self.control_dim = prob_params["control_dim"]
        self.state_dim = prob_params["state_dim"]

        # create casadi parameters
        self.ca_params = {
            "Q": self.opti.parameter(self.state_dim, self.state_dim),
            "R": self.opti.parameter(self.control_dim, self.control_dim),
            "Qf": self.opti.parameter(self.state_dim, self.state_dim),
            # "neighbor_pos": self.opti.parameter(1, 2),
        }
        prob_params["Q"] = Q
        prob_params["R"] = R
        prob_params["Qf"] = Qf
        for param, value in prob_params.items():
            if isinstance(value, (int, float)):
                self.ca_params[param] = self.opti.parameter(1)
            if isinstance(value, np.ndarray):
                if value.ndim == 1:
                    self.ca_params[param] = self.opti.parameter(value.shape[0])
                elif value.ndim == 2:
                    self.ca_params[param] = self.opti.parameter(value.shape[0], value.shape[1])
        # set the parameter initial values
        for param in self.ca_params:
            self.opti.set_value(self.ca_params[param], prob_params[param])
        if Qf is None:
            self.opti.set_value(self.ca_params["Qf"], Q)
        else:
            self.opti.set_value(self.ca_params["Qf"], Qf)
        # self.opti.set_value(self.ca_params["neighbor_pos"], np.array([1, 1])) # TODO

        print("NMPCC Parameters".ljust(50, "-"))
        print("{:<25}: {}".format("init_pose", prob_params["init_pose"]))
        print("{:<25}: {}".format("mass (kg)", prob_params["mass"]))
        print("{:<25}: {}".format("max_thrust", prob_params["max_thrust"]))
        print("{:<25}: {}".format("max_atti_ang (deg)", np.rad2deg(prob_params["max_atti_ang"])))
        print("{:<25}: {}".format("max_thrust_rate (N/s)", prob_params["max_thrust_rate"]))
        print("{:<25}: {}".format("max_atti_ang_rate (deg/s)", np.rad2deg(prob_params["max_atti_ang_rate"])))
        print("{:<25}: {}".format("max_lin_acc (m/s^2)", prob_params["max_lin_acc"]))
        print("{:<25}: {}".format("T", T))
        print("{:<25}: {}".format("N", N))
        print(
            "{:<25}: {}".format(
                "Q", np.array2string(Q, separator=", ", prefix=" " * 27, formatter={"all": lambda x: f"{x}"})
            )
        )
        if Qf is None:
            print("{:<25}: None".format("Qf"))
        else:
            print(
                "{:<25}: {}".format(
                    "Qf", np.array2string(Qf, separator=", ", prefix=" " * 27, formatter={"all": lambda x: f"{x}"})
                )
            )
        print(
            "{:<25}: {}".format(
                "R", np.array2string(R, separator=", ", prefix=" " * 27, formatter={"all": lambda x: f"{x}"})
            )
        )
        print("-" * 50)

        self.x0 = self.opti.parameter(1, self.state_dim)  # initial state
        # history states and controls
        self.x_opti = np.ones((self.N + 1, self.state_dim)) * prob_params["init_pose"]
        self.u_opti = np.zeros((self.N, self.control_dim))

        # state variables
        self.var_states = self.opti.variable(self.N + 1, self.state_dim)
        # x = self.var_states[:, 0]
        # y = self.var_states[:, 1]
        # z = self.var_states[:, 2]
        # dx = self.var_states[:, 3]
        # dy = self.var_states[:, 4]
        # dz = self.var_states[:, 5]
        # control variables
        self.var_controls = self.opti.variable(self.N, self.control_dim)
        thrust = self.var_controls[:, 0]
        phi = self.var_controls[:, 1]  # roll
        theta = self.var_controls[:, 2]  # pitch
        psi = self.var_controls[:, 3]  # yaw

        self.u_ref = self.opti.parameter(self.N, self.control_dim)
        self.x_ref = self.opti.parameter(self.N + 1, self.state_dim)

        # dynamics differential equation
        ## uav dynamics in NED frame
        self.dde = lambda x_, u_: ca.vertcat(
            *[
                x_[3],  # dx
                x_[4],  # dy
                x_[5],  # dz
                -u_[0]
                / self.ca_params["mass"]
                * (ca.cos(u_[3]) * ca.sin(u_[2]) * ca.cos(u_[1]) + ca.sin(u_[3]) * ca.sin(u_[1])),  # ddx
                -u_[0]
                / self.ca_params["mass"]
                * (ca.sin(u_[3]) * ca.sin(u_[2]) * ca.cos(u_[1]) - ca.cos(u_[3]) * ca.sin(u_[1])),  # ddy
                g - u_[0] / self.ca_params["mass"] * (ca.cos(u_[1]) * ca.cos(u_[2])),  # ddz
            ]
        )

        # cost function
        cost = 0
        for i in range(self.N + 1):
            state_error_ = self.var_states[i, :] - self.x_ref[i, :]
            if i < self.N:
                control_error_ = self.var_controls[i, :] - self.u_ref[i, :]
                cost = (
                    cost
                    + ca.mtimes([state_error_, self.ca_params["Q"], state_error_.T])
                    + ca.mtimes([control_error_, self.ca_params["R"], control_error_.T])
                )
            else:
                cost = cost + ca.mtimes([state_error_, self.ca_params["Qf"], state_error_.T])
        self.opti.minimize(cost)

        # constraints

        ## initial condition
        self.opti.subject_to(self.var_states[0, :] == self.x0)

        ## state space constraint
        for i in range(self.N):
            if self.integrator == "euler":
                x_next = self.var_states[i, :] + self.dde(self.var_states[i, :], self.var_controls[i, :]).T * self.T
            elif self.integrator == "rk4":
                x_next = self.runge_kutta(self.dde, self.T, self.var_states[i, :], self.var_controls[i, :].T)
            self.opti.subject_to(self.var_states[i + 1, :] == x_next)

        ## input limits
        self.opti.subject_to(self.opti.bounded(0, thrust, self.ca_params["max_thrust"]))
        self.opti.subject_to(self.opti.bounded(-self.ca_params["max_atti_ang"], phi, self.ca_params["max_atti_ang"]))
        self.opti.subject_to(self.opti.bounded(-self.ca_params["max_atti_ang"], theta, self.ca_params["max_atti_ang"]))

        ## input speed limits
        max_atti_ang_r = self.ca_params["max_atti_ang_rate"]  # alias for readability
        max_lin_acc = self.ca_params["max_lin_acc"]
        for i in range(self.N - 1):
            du = (self.var_controls[i + 1, :] - self.var_controls[i, :]) / self.T
            dx = (self.var_states[i + 1, :] - self.var_states[i, :]) / self.T
            # self.opti.subject_to(self.opti.bounded(-max_thrust_rate, du[0], max_thrust_rate))
            self.opti.subject_to(self.opti.bounded(-max_atti_ang_r, du[1], max_atti_ang_r))
            self.opti.subject_to(self.opti.bounded(-max_atti_ang_r, du[2], max_atti_ang_r))
            self.opti.subject_to(self.opti.bounded(-max_atti_ang_r, du[3], max_atti_ang_r))
            self.opti.subject_to(self.opti.bounded(-max_lin_acc, dx[3], max_lin_acc))  # ddx
            self.opti.subject_to(self.opti.bounded(-max_lin_acc, dx[4], max_lin_acc))  # ddy
            self.opti.subject_to(self.opti.bounded(-max_lin_acc, dx[5], max_lin_acc))  # ddz

        # collision avoidance constriants
        for i in range(5):
            self.opti.subject_to(
                ca.sqrt(
                    (self.var_states[i, 0] - self.neighbor_pos[0]) ** 2
                    + (self.var_states[i, 1] - self.neighbor_pos[1]) ** 2
                )
                >= 0.2
            )

        if solver_params is not None:
            opts_setting = solver_params
        else:
            opts_setting = {
                "ipopt.max_iter": 2000,
                "ipopt.print_level": 0,
                "print_time": 0,
                "ipopt.acceptable_tol": 1e-8,
                "ipopt.acceptable_obj_change_tol": 1e-6,
                # "expand":True
            }
        self.opti.solver("ipopt", opts_setting)

    def solve(self, x_curr: np.array, x_ref, u_ref, return_first_u=True):
        self.opti.set_value(self.x_ref, x_ref)
        self.opti.set_value(self.u_ref, u_ref)
        self.opti.set_value(self.x0, x_curr.T)

        # provide the initial guess of the optimization targets
        self.opti.set_initial(self.var_states, self.x_opti)
        self.opti.set_initial(self.var_controls, self.u_opti)

        # solve the problem
        sol = self.opti.solve()

        # obtain the control input
        self.u_opti = sol.value(self.var_controls)
        self.x_opti = sol.value(self.var_states)
        return self.u_opti[0, :] if return_first_u else self.u_opti, self.x_opti

    def set_param(self, param_name, value):
        if param_name in self.ca_params:
            self.opti.set_value(self.ca_params[param_name], value)
        else:
            print(f"Parameter {param_name} not found!")

    @staticmethod
    def runge_kutta(f, dt, x, u):
        k1 = f(x, u)
        k2 = f(x + dt / 2 * k1.T, u)
        k3 = f(x + dt / 2 * k2.T, u)
        k4 = f(x + dt * k3.T, u)
        new_x = x + dt / 6 * (k1.T + 2 * k2.T + 2 * k3.T + k4.T)
        return new_x

    def add_constraints(self, constraints):
        # TODO
        pass

    def add_cost(self, cost):
        # TODO
        pass


if __name__ == "__main__":
    mpc_controller = NMPCC(np.array([0, 0, 0, 0, 0, 0]), -1, 1, -1, 1, -np.pi / 6, np.pi / 6)
    x_ref = np.array([(0, 0, 2, 0, 0, 0)] * (mpc_controller.N + 1))
    u_ref = np.zeros((mpc_controller.N, 4))
    x_curr = np.array([0, 0, 0, 0, 0, 0])
    opt_u = mpc_controller.solve(x_curr, x_ref, u_ref)
    # print(f"opt_u: {opt_u[0]:.2f,opt_u[1]:.2f,opt_u[2]:.2f,opt_u[3]:.2f} pose: {x_curr}")
