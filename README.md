## Description 🚁

Code archive of my bachelor's thesis *Research on Cooperative Control Algorithm for UAV Ad-Hoc Network*.

![](./test/demo.gif)

## Prerequisites 🛠️

## Installation

- ROS Noetic 
- Gazebo Classic 11
- PX4 13
- Apt Packages:
  
  ```shell
  sudo apt install -y tmux zsh
  ```
- Python Packages:
  
  ```shell
  pip install -r requirements.txt
  ```

## Operation Instructions 🚀

### Quick Start

One command to rule them all:

```shell
roscd dmpc_uav_ad_hoc && sh ./scripts/run_all.sh -n <uav_num> -t <simu_time> -a -r # delete -r if you don't want to run immediately.
```

Or, schedule all the simulations automatically:

```shell
python ./scripts/simu_auto_scheduer.py
```

### Manual Start

1. Launch the simulation environment:
   
   ```shell
   roslaunch dmpc_uav_ad_hoc simu_env.launch
   ```

2. Launch the swarm controller node:
   
   ```shell
   rosrun dmpc_uav_ad_hoc swarm_controller_launcher.py --uav_num <num>
   ```
   
    For specifing setpoint via rviz:
   
   ```shell
   rosrun dmpc_uav_ad_hoc swarm_controller_launcher.py --uav_num <num> --rviz --remap_target_topic
   ```

3. Enable the controller:
    To enable the controller for a UAV, use:
   
   ```shell
   rosservice call /iris_0/enable_ctrl "data: true"
   ```
   
    Manage controllers with:
   
   ```shell
   python ./scripts/swarm_controller_switch.py on # enable all controllers
   python ./scripts/swarm_controller_switch.py off # disable all controllers
   ```

### Debugging 🐞

#### dynamic reconfigure

`uav_controller_node.py` supports ROS dynamic reconfigure mechanism. The tunable parameters are defined in `./config/nmpc.cfg`.You can use `rqt_reconfigure` to tune the parameters online.

> Note:exclamation:: Set default parameters in `uav_controller_node.py` rather than in `nmpc.cfg` to avoid unexpected behavior.

#### Useful Commands 🛠️

- Instantly shut down Gazebo:
  
  ```shell
  ./scripts/kill_gazebo.sh
  ```

- Instantly terminate ROS processes:
  
  ```shell
  ./scripts/kill_ros.sh
  ```

- Instantly terminate all processes:
  
  ```shell
  ./scripts/kill_all.sh
  ```

## ROS Communication Structure 📡

- `src/uav_controller_node.py`

  Subscribtions:

  - `/<uav_name>/real_pose`
  - `/<uav_name>/target_pose`
  - `/<uav_name>/mpc_matrix_q`
  - `/<uav_name>/mpc_matrix_r`

  Publications:

  - `/<uav_name>/opti_traj`
  - `/<uav_name>/opti_input`

  Services:

  - `/<uav_name>/enable_ctrl`

  - `/<uav_name>/set_mpc_matrix`
    msg type: `dmpc_uav_ad_hoc/SetMPCMatrix.srv`


    > Note:exclamation:: Service calling may be laggy, which may cause a decrease in the running speed of other nodes. If synchronous configuration isn't necessarily required, use the topic `/<uav_name>/mpc_matrix_q` and `/<uav_name>/mpc_matrix_r` instead.

  - `/<uav_name>/get_mpc_matrix`
  - `/<uav_name>/set_controller`
    msg type: `dmpc_uav_ad_hoc/SetController.srv`
    
    Set the controller type (nmpc/px4) of the UAV. This action can be performed online.



- `src/perf_analysis_node.py`

  Subscribtions:

  - `/<uav_name>/real_pose`
  - `/<uav_name>/real_twist`
  - `/<uav_name>/thrust_est`
  - `/crowds/target_net_capacity`
  - `/enable_simu`

  Publications:

  - `/<uav_name>/perf_analyzer/induced_power`
  - `/<uav_name>/perf_analyzer/profile_power`
  - `/<uav_name>/perf_analyzer/instant_power`
  - `/perf_analyzer/ave_curr_net_cap`
    Average current network capacity of all UAVs.
  - `/perf_analyzer/ave_tgt_net_cap`
    Average target network capacity of all UAVs.
  - `/perf_analyzer/ave_instant_power`
  - `/perf_analyzer/cpu_usage`
    Instant CPU usage of the simulation environment. 

- `src/scheduler_node.py`

  Subscribtions:
  - `<uav_name>/target_pose` 
  - `<uav_name>/real_pose` 
  - `/crowds/target_net_capacity`
  - `enable_simu` 

  Publications:
  - `<uav_name>/matrix_q`
  - `<uav_name>/matrix_r`
  - `<uav_name>/max_atti_ang`
  - `<uav_name>/max_lin_acc`

### Some Import Mathematical Formulation

#### Quadrotor Dynamics in NED coordinate

In this project, the quadrotor dynamics in NED coordinate is formulated as
   
  $$
  \begin{aligned}
    &\boldsymbol{x}=[x_p,y_p,z_p,\dot{x}_p,\dot{y}_p,\dot{z}_p]^T\\
    &\boldsymbol{u}=[T,{\phi,\theta,\psi}]^T
  \end{aligned}
  $$
 
  $$
  \dot{\boldsymbol x}=
  \left[
    \begin{matrix}
      \dot{x}_p\\
      \dot{y}_p\\
      \dot{z}_p\\
      \ddot{x}_p\\
      \ddot{y}_p\\
      \ddot{z}_p\\
    \end{matrix}
  \right] = 
  \left[\begin{matrix}
    \dot{x}_p\\
    \dot{y}_p\\
    \dot{z}_p\\
    -\frac{T}{m}(\cos\psi \sin\theta \cos\phi + \sin\psi\sin\phi)\\
-\frac{T}{m}(\sin\psi \sin\theta \cos\phi - \cos\psi\sin\phi) \\
g-\frac{T}{m} (\cos\phi\cos\theta)\\
  \end{matrix} \right]
  $$

  Which can be rewritten as 
  
  $$
    \left[
    \begin{matrix}
      \boldsymbol{x}_1\\
      \boldsymbol{x}_2\\
      \boldsymbol{x}_3\\
      \boldsymbol{x}_4\\
      \boldsymbol{x}_5\\
      \boldsymbol{x}_6\\
    \end{matrix}
  \right] =
  \left[
  \begin{matrix}
    {\boldsymbol x}_3\\
    {\boldsymbol x}_4\\
    {\boldsymbol x}_5\\
    -\frac{\boldsymbol{u}_1}{m}(\cos\boldsymbol{u}_3 \sin\boldsymbol{u}_2 \cos\boldsymbol{u}_1 + \sin\boldsymbol{u}_3\sin\boldsymbol{u}_1)\\
    -\frac{\boldsymbol{u}_1}{m}(\sin\boldsymbol{u}_3 \sin\boldsymbol{u}_2 \cos\boldsymbol{u}_1 - \cos\boldsymbol{u}_3\sin\boldsymbol{u}_1) \\
    g-\frac{\boldsymbol{u}_1}{m} (\cos\boldsymbol{u}_1\cos\boldsymbol{u}_2)\\
  \end{matrix} 
  \right]
  $$
   

#### NMPC
  CasADi is used to implement Nonlinear Model Predictive Control (NMPC).
  The NMPC formulation for single UAV is as follows: 

  $$
  \begin{aligned}
  \min_{x,u} \quad & J=\sum_{k=0}^{T-1} \Big(\Vert x(k)-x^r(k)\Vert_Q+ \Vert u(k)-u^r(k) \Vert_R \Big) + \\
  &\Vert x(T)-x^r(T) \Vert_{Q_f} \\
  s.t. \quad & \dot{x} = f(x,u) &(1)\\
  & x(0) = x_0 &(2)\\
  & \vert \dot{x} \vert \le c_3 , |\dot{u}| \le c_4 &(3) \\
  & \vert \ddot{x}\vert \le c_1, \vert \ddot{u}\vert \le c_2 &(4)\\
  & \Vert x-x^{\mathcal{N}}\Vert \ge c_5 &(5)\\
  \end{aligned}
  $$

  Where $||\cdot||_\chi$ denotes the quadratic form, in which $\chi\in \mathbb{R}^n$ is a n-dim diagonal matrix. Equ. 1 is the non-linear system dynamics differential equation. $x^{\mathcal{N}}$ denots the nearest neighbor of UAV $x$.



