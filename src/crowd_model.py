"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-05-12 17:18:22
 # @ Modified time: 2022-05-12 17:25:28
 # @ Description: Mobility model of crowds
 """


import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm, colors
import matplotlib as mpl


class GaussMarkovMobilityModel:
    """refer to:
    - Yapu Li, etc.,Air-to-ground 3D channel modeling for UAV based on Gauss-Markov mobile model,
    AEU - International Journal of Electronics and Communications, Volume 114, 2020, 152995,
    https://doi.org/10.1016/j.aeue.2019.152995.
    - B. Liang and Z. J. Haas, "Predictive distance-based mobility management for multidimensional PCS networks,"
    in IEEE/ACM Transactions on Networking, vol. 11, no. 5, pp. 718-732, Oct. 2003,
    doi: 10.1109/TNET.2003.815301.
    """

    def __init__(self, init_pos, init_speed, init_dir, ave_speed, ave_dir, speed_alpha, dir_alpha) -> None:
        self.speed_alpha = speed_alpha
        self.dir_alpha = dir_alpha
        self.pose = init_pos
        self.speed = init_speed
        self.dir = init_dir  # theta in [0,2pi], using polar coordinate
        self.ave_speed = ave_speed
        self.ave_dir = ave_dir

    def step(self):
        # x_t+1 = alpha * x_t + (1-alpha) * ave_x + N(0,1) * sqrt(1-alpha^2)
        self.speed = (
            self.speed * self.speed_alpha
            + (1 - self.speed_alpha) * self.ave_speed
            + random.gauss(0, self.speed) * np.sqrt(1 - self.speed_alpha**2) * self.ave_speed
        )
        # print(f"ave_speed:{self.ave_speed},speed:{self.speed}")
        self.speed = max(self.speed, 0)
        self.dir = (
            self.dir * self.dir_alpha
            + (1 - self.dir_alpha) * self.ave_dir
            + random.gauss(0, np.radians(360 / 3))  # maually set the standard deviation
            * np.sqrt(1 - self.dir_alpha**2)
        )
        self.pose = [self.pose[0] + self.speed * np.cos(self.dir), self.pose[1] + self.speed * np.sin(self.dir)]
        return self.pose


class CrowdModel(GaussMarkovMobilityModel):
    def __init__(
        self,
        init_pos=[0, 0],
        init_speed=0.5,
        init_dir=np.radians(0),
        ave_speed=0.5,
        ave_dir=np.radians(0),
        speed_alpha=0.1,
        dir_alpha=0.1,
        net_traf_ave=10,
        net_traf_std=1,
        net_traf_mutation_prob=0.2,
        net_traf_mutation_ar=np.concatenate([np.zeros(2), [5, 10, 25, 50]]),
        net_traf_min=0.1,
        net_traf_max=50,
    ) -> None:
        """Crowd model with Gauss Markov mobility model and net traffic model
        Args:
            init_pos (list, optional): [x,y]. Defaults to [0, 0].
            init_speed (float, optional): [m/s]. Defaults to 0.5.
            init_dir (float, optional): [rad]. Defaults to np.radians(0).
            ave_speed (float, optional): [m/s]. Defaults to 0.5.
            ave_dir (float, optional): [rad]. Defaults to np.radians(0).
            speed_alpha (float, optional): [0,1]. Can be thought of as the inertia of the system. Defaults to 0.1.
            dir_alpha (float, optional): [0,1]. Defaults to 0.1.
            net_traf_ave (int, optional): [Mbps]. Defaults to 10.
            net_traf_std (int, optional): [Mbps]. Defaults to 1.
            net_traf_mutation_prob (float, optional): [0,1]. Network capability mutation probability. Defaults to 0.2.
            net_traf_mutation_ar (list, optional): [Mbps]. the traffic that can be mutated. Defaults to np.concatenate([np.zeros(2), [5, 10, 25, 50]]).
        """
        super().__init__(init_pos, init_speed, init_dir, ave_speed, ave_dir, speed_alpha, dir_alpha)
        self.net_traf = 0  # net traffic (Mbps)
        self.net_traf_min = net_traf_min
        self.net_traf_max = net_traf_max
        self.net_traf_ave = net_traf_ave
        self.net_traf_std = net_traf_std
        self.net_traf_mutation = 0
        self.net_traf_mutation_prob = net_traf_mutation_prob
        self.net_traf_mutation_ar = net_traf_mutation_ar

    def step(self):
        """
        Return:
            pose: [x,y]
            net_traf: Mbps
        """
        super().step()
        self.net_traf = np.clip(
            np.random.normal(self.net_traf_ave, self.net_traf_std), self.net_traf_min, self.net_traf_max
        ) + (np.random.choice(self.net_traf_mutation_ar) if np.random.random() < self.net_traf_mutation_prob else 0)
        return self.pose, self.net_traf

    def step_traffic(self):
        """sperated stepping for different update frequency"""
        if random.random() < self.net_traf_mutation_prob:
            self.net_traf_mutation = np.random.choice(self.net_traf_mutation_ar)
            # print(f"traffic mutated, value:{self.net_traf_mutation}")
        self.net_traf = (
            np.clip(np.random.normal(self.net_traf_ave, self.net_traf_std), self.net_traf_min, self.net_traf_max)
            + self.net_traf_mutation
        )

        return self.net_traf

    def step_mobility(self):
        super().step()
        return self.pose


if __name__ == "__main__":
    # gauss markov parameters
    num_crowds = 10
    alpha = 0.8
    ave_speed = 0.5
    ave_dir = np.radians(0)

    # net traffic parameters
    net_traf_ave = 10
    net_traf_std = 3

    simu_steps = 100

    # Initialize crowds
    crowds = []
    for _ in range(num_crowds):
        init_pos = [0, 0]
        init_speed = random.uniform(0, ave_speed)
        crowds.append(CrowdModel(init_pos, init_speed, alpha, net_traf_ave, net_traf_std, 0.2))

    # plot
    color_norm = colors.Normalize(vmin=0, vmax=num_crowds)
    for t in range(simu_steps):  # 100 time steps
        for i, crowd in enumerate(crowds):
            pos, traffic = crowd.step(ave_speed, ave_dir)
            color = mpl.colormaps["hsv"](color_norm(i))
            alpha = t / simu_steps
            size = traffic * 10 / (crowd.net_traf_max - crowd.net_traf_min)
            plt.scatter(pos[0], pos[1], marker="o", alpha=alpha, color=color, s=size)
    plt.title("Random Mobility Model Trajectories")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.show()
