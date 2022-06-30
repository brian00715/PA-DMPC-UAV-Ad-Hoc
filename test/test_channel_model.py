#!/usr/bin/env python
import numpy as np
import sys
from matplotlib import pyplot as plt

if __name__ == "__main__":
    sys.path.append("./src")
    from utils import *

    refer_dist = 1
    freq = 2.4e9
    max_dist = 1000
    scale_factor = 10
    if 0:  # simulate the effect of changing frequency on the channel gain
        freq = np.arange(450e6, 3.8e9, 100e6)
        ax1 = plt.subplot(211)
        ax2 = plt.subplot(212)

        L0 = friis_path_loss(1, freq)
        ax1.plot(freq, L0)
        ax1.set_xlabel("freq")
        ax1.set_ylabel("path loss")
        ax1.set_title("friis path loss")

        for f in freq:
            L0 = friis_path_loss(1, f)
            d = np.arange(10, 100, 10)
            L = log_path_loss(L0, 2, 1, d)

            ax2.plot(d, L, label=f"f={f/1e6}MHz")
            ax2.set_xlabel("distance")
            ax2.set_ylabel("path loss")
        plt.tight_layout()
        plt.legend()
        plt.show()
    if 0:  # simulate the capacity change with distance using log path loss
        L0 = friis_path_loss(refer_dist, freq)
        dist = np.arange(10, max_dist, 10)
        path_loss = log_path_loss(L0, 4, refer_dist, dist)
        gain = 1 / path_loss
        C = calcu_capacity(20e6, dbm2watt(46), gain, dbm2watt(20)) / 1e6

        ax1 = plt.subplot(211)
        ax2 = plt.subplot(212)
        ax1.plot(dist, gain)
        ax1.set_xlabel("distance")
        ax1.set_ylabel("gain")
        ax2.plot(dist, C)
        ax2.set_xlabel("distance")
        ax2.set_ylabel("capacity (mbps)")
        ax2.set_title("log path loss")
        plt.show()
    if 0:  # simulate the effect of path loss exponent on the chennel gain
        L0 = friis_path_loss(refer_dist, freq)
        dist = np.arange(10, 10000, 1)
        for i in range(1, 5):
            path_loss = log_path_loss(L0, i, refer_dist, dist)
            gain = 1 / path_loss
            plt.plot(dist, gain, label=f"n={i}")
        plt.legend()
        plt.show()
    if 0:  # simulate the capacity change with distance using pure friis path loss
        dist = np.arange(10, max_dist, 10)
        path_loss = friis_path_loss(dist, freq)
        gain = 1 / path_loss
        C = calcu_capacity(20e6, dbm2watt(46), gain, dbm2watt(20)) / 1e6

        ax1 = plt.subplot(111)
        ax1.plot(dist, C)
        ax1.set_xlabel("distance")
        ax1.set_ylabel("C")
        plt.show()
    if 1:  # simulate the capacity change with distance using two-ray ground reflection model
        dist = np.arange(10, 100, 10)
        path_loss = two_ray_ground_reflection(dist, 1, 10, 1, 1)
        gain = 1 / path_loss
        C = calcu_capacity(2.4e9, dbm2watt(20), gain, dbm2watt(25)) / 1e6
        ax1 = plt.subplot(111)
        ax1.plot(dist, C)
        ax1.set_xlabel("distance")
        ax1.set_xticks(np.arange(0, 100, 5))
        ax1.set_ylabel("C")
        plt.show()
