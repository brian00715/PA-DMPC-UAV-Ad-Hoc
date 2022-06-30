import os
import json
import matplotlib.pyplot as plt
import numpy as np
import sys

# set latex font
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["font.size"] = 16
plt.rcParams["text.usetex"] = True
# plt.rcParams["text.latex.preamble"] = r"\usepackage{amsmath}"

plot_uav_num = 9
uav_num_step = 1
uav_num_range = [i for i in range(3, plot_uav_num + 1, uav_num_step)]
data_path = os.path.join(sys.path[0], "../data/perf_anaysis/exp7")  # XXX: Remember to change this
cal_diff = False  # difference percentage rate w.r.t. (px4-setpoint)
cal_traf_ene_ratio = False
use_subplot = False
controllers = [
    "nmpc",
    "nmpcg",  # nmpc with global scheduler
    # "px4",
    # "nmpcf2",  # high power
    # "nmpcf1",  # low power
]
metrics = [
    # "simu_time",
    "net_traf_total",
    "cap_resp_time_ave",
    # "cap_resp_time_total",
    # "cap_resp_time_total_weighted",
    # "cap_resp_time_ave_weighted",
    # "induced_power_ave"
    # "parasite_power_ave",
    # "profile_power_ave",
    "instant_power_ave",
    "energy_total",
]

label = {
    "nmpc": "PA-DMPC",
    "px4": "PX4-Setpoint",
    "nmpcf1": "DMPC-LowPower",
    "nmpcf2": "DMPC",  # high power
    "nmpcg": "PA-DMPC With Global Scheduler",
}
title = {
    "net_traf_total": "Total Network Traffic",
    "cap_resp_time_ave": "Average Network Capacity Response Time",
    "instant_power_ave": "Average Instant Power",
    "energy_total": "Total Energy Consumption",
    "traf_ene_ratio": "Traffic-Energy Ratio",
    "traf_diff": "Traffic Rising Rate",
    "ene_diff": "Energy Rising Rate",
    "cpu_usage_ave": "Average CPU Usage",
}
ylabel = {
    "net_traf_total": "Traffic (Mb)",
    "cap_resp_time_ave": "Response Time (s)",
    "instant_power_ave": "Power (w)",
    "energy_total": "Energy (J)",
    "traf_ene_ratio": "Traffic-Energy Ratio",
    "traf_diff": "Traffic Rising Rate (\%)",
    "ene_diff": "Energy Rising Rate (\%)",
    "cpu_usage": "CPU Usage (\%)",
}
color = {
    "nmpc": "#0072bd",
    "px4": "#d95319",
    "nmpcf1": "g",
    "nmpcf2": "#7e2f8e",
    "nmpcg": "#d95319",
}
metric_len = len(metrics)
with_cpu_usage = False

if __name__ == "__main__":
    total_data = {controller: {} for controller in controllers}

    # read and calcu ave
    for filename in os.listdir(data_path):
        if filename.endswith(".json") and filename.startswith("perf"):
            name_split = filename.split("_")
            controller = name_split[3]
            if controller not in controllers:
                continue
            uav_num = int(name_split[4][1])
            file_path = os.path.join(data_path, filename)
            file_obj = open(file_path, "r")
            data_dict = json.load(file_obj)
            total_data[controller][str(uav_num)] = {}
            for metric in metrics:
                metric_data_all_uavs = []
                for i in range(uav_num):
                    metric_data_all_uavs.append(data_dict[f"iris_{i}"][metric])
                metric_ave = np.mean(metric_data_all_uavs)
                metric_sum = np.sum(metric_data_all_uavs)
                total_data[controller][str(uav_num)][metric] = {"ave": metric_ave, "sum": metric_sum}
                if "cpu_usage_ave" in data_dict.keys():
                    total_data[controller][str(uav_num)]["cpu_usage"] = {"ave": data_dict["cpu_usage_ave"]}
                    with_cpu_usage = True
    file_obj = open(data_path + "/other_analysis.json", "w")
    other_ana = {"ene_diff": {}, "traf_diff": {}, "traf_ene_ratio": {}, "cpu_diff": {}}

    # further analysis
    for controller in controllers:
        if cal_diff:  # calculate rising rate w.r.t. baseline (px4-setpoint)
            other_ana["traf_diff"][controller] = {}
            other_ana["ene_diff"][controller] = {}
            other_ana["traf_ene_ratio"][controller] = {}
            for i in uav_num_range:
                if "traf_diff" not in total_data[controller][str(i)]:
                    total_data[controller][str(i)]["traf_diff"] = {}
                total_data[controller][str(i)]["traf_diff"]["ave"] = (
                    (
                        total_data[controller][str(i)]["net_traf_total"]["ave"]
                        - total_data["px4"][str(i)]["net_traf_total"]["ave"]
                    )
                    / total_data["px4"][str(i)]["net_traf_total"]["ave"]
                    * 100
                )

                total_data[controller][str(i)]["traf_diff"]["sum"] = (
                    (
                        total_data[controller][str(i)]["net_traf_total"]["sum"]
                        - total_data["px4"][str(i)]["net_traf_total"]["sum"]
                    )
                    / total_data["px4"][str(i)]["net_traf_total"]["sum"]
                    * 100
                )

                if "ene_diff" not in total_data[controller][str(i)]:
                    total_data[controller][str(i)]["ene_diff"] = {}
                total_data[controller][str(i)]["ene_diff"]["ave"] = (
                    (
                        total_data[controller][str(i)]["energy_total"]["ave"]
                        - total_data["px4"][str(i)]["energy_total"]["ave"]
                    )
                    / total_data["px4"][str(i)]["energy_total"]["ave"]
                    * 100
                )
                total_data[controller][str(i)]["ene_diff"]["sum"] = (
                    (
                        total_data[controller][str(i)]["energy_total"]["sum"]
                        - total_data["px4"][str(i)]["energy_total"]["sum"]
                    )
                    / total_data["px4"][str(i)]["energy_total"]["sum"]
                    * 100
                )
                other_ana["traf_diff"][controller][str(i)] = total_data[controller][str(i)]["traf_diff"]["sum"]
                other_ana["ene_diff"][controller][str(i)] = total_data[controller][str(i)]["ene_diff"]["sum"]

        if cal_traf_ene_ratio:
            energy_all_uavs = [total_data[controller][str(i)]["energy_total"]["ave"] for i in uav_num_range]
            # energy_norm = energy_all_uavs / np.sum(energy_all_uavs)
            energy_norm = (energy_all_uavs - np.min(energy_all_uavs)) / (
                np.max(energy_all_uavs) - np.min(energy_all_uavs)
            ) + 0.001
            traffic_all_uavs = [total_data[controller][str(i)]["net_traf_total"]["ave"] for i in uav_num_range]
            # traffic_norm = traffic_all_uavs / np.sum(traffic_all_uavs)
            traffic_norm = (traffic_all_uavs - np.min(traffic_all_uavs)) / (
                np.max(traffic_all_uavs) - np.min(traffic_all_uavs)
            ) + 0.001
            resp_time_all_uavs = [total_data[controller][str(i)]["cap_resp_time_ave"]["ave"] for i in uav_num_range]
            # resp_time_norm = resp_time_all_uavs / np.sum(resp_time_all_uavs)
            resp_time_norm = (resp_time_all_uavs - np.min(resp_time_all_uavs)) / (
                np.max(resp_time_all_uavs) - np.min(resp_time_all_uavs)
            ) + 0.001
            for i in uav_num_range:
                if "traf_ene_ratio" not in total_data[controller][str(i)]:
                    total_data[controller][str(i)]["traf_ene_ratio"] = {}
                ratio = traffic_norm[i - uav_num_range[0]] / (energy_norm[i - uav_num_range[0]])
                # ratio = traffic_norm[i - uav_num_range[0]] / (
                #     energy_norm[i - uav_num_range[0]] * resp_time_norm[i - uav_num_range[0]]
                # )
                ratio = 10 * np.log10(ratio)
                total_data[controller][str(i)]["traf_ene_ratio"]["ave"] = ratio

            # use sum data
            energy_all_uavs = [total_data[controller][str(i)]["energy_total"]["sum"] for i in uav_num_range]
            # energy_norm = energy_all_uavs / (np.max(energy_all_uavs) - np.min(energy_all_uavs))
            energy_norm = (energy_all_uavs - np.min(energy_all_uavs)) / (
                np.max(energy_all_uavs) - np.min(energy_all_uavs)
            ) + 0.001
            traffic_all_uavs = [total_data[controller][str(i)]["net_traf_total"]["sum"] for i in uav_num_range]
            # traffic_norm = (traffic_all_uavs) / (np.max(traffic_all_uavs) - np.min(traffic_all_uavs))
            traffic_norm = (traffic_all_uavs - np.min(traffic_all_uavs)) / (
                np.max(traffic_all_uavs) - np.min(traffic_all_uavs)
            ) + 0.001
            resp_time_all_uavs = [total_data[controller][str(i)]["cap_resp_time_ave"]["sum"] for i in uav_num_range]
            # resp_time_norm = resp_time_all_uavs / (np.max(resp_time_all_uavs) - np.min(resp_time_all_uavs))
            resp_time_norm = (resp_time_all_uavs - np.min(resp_time_all_uavs)) / (
                np.max(resp_time_all_uavs) - np.min(resp_time_all_uavs)
            ) + 0.001
            for i in uav_num_range:
                if "traf_ene_ratio" not in total_data[controller][str(i)]:
                    total_data[controller][str(i)]["traf_ene_ratio"] = {}
                # ratio = traffic_norm[i - uav_num_range[0]] / (energy_norm[i - uav_num_range[0]])
                ratio = traffic_norm[i - uav_num_range[0]] / (
                    energy_norm[i - uav_num_range[0]] * resp_time_norm[i - uav_num_range[0]]
                )
                ratio = 10 * np.log10(ratio)
                total_data[controller][str(i)]["traf_ene_ratio"]["sum"] = ratio
            other_ana["traf_ene_ratio"][controller] = {
                str(i): total_data[controller][str(i)]["traf_ene_ratio"]["sum"] for i in uav_num_range
            }

    if "nmpcg" in controllers:  # calculate difference percentage between nmpcg and pa-dmpc.
        other_ana["traf_diff"]["nmpcg"] = {}
        other_ana["ene_diff"]["nmpcg"] = {}
        other_ana["traf_ene_ratio"]["nmpcg"] = {}
        other_ana["cpu_diff"]["nmpcg"] = {}
        for i in uav_num_range:
            if "traf_diff" not in total_data["nmpcg"][str(i)]:
                total_data["nmpcg"][str(i)]["traf_diff"] = {}
            if "ene_diff" not in total_data["nmpcg"][str(i)]:
                total_data["nmpcg"][str(i)]["ene_diff"] = {}
            if "cpu_diff" not in total_data["nmpcg"][str(i)]:
                total_data["nmpcg"][str(i)]["cpu_diff"] = {}

            total_data["nmpcg"][str(i)]["traf_diff"]["ave"] = (
                (
                    total_data["nmpcg"][str(i)]["net_traf_total"]["ave"]
                    - total_data["nmpc"][str(i)]["net_traf_total"]["ave"]
                )
                / total_data["nmpc"][str(i)]["net_traf_total"]["ave"]
                * 100
            )
            total_data["nmpcg"][str(i)]["ene_diff"]["ave"] = (
                (total_data["nmpcg"][str(i)]["energy_total"]["ave"] - total_data["nmpc"][str(i)]["energy_total"]["ave"])
                / total_data["nmpc"][str(i)]["energy_total"]["ave"]
                * 100
            )
            total_data["nmpcg"][str(i)]["cpu_diff"]["ave"] = (
                total_data["nmpcg"][str(i)]["cpu_usage"]["ave"] - total_data["nmpc"][str(i)]["cpu_usage"]["ave"]
            )

            total_data["nmpcg"][str(i)]["traf_diff"]["sum"] = (
                (
                    total_data["nmpcg"][str(i)]["net_traf_total"]["sum"]
                    - total_data["nmpc"][str(i)]["net_traf_total"]["sum"]
                )
                / total_data["nmpc"][str(i)]["net_traf_total"]["sum"]
                * 100
            )

            total_data["nmpcg"][str(i)]["ene_diff"]["sum"] = (
                (total_data["nmpcg"][str(i)]["energy_total"]["sum"] - total_data["nmpc"][str(i)]["energy_total"]["sum"])
                / total_data["nmpc"][str(i)]["energy_total"]["sum"]
                * 100
            )
            # total_data["nmpcg"][str(i)]["cpu_usage"]["sum"] - total_data["nmpc"][str(i)]["cpu_usage"]["ave"]

            # use ave data to calculate the difference percenrage for now
            other_ana["traf_diff"]["nmpcg"][str(i)] = total_data["nmpcg"][str(i)]["traf_diff"]["ave"]
            other_ana["ene_diff"]["nmpcg"][str(i)] = total_data["nmpcg"][str(i)]["ene_diff"]["ave"]
            other_ana["cpu_diff"]["nmpcg"][str(i)] = total_data["nmpcg"][str(i)]["cpu_diff"]["ave"]

    json.dump(other_ana, file_obj, indent=4)
    file_obj.close()

    # plot
    metrics.extend(
        [
            # "traf_ene_ratio",
            "traf_diff",
            "ene_diff",
        ]
    )
    if with_cpu_usage:
        metrics.append("cpu_usage")
    x = uav_num_range
    if use_subplot:
        n_row = int(np.sqrt(metric_len)) + 1
        n_col = int(np.ceil(metric_len / n_row))
        fig, plots = plt.subplots(n_row, n_col)
        # according to average
        for metric_idx, metric in enumerate(metrics):
            subplot_x = int(metric_idx / n_col)
            subplot_y = int(metric_idx % n_col)
            for controller in controllers:
                y = [total_data[controller][str(j)][metric]["ave"] for j in x]
                plots[subplot_x, subplot_y].plot(x, y, label=label[controller])
                plots[subplot_x, subplot_y].scatter(x, y)
            plots[subplot_x, subplot_y].set_xlabel("Number of UAVs")
            plots[subplot_x, subplot_y].set_title(f"{metric}")
            plots[subplot_x, subplot_y].grid()
            plots[subplot_x, subplot_y].legend(loc="upper right")
        plt.tight_layout()
        plt.show()

        if 0:  # according to sum
            fig, plots = plt.subplots(n_row, n_col)
            for metric_idx, metric in enumerate(metrics):
                subplot_x = int(metric_idx / n_col)
                subplot_y = int(metric_idx % n_col)
                for controller in controllers:
                    y = [total_data[controller][str(j)][metric]["sum"] for j in x]
                    plots[subplot_x, subplot_y].plot(x, y, label=label[controller])
                plots[subplot_x, subplot_y].set_xlabel("Number of UAVs")
                plots[subplot_x, subplot_y].set_title(f"{metric}")
                plots[subplot_x, subplot_y].legend(loc="upper right")
            plt.tight_layout()
            plt.show()
    else:
        if 1:  # use average data of all uavs
            if 0:
                for metric_idx, metric in enumerate(metrics):
                    plt.figure(figsize=(9, 6))
                    for controller in controllers:
                        y = [total_data[controller][str(j)][metric]["ave"] for j in x]
                        plt.plot(x, y, label=label[controller], color=color[controller], linewidth=2)
                        plt.scatter(x, y, color=color[controller])
                    plt.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))
                    plt.xlabel("Number of UAVs")
                    plt.ylabel(f"{ylabel[metric]}")
                    plt.title(f"{ylabel[metric]}")
                    plt.grid()
                    plt.tight_layout()
                    plt.legend(loc="upper right")
                    plt.show()
            if "nmpcg" in controllers:  # plot the result of global scheduler
                plt.figure(figsize=(9, 6))
                y1 = [total_data["nmpcg"][str(j)]["traf_diff"]["ave"] for j in x]
                y2 = [total_data["nmpcg"][str(j)]["ene_diff"]["ave"] for j in x]
                y3 = [total_data["nmpcg"][str(j)]["cpu_diff"]["ave"] for j in x]
                plt.plot(x, y1, color=color["nmpc"], linewidth=2, label="Traffic")
                plt.scatter(x, y1, color=color["nmpc"])
                plt.plot(x, y2, color=color["nmpcg"], linewidth=2, label="Energy")
                plt.scatter(x, y2, color=color["nmpcg"])
                plt.plot(x, y3, color=color["nmpcf1"], linewidth=2, label="CPU Usage")
                plt.scatter(x, y3, color=color["nmpcf1"])
                plt.ticklabel_format(axis="y")
                plt.xlabel("Number of UAVs")
                plt.ylabel("Percentage (\%)")
                plt.title("PA-DMPC With Global Scheduler vs. PA-DMPC")
                plt.grid()
                plt.tight_layout()
                plt.legend(loc="upper right")
                plt.show()
        if 0:  # use sum data of all uavs
            for metric_idx, metric in enumerate(metrics):
                plt.figure(figsize=(9, 6))
                for controller in controllers:
                    y = [total_data[controller][str(j)][metric]["sum"] for j in x]
                    plt.plot(x, y, label=label[controller], color=color[controller], linewidth=2)
                    plt.scatter(x, y, color=color[controller])
                plt.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))
                plt.xlabel("Number of UAVs")
                plt.ylabel(f"{ylabel[metric]}")
                plt.title(f"{title[metric]}")
                plt.grid()
                plt.tight_layout()
                plt.legend(loc="upper left")
                # plt.savefig(data_path+f"{title[metric]}.eps", bbox_inches="tight")
                plt.show()

            if cal_diff:  # plot rising rate
                plt.figure(figsize=(9, 6))
                for controller in controllers:
                    if controller == "px4":
                        continue
                    y = [total_data[controller][str(j)]["ene_diff"]["sum"] for j in x]
                    plt.plot(
                        x, y, label=label[controller] + "-Energy", color=color[controller], linewidth=2, linestyle="-"
                    )
                    plt.scatter(x, y, color=color[controller])
                    y = [total_data[controller][str(j)]["traf_diff"]["sum"] for j in x]
                    plt.plot(
                        x, y, label=label[controller] + "-Traffic", color=color[controller], linewidth=2, linestyle="--"
                    )
                    plt.scatter(x, y, color=color[controller])
                plt.axhline(y=0, color="black", linestyle="--", linewidth=2)  # 横线
                plt.xlabel("Number of UAVs")
                plt.ylabel("Percentage (\%)")
                # plt.grid()
                plt.tight_layout()
                plt.legend(
                    loc="upper left",
                )
                # plt.savefig(data_path + "/percentage.eps", bbox_inches="tight")
                plt.show()
