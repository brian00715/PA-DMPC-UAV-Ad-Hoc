import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

csv_file = "/home/simon/wbc_ws/src/dmpc_uav_ad_hoc/data/perf_anaysis/exp3/perf_data_nmpc_u1_iris_0_09-03-210540.csv"
df = pd.read_csv(csv_file)


metrics = [
    "induced_power",
    "profile_power",
    "parasite_power",
    "instant_power",
    "energy_total",
    "net_cap_curr",
    "net_cap_tgt",
    "net_cap_th",
    "total_traffic",
    "cap_resp_time",
]
plot_rows = int(np.sqrt(len(metrics)))
plot_cols = int(np.ceil(len(metrics) / plot_rows))
fig, subplots = plt.subplots(plot_rows, plot_cols)
for i, metric in enumerate(metrics):
    row = i // plot_cols
    col = i % plot_cols
    subplots[row, col].plot(df["time"], df[metric], label=metric)
    subplots[row, col].set_xlabel("Time")
    subplots[row, col].set_ylabel(metric)
    subplots[row, col].legend()

plt.xlabel("Time")
plt.ylabel("Power")
plt.title("Power Data Visualization")
plt.legend()

plt.show()
