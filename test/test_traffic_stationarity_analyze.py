import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from statsmodels.tsa import stattools
from statsmodels.graphics.tsaplots import plot_acf
import time

# ROS初始化
rospy.init_node("signal_analysis_node", anonymous=True)

# 用于存储接收到的信号数据
received_signal = []


# 回调函数，用于处理接收到的信号数据
def signal_callback(data):
    global received_signal
    received_signal.append(data.data[0])


# 订阅ROS话题
rospy.Subscriber("/crowd/test", Float32MultiArray, signal_callback)

# 等待接收到足够的数据
print("collecting data...")
while len(received_signal) < 60:
    continue
print("data collected!")

# 将接收到的信号数据转换为NumPy数组
signal_array = np.array(received_signal)

# 绘制信号图
plt.figure(figsize=(10, 4))
plt.plot(signal_array)
plt.title("Received Signal")
plt.xlabel("Time")
plt.ylabel("Value")
plt.show()

# 计算自相关函数并绘制ACF图
plot_acf(signal_array, lags=50)
acf_result = stattools.acf(signal_array, nlags=50)
plt.title("Autocorrelation Function (ACF)")
plt.xlabel("Lag") 
plt.ylabel("Autocorrelation")
plt.show()

# 进行单位根检验
adf_result = stattools.adfuller(signal_array,)
print("ADF Statistic:", adf_result[0])
print("p-value:", adf_result[1])
print("Critical Values:", adf_result[4])

# 根据p-value判断平稳性
if adf_result[1] < 0.05:
    print("The signal is likely stationary.")
else:
    print("The signal is likely non-stationary.")
