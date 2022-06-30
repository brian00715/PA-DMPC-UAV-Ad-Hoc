import numpy as np
import math
from matplotlib import pyplot as plt

# Define simulation parameters
T = 0.01  # time step
N = 100   # number of time steps
state_dim = 6
control_dim = 4
m = 1.5
g = 9.8   # acceleration due to gravity

# Define initial state and control inputs
init_pos = np.array([0, 0, 0, 0, 0, 0])
# u_ref = np.random.rand(N, control_dim)
u_ref = np.array([30,0,0,0]*N)
x_ref = np.zeros((N + 1, state_dim))

# Define dynamics function
def f(x_, u_):
    dx = x_[3]  # dx
    dy = x_[4]  # dy
    dz = x_[5]  # dz
    ddx = -u_[0] / m * (math.cos(u_[3]) * math.sin(u_[2]) * math.cos(u_[1]) + math.sin(u_[3]) * math.sin(u_[1]))
    ddy = -u_[0] / m * (math.sin(u_[3]) * math.sin(u_[2]) * math.cos(u_[1]) - math.cos(u_[3]) * math.sin(u_[1]))
    ddz = -g + u_[0] / m * (math.cos(u_[1]) * math.cos(u_[2]))
    return np.array([dx, dy, dz, ddx, ddy, ddz])

# Initialize state and control arrays
states = np.zeros((N + 1, state_dim))
controls = np.zeros((N, control_dim))

# Simulate the system
for i in range(N):
    controls[i] = u_ref[i]
    states[i + 1] = states[i] + f(states[i], controls[i]) * T

# Print the simulated states
for i in range(N + 1):
    print(f"Step {i}: State = {states[i]}")
    
# Plot the simulated states
time_steps = np.arange(0, T * (N + 1), T)
plt.figure(figsize=(10, 6))
for dim in range(state_dim):
    plt.plot(time_steps, states[:, dim], label=f'State {dim + 1}')
plt.xlabel('Time')
plt.ylabel('State Value')
plt.title('System State Evolution')
plt.legend()
plt.grid(True)
plt.show()






