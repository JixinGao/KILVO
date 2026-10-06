import matplotlib.pyplot as plt
import numpy as np
import os

file_path = "d_imu.txt"

if not os.path.exists(file_path):
    print(f"Error file '{file_path}' is not exist.")
    exit()

try:
    # load data: time、curr_d、acc_x、acc_y、acc_z
    data = np.loadtxt(file_path)
    if data.ndim != 2 or data.shape[1] != 5:
        print(f"Error, file should have 5 columns, actual shape is {data.shape}")
        exit()
    
except Exception as e:
    print(f"Error reading file: {e}")
    exit()

# check
if len(data) == 0:
    print(f"Error: File contains no valid data")
    exit()


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
plt.subplots_adjust(hspace=0.3)

time = data[:, 0] - data[0, 0]

# plot curr_d (first subplot)
ax1.plot(time, data[:, 1], 'b-', linewidth=1.5, label='curr_d')
ax1.set_title("curr_d", fontsize=14)
ax1.set_ylabel("Value", fontsize=12)
ax1.grid(True, linestyle='--', alpha=0.7)
ax1.legend(loc='best')

# plot acc_x, acc_y, acc_z (second subplot)
ax2.plot(time, data[:, 2], 'r-', linewidth=1.5, label='Acceleration X')
ax2.plot(time, data[:, 3], 'g-', linewidth=1.5, label='Acceleration Y')
ax2.plot(time, data[:, 4], 'b-', linewidth=1.5, label='Acceleration Z')

ax2.set_title("IMU Acceleration Data", fontsize=14)
ax2.set_xlabel("Time (s)", fontsize=12)
ax2.set_ylabel("Acceleration", fontsize=12)
ax2.grid(True, linestyle='--', alpha=0.7)
ax2.legend(loc='best')
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()
