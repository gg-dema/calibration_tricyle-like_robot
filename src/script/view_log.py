import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
os.chdir('../../data/log/')


def extract_params(file_path) -> dict:
    with open(file_path, 'r') as file:
        lines = file.readlines()
        params = lines[-4:-1]
    _, K_steer, _, K_driving = params[0][:-1].split(' ')
    _, axis, _, offset = params[1][:-1].split(' ')
    _, x, _, y, _, theta = params[2][:-1].split(' ')

    return f"K_steer:{K_steer}\nK_driving:{K_driving}\naxis len:{axis}\nsteer offset:{offset}\nx_robot2sensor:{x}\ny_robot2sensor:{y}\ntheta_robot2sensor{theta}\n"



# Load data
calib_traj_robot = pd.read_csv('calib_traj_robot.csv', sep=';', index_col=None).to_numpy()
calib_traj_sensor = pd.read_csv('calib_traj_sensor.csv', sep=';', index_col=None).to_numpy()
uncalib_traj_robot = pd.read_csv('trajectory_robot.csv', sep=';', index_col=None).to_numpy()
uncalib_traj_sensor = pd.read_csv('trajectory_sensor.csv', sep=';', index_col=None).to_numpy()
gt = pd.read_csv('gt.csv', sep=';', index_col=None).to_numpy()

chi = pd.read_csv('stats.csv', skiprows=1, skipfooter=4, header=0)

# Create figure and subplots with adjusted layout
fig = plt.figure(figsize=(16, 5))
plt.subplots_adjust(right=0.85)  # Make room for text on the right

# First subplot
ax1 = plt.subplot(131)
ax1.plot(gt[:, 0], gt[:, 1], '--', label='ground truth')
ax1.plot(calib_traj_sensor[:, 0], calib_traj_sensor[:, 1], label='calib sensor traj', linewidth=2)
ax1.legend(loc='upper right')
ax1.set_title('Ground Truth vs Calibrated Sensor')
ax1.grid(True)

# Second subplot
ax2 = plt.subplot(132)
ax2.plot(gt[:, 0], gt[:, 1], label='ground truth', color='red', alpha=0.1)
ax2.plot(calib_traj_robot[:, 0], calib_traj_robot[:, 1], label='calib robot traj')
ax2.plot(calib_traj_sensor[:, 0], calib_traj_sensor[:, 1], '--', label='calib sensor traj')
ax2.plot(uncalib_traj_robot[:, 0], uncalib_traj_robot[:, 1], label='uncalib robot traj', color='purple')
ax2.plot(uncalib_traj_sensor[:, 0], uncalib_traj_sensor[:, 1], '--', label='uncalib sensor traj')
ax2.legend()
ax2.set_title('Calibrated vs Uncalibrated Trajectories')
ax2.grid(True)

# Third subplot
ax3 = plt.subplot(133)
ax3.plot(chi)
ax3.set_title('Chi2 over iterations')
ax3.grid(True)

# Add text on the right side
plt.figtext(0.85, 0.5, extract_params('stats.csv'), fontsize=12,
            bbox=dict(facecolor='white', alpha=0.8))

# Improve layout
plt.tight_layout(rect=[0, 0, 0.85, 1])  # Adjust layout while preserving space for text

plt.show()