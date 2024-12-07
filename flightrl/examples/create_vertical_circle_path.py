# Creates a trajectory that is a circle with a radius of 10m and a speed of 1m/s
# The trajectory is saved to a file called dummy_circle_path.csv
# Format: t	p_x	p_y	p_z	q_w	q_x	q_y	q_z	v_x	v_y	v_z	w_x	w_y	w_z	a_lin_x	a_lin_y	a_lin_z	a_rot_x	a_rot_y	a_rot_z	u_1	u_2	u_3	u_4	jerk_x	jerk_y	jerk_z	snap_x	snap_y	snap_z
# Everything except for t and p is 0
# Derivative of position to get velocity
# Time frequency is 0.01s

import numpy as np
import pandas as pd
import csv

# Create a vertical circle trajectory (drone moves in a circle in the xz plane)
radius = 10
speed_limit = 2
time_step = 0.01
time = 0
time_end = 2 * np.pi * radius / speed_limit
path = []
laps = 3

while time < time_end:
    x = radius * np.cos(speed_limit * time / radius)
    y = 0
    z = radius * np.sin(speed_limit * time / radius) + 5
    q_w = 1
    q_x = 0
    q_y = 0
    q_z = 0
    v_x = -speed_limit * np.sin(speed_limit * time / radius)
    v_y = 0
    v_z = speed_limit * np.cos(speed_limit * time / radius)
    w_x = 0
    w_y = 0
    w_z = 0
    a_lin_x = 0
    a_lin_y = 0
    a_lin_z = 0
    a_rot_x = 0
    a_rot_y = 0
    a_rot_z = 0
    u_1 = 0
    u_2 = 0
    u_3 = 0
    u_4 = 0
    jerk_x = 0
    jerk_y = 0
    jerk_z = 0
    snap_x = 0
    snap_y = 0
    snap_z = 0
    path.append([time, x, y, z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, w_x, w_y, w_z, a_lin_x, a_lin_y, a_lin_z, a_rot_x, a_rot_y, a_rot_z, u_1, u_2, u_3, u_4, jerk_x, jerk_y, jerk_z, snap_x, snap_y, snap_z])
    time += time_step

# Save the trajectory to a file
with open("vertical_circle_path_ccw.csv", mode='w') as file:
    writer = csv.writer(file)
    writer.writerow(["t", "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z", "v_x", "v_y", "v_z", "w_x", "w_y", "w_z", "a_lin_x", "a_lin_y", "a_lin_z", "a_rot_x", "a_rot_y", "a_rot_z", "u_1", "u_2", "u_3", "u_4", "jerk_x", "jerk_y", "jerk_z", "snap_x", "snap_y", "snap_z"])
    for row in path:
        writer.writerow(row)

# # Now do the same clockwise
# path = []
# time = 0

# while time < time_end:
#     x = radius * np.cos(-speed_limit * time / radius)
#     y = radius * np.sin(-speed_limit * time / radius)
#     z = 5
#     q_w = 1
#     q_x = 0
#     q_y = 0
#     q_z = 0
#     v_x = speed_limit * np.sin(-speed_limit * time / radius)
#     v_y = -speed_limit * np.cos(-speed_limit * time / radius)
#     v_z = 0
#     w_x = 0
#     w_y = 0
#     w_z = 0
#     a_lin_x = 0
#     a_lin_y = 0
#     a_lin_z = 0
#     a_rot_x = 0
#     a_rot_y = 0
#     a_rot_z = 0
#     u_1 = 0
#     u_2 = 0
#     u_3 = 0
#     u_4 = 0
#     jerk_x = 0
#     jerk_y = 0
#     jerk_z = 0
#     snap_x = 0
#     snap_y = 0
#     snap_z = 0
#     path.append([time, x, y, z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, w_x, w_y, w_z, a_lin_x, a_lin_y, a_lin_z, a_rot_x, a_rot_y, a_rot_z, u_1, u_2, u_3, u_4, jerk_x, jerk_y, jerk_z, snap_x, snap_y, snap_z])
#     time += time_step

# # Save the trajectory to a file
# with open("dummy_circle_path_cw.csv", mode='w') as file:
#     writer = csv.writer(file)
#     writer.writerow(["t", "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z", "v_x", "v_y", "v_z", "w_x", "w_y", "w_z", "a_lin_x", "a_lin_y", "a_lin_z", "a_rot_x", "a_rot_y", "a_rot_z", "u_1", "u_2", "u_3", "u_4", "jerk_x", "jerk_y", "jerk_z", "snap_x", "snap_y", "snap_z"])
#     for row in path:
#         writer.writerow(row)

# Plot the trajectory
import matplotlib.pyplot as plt
import seaborn as sns

sns.set_style("whitegrid")
path = pd.read_csv("vertical_circle_path_ccw.csv")
plt.plot(path["p_x"], path["p_z"])
plt.xlabel("x")
plt.ylabel("z")
plt.title("Vertical Circle Path")
plt.show()

# Wait for plot to close
plt.close()

# # Now plot clockwise

# path = pd.read_csv("dummy_circle_path_cw.csv")
# plt.plot(path["p_x"], path["p_y"])
# plt.xlabel("x")
# plt.ylabel("y")
# plt.title("Dummy Circle Path")
# plt.show