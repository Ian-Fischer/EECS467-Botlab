import os
import sys

import lcm
import matplotlib.pyplot as plt
import numpy as np

from lcmtypes import pose_xyt_t, odometry_t, lidar_t

sys.path.append("lcmtypes")

if len(sys.argv) < 2:
    sys.stderr.write("usage: plot_step.py <logfile>")
    sys.exit(1)

file = sys.argv[1]

log = lcm.EventLog(file, "r")

odom_data = []
odom_utime = []
slam_data = []
slam_utime = []

lidar_utime = []

for event in log:
    
    if event.channel == "ODOMETRY":
        odom_msg = odometry_t.decode(event.data)
        odom_data.append(odom_msg.x)
        odom_data.append(odom_msg.y)
        odom_data.append(odom_msg.theta)
        odom_utime.append(odom_msg.utime)
    if event.channel == "SLAM_POSE":
        slam_msg = pose_xyt_t.decode(event.data)
        slam_data.append(slam_msg.x)
        slam_data.append(slam_msg.y)
        slam_data.append(slam_msg.theta)
        slam_utime.append(slam_msg.utime)
    if event.channel == "LIDAR": 
        lidar_msg = lidar_t.decode(event.data)
        lidar_utime.append(lidar_msg.utime)

i = np.argmin( np.abs(slam_utime[0] - np.array(odom_utime)))
print(f"argmin {i}")
q = np.min( np.abs(slam_utime[0] - np.array(odom_utime)))
#q = np.min( np.abs(slam_utime[0] - np.array(lidar_utime)))
#print(f"min {q}")

slam_data = np.array(slam_data).reshape(-1, 3)
odom_data = np.array(odom_data).reshape(-1, 3)[i:i+slam_data.shape[0], :]

error = np.sqrt((odom_data[:, 0] - slam_data[:, 0])**2 + (odom_data[:, 1] - slam_data[:, 1])**2)

#error = np.sqrt((odom_data[:, 1] - slam_data[:, 1])**2)

fig = plt.plot(slam_utime, error, "b-", label="odom")
plt.legend()
plt.xlabel("utime")
plt.ylabel("error in position (m)")
plt.show()




