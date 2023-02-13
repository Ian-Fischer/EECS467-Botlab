import os
import sys

import lcm
import matplotlib.pyplot as plt
import numpy as np

from lcmtypes import mbot_encoder_t, mbot_motor_command_t, timestamp_t, odometry_t

sys.path.append("lcmtypes")

if len(sys.argv) < 2:
    sys.stderr.write("usage: plot_step.py <logfile>")
    sys.exit(1)

file = sys.argv[1]

log = lcm.EventLog(file, "r")

odom_data = []
odom_init = 0


for event in log:
    if event.channel == "ODOMETRY":
        odom_msg = odometry_t.decode(event.data)
        if odom_init == 0:
            odom_start_time = odom_msg.utime
            odom_init = 1

    if event.channel == "MBOT_TIMESYNC":
        timesync_msg = timestamp_t.decode(event.data)
        timesync_data = np.append(timesync_data, np.array([[
            (timesync_msg.utime)/1.0E6,
        ]]), axis=0)

odom_data = np.vstack(odom_data)

def plot_xy(odom_data):
    xs = odom_data[:, 0]
    ys = odom_data[:, 1]
    fig = plt.plot(xs - 0.1, ys -0.4, "b-", label="odom")
    #plt.plot([0, -1, -1, 0, 0], [0, 0, -1, -1, 0], "r", label="ground truth")
    plt.legend()
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.show()

def plot_xt(odom_data):
    xs = odom_data[:, 0]
    ys = odom_data[:, 1]
    plt.plot(np.arange(xs.size), xs, "b-")
    plt.xlabel("time (seconds)")
    plt.ylabel("X (meters)")
    plt.show()

def plot_theta_t(odom_data):
    thetas = odom_data[:, 2]
    plt.plot(np.arange(thetas.size), thetas, "r-")
    plt.xlabel("time (seconds)")
    plt.ylabel("Heading (radians)")
    plt.show()

#plot_theta_t(odom_data)
plot_xy(odom_data)


