import os
import sys

import lcm
import matplotlib.pyplot as plt
import numpy as np

from lcmtypes import mbot_encoder_t, mbot_motor_command_t, timestamp_t, odometry_t


def is_between(a, b, c):
    return a <= c <= b or b <= c <= a


sys.path.append("lcmtypes")

WHEEL_BASE = 0.15
WHEEL_DIAMETER = 0.084
GEAR_RATIO = 78
ENCODER_RES = 20
enc2meters = WHEEL_DIAMETER * np.pi / (GEAR_RATIO * ENCODER_RES)

if len(sys.argv) < 2:
    sys.stderr.write("usage: plot_step.py <logfile>")
    sys.exit(1)

file = sys.argv[1]
log = lcm.EventLog(file, "r")

odom_data = []
odom_init = 0

encoder_data = np.empty((0, 5), dtype=int)
encoder_init = 0

command_data = np.empty((0, 3), dtype=float)
command_init = 0

timesync_data = np.empty((0, 1), dtype=int)

for event in log:
    if event.channel == "ODOMETRY":
        odom_msg = odometry_t.decode(event.data)
        if odom_init == 0:
            odom_start_time = odom_msg.utime
            odom_init = 1
        
        odom_data.append(np.array([odom_msg.x,
                          odom_msg.y,
                          odom_msg.theta,
                          odom_msg.utime - odom_start_time]))
            
    if event.channel == "MBOT_ENCODERS":
        encoder_msg = mbot_encoder_t.decode(event.data)
        if encoder_init == 0:
            enc_start_utime = encoder_msg.utime
            print("enc_start_utime: {}".format(enc_start_utime))
            encoder_init = 1
        encoder_data = np.append(encoder_data, np.array([[
            (encoder_msg.utime - enc_start_utime)/1.0E6,
            encoder_msg.leftticks,
            encoder_msg.rightticks,
            encoder_msg.left_delta,
            encoder_msg.right_delta
        ]]), axis=0)

    if event.channel == "MBOT_MOTOR_COMMAND":
        command_msg = mbot_motor_command_t.decode(event.data)
        if command_init == 0:
            cmd_start_utime = command_msg.utime
            print("cmd_start_utime: {}".format(cmd_start_utime))
            command_init = 1
        command_data = np.append(command_data, np.array([[
            (command_msg.utime - cmd_start_utime)/1.0E6,
            command_msg.trans_v,
            command_msg.angular_v
        ]]), axis=0)

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

plot_theta_t(odom_data)


