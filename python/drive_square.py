import lcm
import numpy as np
from lcmtypes import mbot_motor_command_t, timestamp_t, reset_odometry_t
import time

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

DRIVE_LENGTH = 4
STOP_LENGTH = 0.5
ROTATE_LENGTH = 1

def current_utime(): return int(time.time() * 1e6)

def turn(dt : float):
    drive = mbot_motor_command_t()
    drive.utime = current_utime()
    drive.trans_v = 0.0
    drive.angular_v = 3.1415/2

    drive_time = timestamp_t()
    drive_time.utime = drive.utime

    lc.publish("MBOT_TIMESYNC", drive_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", drive.encode())
    time.sleep(dt)

    # Stop
    stop = mbot_motor_command_t()
    stop.utime = current_utime()
    stop.trans_v = 0.0
    stop.angular_v = 0.0

    stop_time = timestamp_t()
    stop_time.utime = stop.utime
    lc.publish("MBOT_TIMESYNC", stop_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", stop.encode())
    time.sleep(STOP_LENGTH)

def drive_fwd(dt : float):
    # Drive forward
    drive = mbot_motor_command_t()
    drive.utime = current_utime()
    drive.trans_v = 0.25
    drive.angular_v = 0.0

    drive_time = timestamp_t()
    drive_time.utime = drive.utime

    lc.publish("MBOT_TIMESYNC", drive_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", drive.encode())
    time.sleep(dt)

    # Stop
    stop = mbot_motor_command_t()
    stop.utime = current_utime()
    stop.trans_v = 0.0
    stop.angular_v = 0.0

    stop_time = timestamp_t()
    stop_time.utime = stop.utime
    lc.publish("MBOT_TIMESYNC", stop_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", stop.encode())
    time.sleep(STOP_LENGTH)

reset = reset_odometry_t()
reset.x = 0
reset.y = 0
reset.theta = 0
lc.publish("RESET_ODOMETRY", reset.encode())
time.sleep(2)


drive_fwd(4)
turn(1.0)
drive_fwd(4)
turn(1.0)
drive_fwd(4)
turn(1.0)
drive_fwd(4)
turn(1.0)
# # Drive backward
# drive = mbot_motor_command_t()
# drive.utime = current_utime()
# drive.trans_v = -0.25
# drive.angular_v = 0.0

# drive_time = timestamp_t()
# drive_time.utime = drive.utime

# lc.publish("MBOT_TIMESYNC", drive_time.encode())
# lc.publish("MBOT_MOTOR_COMMAND", drive.encode())
# time.sleep(DRIVE_LENGTH)

# # Stop
# stop = mbot_motor_command_t()
# stop.utime = current_utime()
# stop.trans_v = 0.0
# stop.angular_v = 0.0

# stop_time = timestamp_t()
# stop_time.utime = stop.utime
# lc.publish("MBOT_TIMESYNC", stop_time.encode())
# lc.publish("MBOT_MOTOR_COMMAND", stop.encode())
# time.sleep(STOP_LENGTH)

# # Rotate
# rotate = mbot_motor_command_t()
# rotate.utime = current_utime()
# rotate.trans_v = 0.0
# rotate.angular_v = np.pi

# rotate_time = timestamp_t()
# rotate_time.utime = rotate.utime
# lc.publish("MBOT_TIMESYNC", rotate_time.encode())
# lc.publish("MBOT_MOTOR_COMMAND", rotate.encode())
# time.sleep(ROTATE_LENGTH)

# # Stop
# stop = mbot_motor_command_t()
# stop.utime = current_utime()
# stop.trans_v = 0.0
# stop.angular_v = 0.0

# stop_time = timestamp_t()
# stop_time.utime = stop.utime
# lc.publish("MBOT_TIMESYNC", stop_time.encode())
# lc.publish("MBOT_MOTOR_COMMAND", stop.encode())
# time.sleep(STOP_LENGTH)

# # Rotate
# rotate = mbot_motor_command_t()
# rotate.utime = current_utime()
# rotate.trans_v = 0.0
# rotate.angular_v = -np.pi

# rotate_time = timestamp_t()
# rotate_time.utime = rotate.utime
# lc.publish("MBOT_TIMESYNC", rotate_time.encode())
# lc.publish("MBOT_MOTOR_COMMAND", rotate.encode())
# time.sleep(ROTATE_LENGTH)

# # Stop
# stop = mbot_motor_command_t()
# stop.utime = current_utime()
# stop.trans_v = 0.0
# stop.angular_v = 0.0

# stop_time = timestamp_t()
# stop_time.utime = stop.utime
# lc.publish("MBOT_TIMESYNC", stop_time.encode())
# lc.publish("MBOT_MOTOR_COMMAND", stop.encode())
# time.sleep(STOP_LENGTH)
