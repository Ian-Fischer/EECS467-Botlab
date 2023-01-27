import sys

import lcm

from lcmtypes import odometry_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")


def odom_handler(channel, data):
    msg = odometry_t.decode(data)
    #print("got odom message: %d %d" % (msg.x, msg.y))
    print(f"got odom message: {msg.x} {msg.y}")


sub = lc.subscribe("ODOMETRY", odom_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    print("lcm exit!")
    sys.exit()
