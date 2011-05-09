# ART pilot state message

# $Id$

Header  header

DriverState pilot               # pilot state

# current states of individual devices
# (considered CLOSED if device not publishing)
DriverState brake
DriverState imu
DriverState odom
DriverState shifter
DriverState steering
DriverState throttle

# true if pilot preempted for learning speed control
bool preempted

# latest commanded goal and current status
CarDrive target                 # current command
CarDrive plan                   # intermediate goal
CarDrive current                # current status
