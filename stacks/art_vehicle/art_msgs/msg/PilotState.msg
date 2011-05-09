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
CarControl2 target              # current command
CarControl2 plan                # intermediate goal
CarControl2 current             # current status
