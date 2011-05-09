# Driving command for a car-like vehicle using Ackermann steering.
# $Id$

# Drive at requested speed, acceleration and jerk (the 1st, 2nd and
# 3rd derivatives of position). All are non-negative scalars. 
#
# Speed is defined as the scalar magnitude of the velocity
# vector. Direction (forwards or backwards) is determined by the gear.
#
# Zero acceleration means change speed as quickly as
# possible. Positive acceleration may include deceleration as needed
# to match the desired speed. It represents a desired rate and also a
# limit not to exceed.
#
# Zero jerk means change acceleration as quickly as possible. Positive
# jerk describes the desired rate of acceleration change in both
# directions (positive and negative).
#
float32 speed                   # magnitude of velocity vector (m/s)
float32 acceleration            # desired acceleration (m/s^2)
float32 jerk                    # desired jerk (m/s^3)

# Assumes Ackermann (front-wheel) steering. This angle is the average
# yaw of the two front wheels in the vehicle frame of reference
# (positive left), ignoring their slightly differing angles as if it
# were a tricycle. This is *not* the angle of the steering wheel
# inside the passenger compartment.
#
float32 steering_angle          # steering angle (radians)

Gear gear                       # requested gear (no change if Naught)
PilotBehavior behavior          # requested pilot behavior
