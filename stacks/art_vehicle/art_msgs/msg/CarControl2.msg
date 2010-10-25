# car acceleration control command
# $Id$

float32 acceleration            # (m/s^2), negative is deceleration
float32 goal_velocity           # (m/s), negative is reverse
float32 steering_angle          # steering angle (radians)

# gear numbers
uint8 Drive = 0
uint8 Park = 1
uint8 Reverse = 2

uint8 gear                      # requested gear
