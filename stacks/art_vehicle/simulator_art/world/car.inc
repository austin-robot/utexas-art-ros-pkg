
# Desc: Device definitions for generic robotic car
# Author: Jack O'Quin
# $Id: car.inc 9053 2009-11-17 15:16:52Z joq $


# a generic car, similar to ART's Marvin
define car position
(
  # actual size in meters
  # (need height so it shows up on laser scans)
  size [4.8 2.12 2.0]

  wheelbase 2.33918

  # the car rotates about the rear axle, behind the center of area
  # it seems front of the car is neg - y-offset should be pos
  origin [1.3 0.0 0 0]

  # car-like steering
  drive "car"

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 1200.0 
	
  # this polygon approximates the shape of a pioneer
  polygons 1
  polygon[0].points 8
  polygon[0].point[0] [  0.23  0.05 ]
  polygon[0].point[1] [  0.15  0.15 ]
  polygon[0].point[2] [ -0.15  0.15 ]
  polygon[0].point[3] [ -0.23  0.05 ]
  polygon[0].point[4] [ -0.23 -0.05 ]
  polygon[0].point[5] [ -0.15 -0.15 ]
  polygon[0].point[6] [  0.15 -0.15 ]
  polygon[0].point[7] [  0.23 -0.05 ]
) 
