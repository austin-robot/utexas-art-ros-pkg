# Way-point attributes
# $Id$

float64 latitude			# latitude in degrees
float64 longitude			# longitude in degrees
geometry_msgs/Point32 mapxy		# MapXY position
MapID id				# way-point ID
uint16 index    			# parser index of waypoint

# way-point flags
bool is_entry                           # lane or zone exit point
bool is_exit				# lane or zone entry point
bool is_goal				# this is a goal checkpoint
bool is_lane_change			# change lanes after here
bool is_spot				# parking spot
bool is_stop				# stop line here
bool is_perimeter			# zone perimeter point
int32 checkpoint_id			# checkpoint ID or zero
float32 lane_width
