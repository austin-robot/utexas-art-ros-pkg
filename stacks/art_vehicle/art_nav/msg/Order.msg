# commander order for the navigator
# $Id$

uint32 N_ORDER_WAYPTS = 5		# number of way-points in order
uint32 N_ORDER_CHKPTS = 2		# number of checkpoints in order

Behavior behavior			# requested behavior
art_map/WayPoint[5] waypt       	# way-point array
art_map/WayPoint[2] chkpt       	# next two goal checkpoints
float32 min_speed			# in meters/sec
float32 max_speed
int32 replan_num
int32 next_uturn         		# Uturn between [1] and [2]
