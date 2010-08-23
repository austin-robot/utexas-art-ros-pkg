# Observations from a set of observers
# TODO: move to a separate art_observers package
# $Id$

# list of observer ID values
int32 Nearest_forward    = 0		# current or closest lane
int32 Nearest_backward   = 1                  
int32 Adjacent_left	 = 2		# adjacent lane to left
int32 Adjacent_right	 = 3		# adjacent lane to right
int32 All_left		 = 4		# all lanes to left
int32 All_right 	 = 5		# all lanes to right
int32 Change_lanes	 = 6		# unused
int32 Merge_into_nearest = 7            # merge into nearest lane on diff seg
int32 Merge_across_all   = 8            # merge across all lanes on diff seg
int32 Intersection	 = 9		# intersection precedence
int32 N_Observers        = 10

Header header
nav_msgs/Odometry odom                  # position when observations made
Observation[] obs                       # vector of observations
