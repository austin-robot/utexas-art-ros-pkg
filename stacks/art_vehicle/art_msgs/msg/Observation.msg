# result returned from a single observer
# $Id$

# possible observer ID values
int32 Nearest_forward    = 0		# current or closest lane
int32 Nearest_backward   = 1                  
int32 Adjacent_left	 = 2		# adjacent left or right lane
int32 Adjacent_right	 = 3
int32 All_left		 = 4		# all left or right lanes
int32 All_right 	 = 5
int32 Merge_into_nearest = 6            # merge into nearest lane on diff seg
int32 Merge_across_all   = 7            # merge across all lanes on diff seg
int32 Intersection	 = 8		# stop sign intersection precedence
int32 N_Observers        = 9

int32  oid                      # observer ID
string name                     # observer name

bool applicable                 # true if obseravation is applicable
bool clear                      # true if clear to go

# optional time and distance to nearest obstacle
# (only reported by some observers)
float32 time
float32 distance
float32 velocity
int32 nobjects  		# number of objects
