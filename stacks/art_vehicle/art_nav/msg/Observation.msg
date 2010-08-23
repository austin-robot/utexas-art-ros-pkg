# result returned from a single observer
# TODO: move to a separate art_observers package
# $Id$

bool applicable                 # true if obseravation is applicable
bool clear			# true if clear to go

# optional time and distance to nearest obstacle
# (only reported by some observers)
float32 time
float32 distance
float32 velocity
int32 nobjects  		# number of objects, when relevant
