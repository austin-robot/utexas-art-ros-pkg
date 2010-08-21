# ART MapXY coordinates
#
# These are two-dimensional Euclidean coordinates in meters, relative
# to some map origin, usually the starting position of the run.  They
# correspond to player_point_2d structures.  We define our own message
# to emphasize its role in the system and the fact that its origin is
# relative to the map for this run.

# $Id$

float32 x
float32 y
