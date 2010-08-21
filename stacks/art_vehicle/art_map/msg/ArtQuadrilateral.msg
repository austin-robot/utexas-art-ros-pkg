# ART quadrilateral polygon
# $Id$

# Each of these polygons is a quadrilateral.  The four vertex points
# are ordered relative to the heading of the lane containing it.

geometry_msgs/Polygon poly
int32 bottom_left  = 0
int32 top_left     = 1
int32 top_right    = 2
int32 bottom_right = 3
int32 quad_size = 4

geometry_msgs/Point midpoint # Middle of the polygon

float32 heading         # average of right and left boundary headings
float32 length          # length of the polygon
int32 poly_id           # unique MapLanes ID

bool is_stop            # this poly has a stop waypoint
bool is_transition      # not a lane polygon, no waypoint
bool contains_way       # both start_way and end_way are the contained waypoint

MapID start_way
MapID end_way

LaneMarking left_boundary
LaneMarking right_boundary
