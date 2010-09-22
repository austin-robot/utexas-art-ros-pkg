# GPS position message
#
# Probably to be replaced by a standard ROS message for Diamondback.

# $Id$

# standard ROS header, includes time stamp
Header header

# Latitude in degrees.  Positive is north of equator, negative is
# south of equator.
float64 latitude

# Longitude in degrees.  Positive is east of prime meridian, negative
# is west of prime meridian.
float64 longitude

# Altitude, in meters.  Positive is above reference (e.g., sea-level),
# and negative is below.
float64 altitude

# UTM WGS84 coordinates, easting [m]
float64 utm_e

# UTM WGS84 coordinates, northing [m]
float64 utm_n

# UTM zone
string zone

# Horizontal dilution of position (HDOP)
float64 hdop

# Vertical dilution of position (VDOP)
float64 vdop

# Horizonal error [m]
float64 err_horz

# Vertical error [m]
float64 err_vert

# Quality of fix 0 = invalid, 1 = GPS fix, 2 = Differential GPS fix
uint16 INVALID_FIX = 0
uint16 GPS_FIX = 1
uint16 DGPS_FIX = 2
uint16 quality

# Number of satellites in view.
uint16 num_sats
