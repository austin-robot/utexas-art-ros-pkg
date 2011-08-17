/* -*- mode: C++ -*- */
/*
 *  Description:  Map coordinate class definitions
 *
 *  Copyright (C) 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _COORDINATES_HH
#define _COORDINATES_HH

/**  @file
   
     @brief Class definitions for various map coordinate types.
   
     @author Jack O'Quin
 */

#include <math.h>
#include <vector>

#include <angles/angles.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <art/conversions.h>

/**  GPS latitude and longitude */
class LatLong
{
public:
  double latitude;
  double longitude;

  // void constructor
  LatLong(void)
  {
    latitude = longitude = 0.0;
  }

  // initialization constructor
  LatLong(double lat, double lon)
  {
    latitude = lat;
    longitude = lon;
  }
  bool operator==(const LatLong &that){
    //    double latdiff = this->latitude - that.latitude;
    //    double longdiff = this->longitude - that.longitude;
    //    printf("1:Lat: %3.12f Long %3.12f \n", this->latitude, this->longitude);
    //    printf("2:Lat: %3.12f Long %3.12f \n", that.latitude, that.longitude);
    //    printf("Difference %3.12f, %3.12f\n", latdiff, longdiff);
        bool match = this->latitude == that.latitude &&
      this->longitude == that.longitude;
	//printf("%s\n", match?"Match":"No Match");
      return (match);
  }
};

/**  MapXY coordinates
 *
 * These are two-dimensional Euclidean coordinates in meters, relative
 * to the nearest 10km UTM grid point to the map for this run. East is
 * +x, north is +y.
 *
 * We define our own class to emphasize its role in the system and the
 * fact that its origin is relative to the map for this run.
 */
class MapXY				
{
public:
  float x;
  float y;

  // constructors
  MapXY(void): x(0.0), y(0.0) {};
  MapXY(float _x, float _y): x(_x), y(_y) {};
  MapXY(double _x, double _y): x(_x), y(_y) {};
  MapXY(const geometry_msgs::Point &pt): x(pt.x), y(pt.y) {};
  MapXY(const geometry_msgs::Point32 &pt): x(pt.x), y(pt.y) {};
  MapXY(const MapXY &pt): x(pt.x), y(pt.y) {};
  
  // TODO figure out how to define this without circular references
  //MapXY(const MapPose &pose): x(pose.map.x), y(pose.map.y) {};

  bool operator==(const MapXY &that) const
  {
    return (this->x == that.x && this->y == that.y);
  }
  bool operator!=(const MapXY &that) const
  {
    return (this->x != that.x || this->y != that.y);
  }
  MapXY operator+(const MapXY &that) const
  {
    return MapXY(this->x + that.x, this->y + that.y);
  }
  MapXY operator-(const MapXY &that) const
  {
    return MapXY(this->x - that.x, this->y - that.y);
  }

  void toMsg(geometry_msgs::Point &pt) {
    pt.x = x;
    pt.y = y;
    pt.z = 0;
  }

  void toMsg(geometry_msgs::Point32 &pt) {
    pt.x = x;
    pt.y = y;
    pt.z = 0;
  }
 
};

typedef std::vector<MapXY> mapxy_list_t;


/**  MapPose coordinates
 *
 * These are two-dimensional poses relative to the MapXY origin with a
 * yaw angle in radians.  We define our own class to define useful
 * initializers.
 */
class MapPose				
{
public:
  MapXY map;
  float yaw;

  // constructors
  MapPose(void): map(0.0, 0.0), yaw(0.0) {};
  MapPose(MapXY _map, float _yaw): map(_map), yaw(_yaw) {};
  MapPose(float _x, float _y, float _yaw): map(_x, _y), yaw(_yaw) {};
  MapPose(const geometry_msgs::Pose &pose)
  {
    map = MapXY(pose.position);
    yaw = tf::getYaw(pose.orientation);
  };
};

/** egocentric polar coordinates
 *
 * These are distances from the vehicle's origin (center of rear axle)
 * with a heading in radians relative to the current vehicle heading.
 * In general, they can represent a bearing relative to any MapPose.
 */
class Polar
{
public:
  float heading;			// radians
  float range;				// meters

  // void constructor
  Polar(void)
  {
    heading = range = 0.0;
  }

  // initialization constructor
  Polar(float _heading, float _range)
  {
    heading = _heading;
    range = _range;
  }
};

typedef std::vector<Polar> polar_list_t;

// coordinate transformation functions

namespace Coordinates
{
  // normalize a heading to the range (-M_PI, M_PI]
  // (disallows -M_PI, allowing normalized headings to compare equal)
  inline float normalize(float heading)
  {
    while (heading > M_PI)
      heading -= TWOPI;
    while (heading <= -M_PI)
      heading += TWOPI;
    return heading;
  }


  inline float mod2pi(float angle)
  {
    while (angle < 0.0) 
      angle = angle + TWOPI;
    while (angle >= TWOPI) 
      angle = angle - TWOPI;
    return angle;
  }


  inline float bearing(MapXY from_point, MapXY to_point)
  {
    MapXY offset = to_point - from_point;
    return atan2f(offset.y, offset.x);
  }

  inline float bearing(MapPose from_pose, MapXY to_point)
  {
    return normalize(bearing(from_pose.map, to_point) - from_pose.yaw);
  }

  // transform MapXY coordinate to egocentric Polar
  inline Polar MapXY_to_Polar(MapXY point,
                              const nav_msgs::Odometry &origin)
  {
    // TODO: figure out how to use Euclidean::DistanceTo() function,
    // (it is not working here for some reason) 

    //PFB: It's because there are circular header dependencies.
    //euclidean_distance.h needs this header file.
    
    MapPose orgpose = MapPose(origin.pose.pose);
    MapXY diff = point - orgpose.map;
    return Polar(bearing(orgpose, point),
		 sqrtf(diff.x*diff.x + diff.y*diff.y));
  }

  // transform egocentric Polar coordinate to MapXY
  inline MapXY Polar_to_MapXY(Polar polar, const MapPose &origin)
  {
    // Don't normalize map_heading, we just need sin() and cos().
    float map_heading = origin.yaw + polar.heading;
    MapXY retval;
    retval.x = (origin.map.x + cosf(map_heading) * polar.range);
    retval.y = (origin.map.y + sinf(map_heading) * polar.range);
    return retval;
  }

  inline float sign(float val)
  {
    return (val >= 0? 1.0f: -1.0f);
  }
};

#endif // _COORDINATES_HH
