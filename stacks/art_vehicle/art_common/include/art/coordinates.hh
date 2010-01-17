/*
 *  Description:  System-wide coordinate class definitions
 *
 *  Copyright (C) 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _COORDINATES_HH
#define _COORDINATES_HH

#include <math.h>
#include <vector>

#ifdef USE_PLAYER
#include <libplayercore/player.h>
#include <art/rotate_translate_transform.hh>
#endif

#include <angles/angles.h>
#include <art/conversions.h>

/** @brief Class definitions for system-wide coordinates. */

namespace Coordinates
{
  /** @brief A pose in space (like player_pose3d_t) */
  class Pose3D
  {
  public:
    /** X [m] */
    double x;
    /** Y [m] */
    double y;
    /** Z [m] */
    double z;
    /** roll [rad] */
    double roll;
    /** pitch [rad] */
    double pitch;
    /** yaw [rad] */
    double yaw;

    Pose3D(void):
      x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0)
    {}
    Pose3D(const Pose3D &copy):
      x(copy.x), y(copy.y), z(copy.z),
      roll(copy.roll), pitch(copy.pitch), yaw(copy.yaw)
    {}
    Pose3D(double _x, double _y, double _z,
           double _roll, double _pitch, double _yaw):
      x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw)
    {}

    // some useful operators, define more if needed
    bool operator==(const Pose3D &that) const
    {
      return (this->x == that.x
              && this->y == that.y
              && this->z == that.z
              && this->roll == that.roll
              && this->pitch == that.pitch
              && this->yaw == that.yaw);
    }
    bool operator!=(const Pose3D &that) const
    {
      return (this->x != that.x
              || this->y != that.y
              || this->z != that.z
              || this->roll != that.roll
              || this->pitch != that.pitch
              || this->yaw != that.yaw);
    }
    Pose3D operator+(const Pose3D &that) const
    {
      return Pose3D(this->x + that.x,
                    this->y + that.y,
                    this->z + that.z,
                    angles::normalize_angle(this->roll + that.roll),
                    angles::normalize_angle(this->pitch + that.pitch),
                    angles::normalize_angle(this->yaw + that.yaw));
    }
    Pose3D operator-(const Pose3D &that) const
    {
      return Pose3D(this->x - that.x,
                    this->y - that.y,
                    this->z - that.z,
                    angles::normalize_angle(this->roll - that.roll),
                    angles::normalize_angle(this->pitch - that.pitch),
                    angles::normalize_angle(this->yaw - that.yaw));
    }
  };

  /** @brief 3D position and velocity (like player_position3d_data_t) */
  class Position3D
  {
  public:
    /** position [meters and radians] */
    Pose3D pos;
    /** velocity [meters/second and radians/second] */
    Pose3D vel;
  };
}

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
 * to some map origin, usually the starting position of the run.  They
 * correspond to player_point_2d structures.  We define our own class
 * to emphasize its role in the system and the fact that its origin is
 * relative to the map for this run.
 */
class MapXY				
{
public:
  float x;
  float y;

  // constructors
  MapXY(void): x(0.0), y(0.0) {};
  MapXY(float _x, float _y): x(_x), y(_y) {};
  MapXY(const Coordinates::Pose3D &pose3d): x(pose3d.x), y(pose3d.y) {};
#ifdef USE_PLAYER
  MapXY(const posetype &ppose): x(ppose.x), y(ppose.y) {};
  MapXY(const player_pose2d_t &ppose): x(ppose.px), y(ppose.py) {};
#endif

  void Set(float _x, float _y)
  {
    x = _x;
    y = _y;
  }
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
};

typedef std::vector<MapXY> mapxy_list_t;


/**  MapPose coordinates
 *
 * These are two-dimensional poses relative to the MapXY origin with a
 * yaw angle in radians.  They correspond to player_pose2d_t structures.
 * We define our own class to define useful initializers.
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
  MapPose(const Coordinates::Pose3D &pose3d):
    map(pose3d.x, pose3d.y), yaw(pose3d.yaw) {};
#ifdef USE_PLAYER
  MapPose(const player_pose2d_t &ppose): map(ppose), yaw(ppose.pa) {};
#endif
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

#ifdef USE_PLAYER
  inline float bearing(const player_pose2d_t &from_pose, MapXY to_point)
  {
    return normalize(bearing(MapXY(from_pose), to_point) - from_pose.pa);
  }

  // transform MapXY coordinate to egocentric Polar
  inline Polar MapXY_to_Polar(MapXY point, const player_pose2d_t &origin)
  {

    // TODO: figure out how to use Euclidean::DistanceTo() function,
    // (it is not working here for some reason) 

    //PFB: It's because there are circular header dependencies.
    //euclidean_distance.h needs this header file.
    
    MapPose orgpose = origin;
    MapXY diff=point - orgpose.map;
    return Polar(bearing(origin, point),
		 sqrtf(diff.x*diff.x + diff.y*diff.y));
  }

  // transform egocentric Polar coordinate to MapXY
  inline MapXY Polar_to_MapXY(Polar polar, const player_pose2d_t &origin)
  {
    MapXY retval;
    float map_heading = origin.pa + polar.heading;
    // Don't normalize map_heading, we just need sin() and cos().
    retval.x = (origin.px + cosf(map_heading) * polar.range);
    retval.y = (origin.py + sinf(map_heading) * polar.range);
    return retval;
  }
#endif // USE_PLAYER

  inline float sign(float val)
  {
    return (val >= 0? 1.0f: -1.0f);
  }
};

#endif // _COORDINATES_HH
