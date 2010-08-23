/*
 *  Lasers class implementation
 *
 *  Copyright (C) 2007, 2010 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <iostream>

#include <art/error.h>
#include <art_map/rotate_translate_transform.h>
#include <art_common/ArtVehicle.h>
#include <art_nav/lasers.h>
#include <art_nav/odometry.h>

// Constructor
Lasers::Lasers(Odometry *_odometry, int _verbose, int _magic)
{
  verbose = _verbose;
  odometry = _odometry;
  magic = _magic;

  have_ranges = false;
  max_range = 0.0;
  front_max_range = 0.0;
  rear_max_range = 0.0;
  velodyne_max_range = 0.0;
  fusion_max_range = 0.0;

  have_front_laser = false;
  have_rear_laser = false;
  have_velodyne_laser = false;
  have_fusion_laser = false;

  last_type=front;

  reset();
}

int Lasers::configure()
{
  // use private node handle to get parameters
  ros::NodeHandle nh("~");

  // distance at which we start paying attention to an obstacle
  nh.param("max_obstacle_dist", max_obstacle_dist, 100.0);
  ROS_INFO_STREAM("maximum obstacle distance considered is "
                  << max_obstacle_dist << " meters");
  return 0;
}

/** handle laser scan message
 *
 *  Called from the match_lasers() method when new laser data arrive.
 */
void Lasers::laser_message(sensor_msgs::LaserScan *scan,
                           Position::Pose3D *pose,
                           lasertype laser_t)
{
  if (laser_t==front)
    {
      front_laser_time = scan->header.stamp;

      if (front_laser_time > last_front_time)
	{
	  ROS_INFO_STREAM("[" << magic << "] new front laser at "
                          << front_laser_time);
	  last_type = front;

	  front_laser_scan = *scan;
	  front_max_range = fminf(scan->range_max, max_obstacle_dist);
	  max_range = fmaxf(max_range, front_max_range);
	  have_ranges = true;
	  
	  last_front_time = front_laser_time;
	  laser_time = front_laser_time;

	  front_laser_pose = *pose;

	  laserscan_to_obstaclelists(front);
	}

#if 0
      ART_MSG(5, "front laser scan: %u ranges, max_range %.3f, time %.6f",
              front_laser_scan.ranges_count,
              front_laser_scan.max_range,
              front_laser_time);
      ART_MSG(5, "front_laser scan: angles (%.3f, %.3f), resolution %.3f rad",
              front_laser_scan.min_angle,
              front_laser_scan.max_angle,
              front_laser_scan.resolution);
      ART_MSG(5, "front_laser scan pose: (%.3f, %.3f, %.3f)",
              front_laser_pose.px, front_laser_pose.py, front_laser_pose.pa);
#endif
    }
}

// reset lasers class
void Lasers::reset(void)
{
  // TODO: figure out when this needs to happen and what to do
}

// subscribe to all configured lasers
int Lasers::subscribe_lasers()
{
  return 0;
}

void Lasers::laserscan_to_obstaclelists(lasertype laser_t)
{
  if (laser_t == front)
    {
      front_obstacle_list.clear();

      for (uint32_t i = 0; i < front_laser_scan.ranges.size(); i++)
        {
          //front_laser_scan.ranges[i] = 1;
          if (front_laser_scan.ranges[i] < front_max_range)
            {
#if 0 // TODO:              
              float heading = (front_laser_scan.min_angle
                               + DTOR(i) + front_laser_pose.pa);
              float cx = (front_laser_pose.px
                          + cosf(heading) * front_laser_scan.ranges[i]);
              float cy = (front_laser_pose.py
                          + sinf(heading) * front_laser_scan.ranges[i]);
              front_obstacle_list.push_back(MapXY(cx, cy));
#endif
            }
        }
    }

  all_obstacle_list.resize(front_obstacle_list.size()
                           +rear_obstacle_list.size()
                           +velodyne_obstacle_list.size()
                           +fusion_obstacle_list.size());

  for (uint i=0;i<front_obstacle_list.size();i++)
    all_obstacle_list[i]=front_obstacle_list[i];
  for (uint i=0;i<rear_obstacle_list.size();i++)
    all_obstacle_list[i+front_obstacle_list.size()]=rear_obstacle_list[i];
  for (uint i=0;i<velodyne_obstacle_list.size();i++)
    all_obstacle_list[i+rear_obstacle_list.size()
                      +front_obstacle_list.size()]=velodyne_obstacle_list[i];
  for (uint i=0;i<fusion_obstacle_list.size();i++)
    all_obstacle_list[i+rear_obstacle_list.size()
                      +front_obstacle_list.size()
                      +velodyne_obstacle_list.size()] = fusion_obstacle_list[i];
}
