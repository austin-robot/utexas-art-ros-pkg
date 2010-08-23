/* -*- mode: C++ -*-
 *
 *  Observer lasers class
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _LASERS_HH_
#define _LASERS_HH_

#include <sensor_msgs/LaserScan.h>

#include <art_map/ArtLanes.h>
#include <art_map/coordinates.h>
#include <art_nav/odometry.h>

/** @brief Observer lasers class. */
class Lasers
{
 public:
  typedef enum {front, rear, velodyne, fusion} lasertype;

  mapxy_list_t front_obstacle_list;    //< coordinates of all the 'obstacles'
                                      //< relative to the front bumber of car
  mapxy_list_t rear_obstacle_list;    //< coordinates of all the 'obstacles'
                                      //< relative to the origin of car
  mapxy_list_t velodyne_obstacle_list;    //< coordinates of all the 'obstacles'
  mapxy_list_t fusion_obstacle_list;
  mapxy_list_t all_obstacle_list;    //< coordinates of all the 'obstacles'

  // laser data state
  // TODO: make this an array of lasers
  bool have_ranges;                   //< true when range data present
  sensor_msgs::LaserScan front_laser_scan; //< current front_laser scan
  Position::Pose3D front_laser_pose;       //< estimated pose of scan
  Position::Pose3D front_laser_odom;
  ros::Time front_laser_time;        //< timestamp of front_laser scan
  sensor_msgs::LaserScan rear_laser_scan; //< current front_laser scan
  Position::Pose3D rear_laser_pose;       //< estimated pose of scan
  Position::Pose3D rear_laser_odom;
  ros::Time rear_laser_time;         //< timestamp of front_laser scan
  sensor_msgs::LaserScan velodyne_laser_scan; //< current front_laser scan
  Position::Pose3D velodyne_laser_pose;	//< estimated pose of scan
  Position::Pose3D velodyne_laser_odom;
  ros::Time velodyne_laser_time;     //< timestamp of front_laser scan
  sensor_msgs::LaserScan fusion_laser_scan; //< current front_laser scan
  Position::Pose3D fusion_laser_pose;       //< estimated pose of scan
  Position::Pose3D fusion_laser_odom;
  ros::Time fusion_laser_time;       //< timestamp of front_laser scan
  float max_range;                   //< maximum scan range
  float front_max_range;             //< maximum scan range
  float rear_max_range;              //< maximum scan range
  float velodyne_max_range;          //< maximum scan range
  float fusion_max_range;            //< maximum scan range


  ros::Time laser_time;              //< timestamp of last laser scan
  int magic;

  /** @brief Constructor */
  Lasers(Odometry *_odometry, int _verbose, int _magic = 0);

  /** @brief Destructor */
  ~Lasers() {}

  /** @brief set configuration variables. */
  int configure();

  /** @brief reset lasers. */
  void reset(void);

  /** @brief Subscribe to laser devices */
  int subscribe_lasers();

  /** @brief Unsubscribe laser devices */
  void unsubscribe_lasers() {}

  bool have_front_laser;
  bool have_rear_laser;
  bool have_velodyne_laser;
  bool have_fusion_laser;

  lasertype last_type;

 private:

  // constructor variables:
  int verbose;				// log level verbosity
  Odometry *odometry;

  // parameters
  double  max_obstacle_dist;

  // lanes interface
  ros::Subscriber front_laser_topic_;    // front laser topic
  ros::Subscriber rear_laser_topic_;     // rear laser topic
  ros::Subscriber velodyne_laser_topic_; // velodyne laser topic
  ros::Subscriber fusion_laser_topic_;   // fusion laser address

  // handle laser scan message
  void laser_message(sensor_msgs::LaserScan *scan,
		     Position::Pose3D *pose,
		     lasertype laser_t);

  //convert laser_scans to MapXY
  //Populate obstacle_list, and obstacle_list_full
  void laserscan_to_obstaclelists(lasertype laser_t);


  ros::Time last_front_time; 
  ros::Time last_rear_time; 
  ros::Time last_velodyne_time; 
  ros::Time last_fusion_time; 


};

#endif // _LASERS_HH_
