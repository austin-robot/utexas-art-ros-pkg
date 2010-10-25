/*
 *  Copyright (C) 2006 Austin Robot Technology
 *  Copyright (C) 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file
 
     @brief vehicle position and velocity monitoring

ROS odometry driver for the Applanix Position and Orientation System
for Land Vehicles (POS-LV).

The odometry driver publishes its best estimate of the vehicle's
location, velocity and yaw rate.  It collects data from the Applanix
Position and Orientation System for Land Vehicles (POS-LV) which
provides differential GPS and accurate inertial navigation.

@par Publishes

- \b odom (nav_msgs/Odometry): Current estimate of vehicle position
and velocity in three dimensions, including roll, pitch, and yaw.  All
data are in the \b /odom frame of reference.

- \b imu (sensor_msgs/Imu): Current inertial measurements from the Applanix.

- \b gps (art_msgs/GpsInfo): Current GPS status from the Applanix.

- \b tf: broadcast transform from \b vehicle frame to \b odom frame.

@par Subscribes

- \b shifter/state: current transmission gear

 
\author Jack O'Quin, Patrick Beeson

*/

#include <ros/ros.h>
#include <angles/angles.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <art_msgs/GpsInfo.h>
#include <tf/transform_broadcaster.h>

#include <art_msgs/Shifter.h>
#include <art/frames.h>
#include <art/UTM.h>

#include "applanix.h"
#include "position.h"

#define NODE "applanix"

namespace
{
  // transmission gear information
  ros::Subscriber shifter_sub_;         // shifter position ROS topic
  uint8_t shifter_gear_ = art_msgs::Shifter::Drive;

  // command parameters
  int qDepth = 1;                       // ROS topic queue size

  // class for generating vehicle-relative frame IDs
  ArtFrames::VehicleRelative vr_;

  // GPS information
  applanix_data_t adata;		// saved applanix data packets
  DevApplanix *applanix_;		// Applanix device interface
};


/** Global to local coordinate transform. 
 *
 * Translate current pose from UTM meters (northing, easting) to local
 * coordinates by subtracting initial pose.
 *
 * returns: true if initial pose, false otherwise
 */
bool GlobalToLocal(Position::Pose3D *current) 
{
  // original pose when started (global coordinates)
  static Position::Pose3D map_origin;
  static bool first_pose_received = false;

  ROS_DEBUG("Global data (%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f)",
            current->x, current->y, current->z,
            current->roll, current->pitch, current->yaw);

  bool initial_pose = !first_pose_received;

  if (initial_pose)
    {
      // Initial conditions. Compute map origin from starting point
      // using a UTM grid so we can offset future data points from
      // there. If the driver restarts within the same region, it will
      // pick the same origin.

      map_origin = *current;
      map_origin.x = rint(map_origin.x/UTM::grid_size) * UTM::grid_size;
      map_origin.y = rint(map_origin.y/UTM::grid_size) * UTM::grid_size;
      map_origin.z = 0.0;               // leave elevations alone

      first_pose_received = true;

      ROS_INFO("INITIAL data (%.3f, %.3f, %.3f), map origin (%.3f, %.3f, %.3f)",
               current->x, current->y, current->z,
               map_origin.x, map_origin.y, map_origin.z);
    }


  // General case.  See how far we have gone & the angle between
  // the coordinate systems and translate between global (X, Y)
  // and local (x, y)

  // We do not subtract one entire Pose3D from the other to avoid
  // changing the roll, pitch and yaw fields (and also to forgo
  // unnecessary arithmetic).
  current->x -= map_origin.x;
  current->y -= map_origin.y;
  current->z -= map_origin.z;

  ROS_DEBUG("Local data  (%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f)",
            current->x, current->y, current->z,
            current->roll, current->pitch, current->yaw);


  return initial_pose;
}

/** Get new Applanix data.
 *
 *  \returns true if new data available.
 *
 *  Updates *adata if possible.
 */
bool getNewData(applanix_data_t *adata)
{
  static ros::Time last_time;

  // read and unpack first packet
  int rc = applanix_->get_packet(adata);
  if (rc != 0)				// none available?
    {
      ROS_DEBUG("no packet found");
      return false;
    }

  // Get any additional packets already queued.  It is OK if there are
  // none, but we want to return the latest available information.
  do
    {
      ROS_DEBUG_STREAM("got packet, time: " << adata->time);
      rc = applanix_->get_packet(adata);
    }
  while (rc == 0);

  // see if a new navigation solution is available
  if (adata->time == last_time)
    return false;
  // see if device is returning valid data yet
  if (adata->grp1.alignment == ApplStatusInvalid)
    {
      ROS_WARN("Applanix alignment invalid");
      return false;			// no valid solution yet
    }

#if 1
  if (adata->time == ros::Time())
    {
      ROS_WARN_STREAM("invalid packet time: " << adata->time);
      return false;
    }
#endif

  last_time = adata->time;              // remember time of last update
  return true;
}

/** Publish GpsInfo message. */
void publishGPS(const applanix_data_t &adata, double utm_e, double utm_n,
                const char* zone, ros::Publisher *gps_pub)
{
  art_msgs::GpsInfo gpsi;

  gpsi.header.stamp = adata.time;
  gpsi.header.frame_id = vr_.getFrame(ArtFrames::odom);
  gpsi.latitude   = adata.grp1.lat;
  gpsi.longitude  = adata.grp1.lon;
  gpsi.altitude   = adata.grp1.alt;
  gpsi.utm_e  = utm_e;
  gpsi.utm_n  = utm_n;
  gpsi.zone = std::string(zone);

  switch (adata.grp1.alignment)
    {
    case ApplStatusFull:
      gpsi.quality = art_msgs::GpsInfo::DGPS_FIX;
      break;
    case ApplStatusFine:
      gpsi.quality = art_msgs::GpsInfo::GPS_FIX;
      break;
    default:
      gpsi.quality = art_msgs::GpsInfo::INVALID_FIX;
    }

  /// \todo unpack Applanix grp2 and grp3 data to complete other fields

  gps_pub->publish(gpsi);
}

/** Get any new odometry data available
 *
 *  Updates odometry information, which will be published if valid
 *  packets available and this is not the first call.
 *
 *  Publishes GPS information, if new data received.
 *
 *  \param odom_pos3d -> updated position, if new data
 *  \param odom_time  -> updated time when new data received
 *  \param gps_pub    -> GpsInfo ROS topic publisher
 *
 *  \returns true if odometry should be published
 */
bool getOdom(Position::Position3D *odom_pos3d, ros::Time *odom_time,
             ros::Publisher *gps_pub)
{
  if (!getNewData(&adata))
    {
      ROS_DEBUG("no data this cycle");
      return false;                     // nothing to publish
    }

  // remember when the new data arrived
  *odom_time = adata.time;

  // Convert latitude and longitude (spherical coordinates) to
  // Universal Transverse Mercator (Cartesian).
  double utm_e, utm_n;			// easting, northing (in meters)
  char zone[20];
  UTM::LLtoUTM(adata.grp1.lat, adata.grp1.lon, utm_n, utm_e, zone);

  // publish GPS information topic
  publishGPS(adata, utm_e, utm_n, zone, gps_pub);

  using namespace angles;
    
  // fill in Position3D position
  odom_pos3d->pos.x = utm_e;
  odom_pos3d->pos.y = utm_n;
  odom_pos3d->pos.z = adata.grp1.alt;

  // Translate heading.  GPS heading is like a compass, zero degrees
  // is North, East is 90, West is 270. The robot heading is zero for
  // East (positive X direction), and pi/2 radians for North (positive
  // Y).
  odom_pos3d->pos.roll =  from_degrees(adata.grp1.roll);
  odom_pos3d->pos.pitch = from_degrees(-adata.grp1.pitch);
  odom_pos3d->pos.yaw = normalize_angle(from_degrees(90 - adata.grp1.heading));

  // Convert the current global coordinates to local values relative
  // to our initial position.
  if (GlobalToLocal(&odom_pos3d->pos))  // initial position?
    return false;                       // do not publish

  // Invert speed if the vehicle is going in reverse.
  double speed = adata.grp1.speed;	// in meters/second
  if (shifter_gear_ == art_msgs::Shifter::Reverse)
    speed = -speed;			// handle reverse movement

  // Fill in Position3D velocity in /vehicle frame.  Y velocity should
  // normally be zero (unless skidding sideways).
  odom_pos3d->vel.x = speed;
  odom_pos3d->vel.y = 0.0;              // use adata.grp4.vel_y somehow?
  odom_pos3d->vel.z = -adata.grp1.vel_down;

  odom_pos3d->vel.roll =  from_degrees(adata.grp1.arate_lon);
  odom_pos3d->vel.pitch = from_degrees(-adata.grp1.arate_trans);
  odom_pos3d->vel.yaw =   from_degrees(-adata.grp1.arate_down);

  return true;				// need to publish
}  

/** subscriber callback for current shifter position data */
void getShifter(const art_msgs::Shifter::ConstPtr &shifterIn)
{
  if (shifter_gear_ != shifterIn->gear)
    ROS_INFO("Gear changed from %u to %u",
             shifter_gear_, shifterIn->gear);
  shifter_gear_ = shifterIn->gear;
}

/** Publish the current 3D Pose and accelerations */
void putPose(const Position::Position3D *odom_pos3d,
             const ros::Time *odom_time,
             tf::TransformBroadcaster *odom_broad,
             ros::Publisher *odom_pub,
             ros::Publisher *imu_pub)
{
  // translate roll, pitch and yaw into a Quaternion
  tf::Quaternion q;
  q.setRPY(odom_pos3d->pos.roll, odom_pos3d->pos.pitch, odom_pos3d->pos.yaw);
  geometry_msgs::Quaternion odom_quat;
  tf::quaternionTFToMsg(q, odom_quat);

  // broadcast Transform from /earth to /vehicle
  geometry_msgs::TransformStamped earth_tf;
  earth_tf.header.stamp = *odom_time;
  earth_tf.header.frame_id = vr_.getFrame(ArtFrames::earth);
  earth_tf.child_frame_id = vr_.getFrame(ArtFrames::vehicle);
  earth_tf.transform.translation.x = odom_pos3d->pos.x;
  earth_tf.transform.translation.y = odom_pos3d->pos.y;
  earth_tf.transform.translation.z = odom_pos3d->pos.z;
  earth_tf.transform.rotation = odom_quat;
  odom_broad->sendTransform(earth_tf);

  // broadcast Transform from /odom to /earth with same X, Y position
  // and orientation, but same Z coordinate as /vehicle
  geometry_msgs::TransformStamped odom_tf;
  odom_tf.header.stamp = *odom_time;
  odom_tf.header.frame_id = vr_.getFrame(ArtFrames::odom);
  odom_tf.child_frame_id = vr_.getFrame(ArtFrames::earth);
  odom_tf.transform.translation.z = -odom_pos3d->pos.z;
  odom_tf.transform.rotation.w = 1.0;
  odom_broad->sendTransform(odom_tf);

  // publish the Odometry message
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = *odom_time;
  odom_msg.header.frame_id = vr_.getFrame(ArtFrames::odom);
  odom_msg.pose.pose.position.x = odom_pos3d->pos.x;
  odom_msg.pose.pose.position.y = odom_pos3d->pos.y;
  odom_msg.pose.pose.position.z = odom_pos3d->pos.z;
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.child_frame_id = vr_.getFrame(ArtFrames::vehicle);

  // Twist is relative to the /vehicle frame, not /odom
  odom_msg.twist.twist.linear.x = odom_pos3d->vel.x;
  odom_msg.twist.twist.linear.y = odom_pos3d->vel.y;
  odom_msg.twist.twist.linear.z = odom_pos3d->vel.z;
  odom_msg.twist.twist.angular.x = odom_pos3d->vel.roll;
  odom_msg.twist.twist.angular.y = odom_pos3d->vel.pitch;
  odom_msg.twist.twist.angular.z = odom_pos3d->vel.yaw;

  /// \todo figure covariances of Pose and Twist

  odom_pub->publish(odom_msg);

  // publish the Imu message
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = *odom_time;
  imu_msg.header.frame_id = vr_.getFrame(ArtFrames::vehicle);
  imu_msg.orientation = odom_quat;
  imu_msg.angular_velocity = odom_msg.twist.twist.angular;
  imu_msg.linear_acceleration.x = adata.grp1.accel_lon;
  imu_msg.linear_acceleration.y = adata.grp1.accel_trans;
  imu_msg.linear_acceleration.z = - adata.grp1.accel_down;

  /// \todo figure covariances of IMU data

  imu_pub->publish(imu_msg);
}

void displayHelp() 
{
  std::cerr << "ART Applanix odometry driver\n"
            << std::endl
            << "Usage: rosrun applanix odometry <options>\n"
            << std::endl
            << "Options:\n"
            << "\t -h, -?       print usage message\n"
            << "\t -f <file>    use PCAP dump from <file>\n"
            << "\t -q <integer> set ROS topic queue depth (default: 1)\n"
            << "\t -t <file>    run unit test with fake data from <file>\n"
            << std::endl
            << "Example:\n"
            << "  rosrun applanix odometry -q2\n"
            << std::endl;
}


/** get command line and ROS parameters
 *
 * \returns 0 if successful
 */
int getParameters(int argc, char *argv[])
{
  static std::string pcap_file = "";    // pcap dump file
  static std::string test_file = "";    // unit test input file

  // use getopt to parse the flags
  char ch;
  const char* optflags = "hf:q:t:?";
  while(-1 != (ch = getopt(argc, argv, optflags)))
    {
      switch(ch)
        {
        case 'f':
          pcap_file = optarg;
          break;
        case 'q':
          qDepth = atoi(optarg);
          if (qDepth < 1)
            qDepth = 1;
          break;
        case 't':
          test_file = optarg;
          break;
        default:                        // unknown
          ROS_WARN("unknown parameter: %c", ch);
          // fall through to display help...
        case 'h':                       // help
        case '?':
          displayHelp();
          return 1;
        }
    }

  ROS_INFO("topic queue depth = %d", qDepth);
  
  // Create Applanix odometry device interface
  if (pcap_file != "")
    applanix_ = new DevApplanixPCAP(pcap_file);
  else if (test_file != "")
    applanix_ = new DevApplanixTest(test_file);
  else
    applanix_ = new DevApplanix();

  vr_.getPrefixParam();                 // get vehicle-relative tf prefix

  return 0;
}

/** main program */
int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  if (0 != getParameters(argc, argv))
    return 9;

  // make this a class w/constructor to initialize?
  memset(&adata, 0, sizeof(adata));
  adata.grp1.alignment = ApplStatusInvalid; // no valid solution yet

  // initialize odometry state
  ros::Time odom_time;                  // time last full packet read
  Position::Position3D odom_pos3d;      // current 3D position
  
  // connect to some ROS topics
  // no delay: we always want the most recent data
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Publisher odom_pub =
    node.advertise<nav_msgs::Odometry>("odom", qDepth);
  ros::Publisher imu_pub =
    node.advertise<sensor_msgs::Imu>("imu", qDepth);
  ros::Publisher gps_pub =
    node.advertise<art_msgs::GpsInfo>("gps", qDepth);
  tf::TransformBroadcaster odom_broadcaster;
  shifter_sub_ = node.subscribe("shifter/state", qDepth, getShifter, noDelay);

  // connect to Applanix data socket
  int rc = applanix_->connect_socket();
  if (rc != 0)				// device init failed?
    return 2;

  ros::Rate cycle(20);                  // set driver cycle rate
  
  ROS_INFO(NODE ": starting main loop");

  // main loop
  while(ros::ok())
    {
      if (getOdom(&odom_pos3d, &odom_time, &gps_pub))
        {
          // publish transform and odometry only when there are new
          // Applanix data
          putPose(&odom_pos3d, &odom_time, &odom_broadcaster,
                  &odom_pub, &imu_pub);
        }

      ros::spinOnce();                  // handle incoming messages
      cycle.sleep();                    // sleep until next cycle
    }

  ROS_INFO(NODE ": exiting main loop");

  // Stop the Applanix device, close its socket

  delete applanix_;

  return 0;
}
