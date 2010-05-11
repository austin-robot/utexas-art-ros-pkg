/*
 *  Copyright (C) 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>

#include <art/frames.h>
#include <art/hertz.h>
#include <art/vehicle.hh>

/**  \file

@brief ROS static transform broadcaster for the ART autonomous vehicle.

This node broadcasts static transforms from various devices to the
"/vehicle" frame of reference.  Some ROS components require that to be
done about once every 10 seconds.

@par Advertises

- \b /tf topic: broadcast transforms from \b /velodyne, \b
  /front_sick, and \b /rear_sick frames to \b /vehicle frame.

\author Jack O'Quin

*/

#define NODE "vehicle_static_tf"

namespace
{
  /// These are *static* transforms, so it's safe to post-date them
  //  into the future.  Otherwise, some transform listeners will see
  //  old data at times. The extra 10% duration ensures an overlap
  //  between successive messages.
  ros::Duration transform_post_date_(1.1/HERTZ_VEHICLE_TF);

  // class for generating vehicle-relative frame IDs
  ArtFrames::VehicleRelative vr_;
}

/** Publish the 3D pose of a device in the vehicle's frame of reference. */
void broadcastTF(tf::TransformBroadcaster *tf_broadcaster,
                 std::string device_frame,
                 double x, double y, double z, 
                 double roll, double pitch, double yaw)
{
  // translate roll, pitch and yaw into a Quaternion
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion tf_quat;
  tf::quaternionTFToMsg(q, tf_quat);

  // broadcast Transform from device to vehicle
  geometry_msgs::TransformStamped static_tf;
  static_tf.header.stamp = ros::Time::now() + transform_post_date_;
  static_tf.header.frame_id = vr_.getFrame(ArtFrames::vehicle);
  static_tf.child_frame_id = vr_.getFrame(device_frame);
  static_tf.transform.translation.x = x;
  static_tf.transform.translation.y = y;
  static_tf.transform.translation.z = z;
  static_tf.transform.rotation = tf_quat;

  tf_broadcaster->sendTransform(static_tf);
}

/** main program */
int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  tf::TransformBroadcaster tf_broadcaster;
  vr_.getPrefixParam();                 // get vehicle-relative tf prefix

  ros::Rate cycle(HERTZ_VEHICLE_TF);    // set driver cycle rate
  
  ROS_INFO(NODE ": starting main loop");

  // main loop
  while(ros::ok())
    {
      using namespace ArtVehicle;

      // Velodyne 3D LIDAR
      broadcastTF(&tf_broadcaster, ArtFrames::velodyne,
                  velodyne_px, velodyne_py, velodyne_pz,
                  velodyne_roll, velodyne_pitch, velodyne_yaw);

      // Front Sick LIDAR
      broadcastTF(&tf_broadcaster, ArtFrames::front_sick,
                  front_SICK_px, front_SICK_py, front_SICK_pz,
                  front_SICK_roll, front_SICK_pitch, front_SICK_yaw);

      // Rear Sick LIDAR
      broadcastTF(&tf_broadcaster, ArtFrames::rear_sick,
                  rear_SICK_px, rear_SICK_py, rear_SICK_pz,
                  rear_SICK_roll, rear_SICK_pitch, rear_SICK_yaw);

      // Front Right Camera
      broadcastTF(&tf_broadcaster, ArtFrames::front_right_camera,
                  front_right_camera_px, front_right_camera_py, front_right_camera_pz,
                  front_right_camera_roll, front_right_camera_pitch, front_right_camera_yaw);

      ros::spinOnce();                  // handle incoming messages
      cycle.sleep();                    // sleep until next cycle
    }

  ROS_INFO(NODE ": exiting main loop");

  return 0;
}
