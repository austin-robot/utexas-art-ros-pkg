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
#include <art_msgs/ArtHertz.h>
#include <art_msgs/ArtVehicle.h>

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
  //  old data at times.
  ros::Duration transform_post_date_(1.0/art_msgs::ArtHertz::VEHICLE_TF);

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

  // broadcast Transform from vehicle to device
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

/** Publish the 3D pose of a device in the vehicle's frame of
    reference, plus its optical frame. */
void opticalTF(tf::TransformBroadcaster *tf_broadcaster,
               std::string device_frame,
               double x, double y, double z, 
               double roll, double pitch, double yaw)
{
  // first broadcast the device frame
  broadcastTF(tf_broadcaster, device_frame, x, y, z, roll, pitch, yaw);

  // broadcast Transform from device to corresponding optical frame
  geometry_msgs::TransformStamped optical_tf;
  optical_tf.header.stamp = ros::Time::now() + transform_post_date_;
  optical_tf.header.frame_id = vr_.getFrame(device_frame);
  optical_tf.child_frame_id = vr_.getFrame(device_frame + "_optical");

  // this Quaternion rotates the device frame into the optical frame
  optical_tf.transform.rotation.w = 0.5;
  optical_tf.transform.rotation.x = -0.5;
  optical_tf.transform.rotation.y = 0.5;
  optical_tf.transform.rotation.z = -0.5;

  tf_broadcaster->sendTransform(optical_tf);
}

/** main program */
int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  tf::TransformBroadcaster tf_broadcaster;
  vr_.getPrefixParam();                 // get vehicle-relative tf prefix

  ros::Rate cycle(art_msgs::ArtHertz::VEHICLE_TF); // set driver cycle rate
  
  ROS_INFO(NODE ": starting main loop");

  // main loop
  while(ros::ok())
    {
      using art_msgs::ArtVehicle;

      // Velodyne 3D LIDAR
      broadcastTF(&tf_broadcaster, ArtFrames::velodyne,
                  ArtVehicle::velodyne_px,
                  ArtVehicle::velodyne_py,
                  ArtVehicle::velodyne_pz,
                  ArtVehicle::velodyne_roll,
                  ArtVehicle::velodyne_pitch,
                  ArtVehicle::velodyne_yaw);

      // Front Sick LIDAR
      broadcastTF(&tf_broadcaster, ArtFrames::front_sick,
                  ArtVehicle::front_SICK_px,
                  ArtVehicle::front_SICK_py,
                  ArtVehicle::front_SICK_pz,
                  ArtVehicle::front_SICK_roll,
                  ArtVehicle::front_SICK_pitch,
                  ArtVehicle::front_SICK_yaw);

      // Rear Sick LIDAR
      broadcastTF(&tf_broadcaster, ArtFrames::rear_sick,
                  ArtVehicle::rear_SICK_px,
                  ArtVehicle::rear_SICK_py,
                  ArtVehicle::rear_SICK_pz,
                  ArtVehicle::rear_SICK_roll,
                  ArtVehicle::rear_SICK_pitch,
                  ArtVehicle::rear_SICK_yaw);

      // Left front camera
      opticalTF(&tf_broadcaster, ArtFrames::left_front_camera,
                  ArtVehicle::left_front_camera_px,
                  ArtVehicle::left_front_camera_py,
                  ArtVehicle::left_front_camera_pz,
                  ArtVehicle::left_front_camera_roll,
                  ArtVehicle::left_front_camera_pitch,
                  ArtVehicle::left_front_camera_yaw);

      // Center front camera
      opticalTF(&tf_broadcaster, ArtFrames::center_front_camera,
                  ArtVehicle::center_front_camera_px,
                  ArtVehicle::center_front_camera_py,
                  ArtVehicle::center_front_camera_pz,
                  ArtVehicle::center_front_camera_roll,
                  ArtVehicle::center_front_camera_pitch,
                  ArtVehicle::center_front_camera_yaw);

      // Right front camera
      opticalTF(&tf_broadcaster, ArtFrames::right_front_camera,
                  ArtVehicle::right_front_camera_px,
                  ArtVehicle::right_front_camera_py,
                  ArtVehicle::right_front_camera_pz,
                  ArtVehicle::right_front_camera_roll,
                  ArtVehicle::right_front_camera_pitch,
                  ArtVehicle::right_front_camera_yaw);

      ros::spinOnce();                  // handle incoming messages
      cycle.sleep();                    // sleep until next cycle
    }

  ROS_INFO(NODE ": exiting main loop");

  return 0;
}
