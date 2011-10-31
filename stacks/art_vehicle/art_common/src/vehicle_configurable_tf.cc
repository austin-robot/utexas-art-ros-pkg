/*
 *  Copyright (C) 2011 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id: vehicle_configurable_tf.cc 1774 2011-09-10 19:23:37Z austinrobot $
 */

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>

#include <art/frames.h>
#include <art_msgs/ArtHertz.h>
#include <art_msgs/ArtVehicle.h>

#include <dynamic_reconfigure/server.h>
#include <art_common/CameraTransformConfig.h>

/**  \file

@brief ROS static transform broadcaster for the ART Vehicle with 
parameters set using dynamic reconfigure

This node broadcasts transforms from various devices to the
"/vehicle" frame of reference.  Some ROS components require that to be
done about once every 10 seconds. It provides a dynamic reconfigure
environment to configure these transforms.

@par Advertises

- \b /tf topic: broadcast transforms from \b /velodyne, \b
  /front_sick, and \b /rear_sick frames to \b /vehicle frame.

\author Jack O'Quin

*/

#define NODE "vehicle_configurable_tf"

namespace
{
  /// These are *static* transforms, so it's safe to post-date them
  //  into the future.  Otherwise, some transform listeners will see
  //  old data at times.
  ros::Duration transform_post_date_(1.0/art_msgs::ArtHertz::VEHICLE_TF);

  // class for generating vehicle-relative frame IDs
  ArtFrames::VehicleRelative vr_;

  art_common::CameraTransformConfig config_;
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

void callback(art_common::CameraTransformConfig &config, uint32_t level) {
  config_ = config;
}

/** main program */
int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  tf::TransformBroadcaster tf_broadcaster;
  vr_.getPrefixParam();                 // get vehicle-relative tf prefix

  ros::Rate cycle(art_msgs::ArtHertz::VEHICLE_TF); // set driver cycle rate
  
  // Start the dynamic reconfigure server to set the transform values
  dynamic_reconfigure::Server<art_common::CameraTransformConfig> srv;
  dynamic_reconfigure::Server<art_common::CameraTransformConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);

  ROS_INFO(NODE ": starting main loop");

  // main loop
  while(ros::ok())
    {
      using art_msgs::ArtVehicle;

      // Velodyne 3D LIDAR
      broadcastTF(&tf_broadcaster, ArtFrames::velodyne,
                  config_.velodyne_px,
                  config_.velodyne_py,
                  config_.velodyne_pz,
                  config_.velodyne_roll,
                  config_.velodyne_pitch,
                  config_.velodyne_yaw);

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
                  config_.left_front_camera_px,
                  config_.left_front_camera_py,
                  config_.left_front_camera_pz,
                  config_.left_front_camera_roll,
                  config_.left_front_camera_pitch,
                  config_.left_front_camera_yaw);

      // Center front camera
      opticalTF(&tf_broadcaster, ArtFrames::center_front_camera,
                  config_.center_front_camera_px,
                  config_.center_front_camera_py,
                  config_.center_front_camera_pz,
                  config_.center_front_camera_roll,
                  config_.center_front_camera_pitch,
                  config_.center_front_camera_yaw);

      // Right front camera
      opticalTF(&tf_broadcaster, ArtFrames::right_front_camera,
                  config_.right_front_camera_px,
                  config_.right_front_camera_py,
                  config_.right_front_camera_pz,
                  config_.right_front_camera_roll,
                  config_.right_front_camera_pitch,
                  config_.right_front_camera_yaw);

      ros::spinOnce();                  // handle incoming messages
      cycle.sleep();                    // sleep until next cycle
    }

  ROS_INFO(NODE ": exiting main loop");

  return 0;
}
