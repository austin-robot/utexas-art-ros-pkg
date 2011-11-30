/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology, Michael Quinlan
 *  Copyright (C) 2011 UT-Austin & Austin Robot Technology
 *  License: Modified BSD Software License
 *
 *  $Id$
 */

/** @file

    Lane observers class implementation.

    Creates and runs the observers that use map lanes for a pseudo
    occupancy grid. Publishes their observations.

    @author Michael Quinlan
    @author Jack O'Quin

*/

#include <sensor_msgs/point_cloud_conversion.h>
#include <art_observers/lane_observations.h>
#include <art_observers/QuadrilateralOps.h>

/** @brief Run lane observers, publish their observations. */
LaneObservations::LaneObservations(ros::NodeHandle &node,
				   ros::NodeHandle &priv_nh):
  node_(node),
  priv_nh_(priv_nh),
  config_(priv_nh),
  nearest_forward_observer_(config_),
  nearest_backward_observer_(config_),
  adjacent_left_observer_(config_),
  adjacent_right_observer_(config_),
  tf_listener_(new tf::TransformListener())
{ 
  // subscribe to point cloud topics
  pc_sub_ =
    node_.subscribe("velodyne/obstacles", 1,
                    &LaneObservations::processPointCloud, this,
                    ros::TransportHints().tcpNoDelay(true));
  pc2_sub_ =
    node_.subscribe("obstacles", 1,
                    &LaneObservations::processPointCloud2, this,
                    ros::TransportHints().tcpNoDelay(true));

  // subscribe to local road map
  road_map_sub_ =
    node_.subscribe("roadmap_local", 1,
                    &LaneObservations::processLocalMap, this,
                    ros::TransportHints().tcpNoDelay(true));

  // subscribe to odometry
  odom_sub_ = 
    node_.subscribe("odom", 1,
                    &LaneObservations::processPose, this,
                    ros::TransportHints().tcpNoDelay(true));

  // advertise published topics
  const std::string viz_topic("visualization_marker_array");
  viz_pub_ =
    node_.advertise<visualization_msgs::MarkerArray>(viz_topic, 1, true);
  observations_pub_ =
    node_.advertise <art_msgs::ObservationArray>("observations", 1, true);

  // Initialize observers.  They will be updated in this order.
  addObserver(nearest_forward_observer_);
  addObserver(nearest_backward_observer_);
  addObserver(adjacent_left_observer_);
  addObserver(adjacent_right_observer_);
}

/** @brief Deconstructor. */
LaneObservations::~LaneObservations() {}

/** @brief Obstacles point cloud processing.  Starts all the observers
 *
 *  @pre @c obstacles_ contains the point cloud data received
 */
void LaneObservations::processObstacles(void) 
{
  observations_.header.stamp = obstacles_.header.stamp;
  obs_quads_.polygons.clear();
  transformPointCloud(obstacles_);
  
  // skip the rest until the local road map has been received at least once
  if (local_map_.header.stamp > ros::Time())
    {
      filterPointsInLocalMap();
      runObservers();
      publishObstacleVisualization();
    }
}

/** @brief PointCloud callback. */
void LaneObservations::processPointCloud(const sensor_msgs::PointCloud::ConstPtr &msg)
{
  // convert to PointCloud2, then process that
  boost::shared_ptr<sensor_msgs::PointCloud2> pc2(new sensor_msgs::PointCloud2);
  sensor_msgs::convertPointCloudToPointCloud2(*msg, *pc2);
  processPointCloud2(pc2);
}

/** @brief PointCloud2 callback. */
void LaneObservations::processPointCloud2(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // convert ROS message into PCL point cloud
  pcl::fromROSMsg(*msg, obstacles_);

  // process the PCL point cloud data
  processObstacles();
}

/** @brief Local road map callback. */
void LaneObservations::processLocalMap(const art_msgs::ArtLanes::ConstPtr &msg) 
{
  local_map_ = *msg;
}

/** @brief process the pose of the map **/
void LaneObservations::processPose(const nav_msgs::Odometry &odom)
{
  pose_ = MapPose(odom.pose.pose);
}

/** @brief Filter obstacle points to those in a road map polygon. */
void LaneObservations::filterPointsInLocalMap() 
{
  // set the exact point cloud size
  size_t npoints = obstacles_.points.size();
  added_quads_.clear();
  for (unsigned i = 0; i < npoints; ++i)
    {
      isPointInAPolygon(obstacles_.points[i].x,
                        obstacles_.points[i].y);
    }
}

/** @brief Transform obstacle points into the map frame of reference. */
void LaneObservations::transformPointCloud(const PtCloud &msg) 
{
  try
    {
      // wait for transform to be available
      tf_listener_->waitForTransform(config_.map_frame_id, msg.header.frame_id,
				     msg.header.stamp, ros::Duration(0.2));
      pcl_ros::transformPointCloud(config_.map_frame_id, msg, obstacles_,
				   *tf_listener_);
      observations_.header.frame_id = obstacles_.header.frame_id;

      // hopefully not needed in future
      calcRobotPolygon();
    }
  catch (tf::TransformException ex)
    {
      // only log tf error once every 20 times
      ROS_WARN_STREAM_THROTTLE(20, ex.what());
    }
}

/** Point in polygon predicate.
 *
 *  @return true if (X, Y) is within a road map polygon.
 *
 *  @post @a added_quads contains polygon ID, if found.
 *        @a obstacle_quads contains polygon, if found.
 */
bool LaneObservations::isPointInAPolygon(float x, float y) 
{
  size_t num_polys = local_map_.polygons.size();
  
  bool inside = false;
  std::pair<std::tr1::unordered_set<int>::iterator, bool> pib;
  
  for (size_t i=0; i<num_polys; i++)
    {
      art_msgs::ArtQuadrilateral *p= &(local_map_.polygons[i]);
      float dist= ((p->midpoint.x-x)*(p->midpoint.x-x)
                   + (p->midpoint.y-y)*(p->midpoint.y-y));

      if (dist > 16)         // quick check: are we near the polygon?
        continue;

      inside = quad_ops::quickPointInPolyRatio(x,y,*p,0.6);
      // Add polygon to lane if so
      if (inside)
        {
          pib = added_quads_.insert(p->poly_id);
          if (pib.second)
            {
              obs_quads_.polygons.push_back(*p);
            }
        }
    }

  return inside;
}

/** @brief Run all registered observers and publish their observations. */
void LaneObservations::runObservers() 
{
  // update all the registered observers
  for (unsigned i = 0; i < observers_.size(); ++i)
    {
      observations_.obs[i] =
	observers_[i]->update(local_map_, obs_quads_, pose_);
    }

  // Publish their observations
  observations_pub_.publish(observations_);
}                                                            

/** @brief Calculate which polygon currently contains the robot. */
void LaneObservations::calcRobotPolygon() 
{
  // --- Hack to get my own polygon
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = config_.robot_frame_id;  
  laser_point.header.stamp = ros::Time();
  
  laser_point.point.x = 0.0;
  laser_point.point.y = 0.0;
  laser_point.point.z = 0.0;
  
  geometry_msgs::PointStamped robot_point;
  tf_listener_->transformPoint(config_.map_frame_id, laser_point, robot_point);

  size_t numPolys = local_map_.polygons.size();
  float x = robot_point.point.x; 
  float y = robot_point.point.y;
  bool inside=false;
  for (size_t i=0; i<numPolys; i++)
    {
      art_msgs::ArtQuadrilateral *p= &(local_map_.polygons[i]);
      float dist = ((p->midpoint.x-x)*(p->midpoint.x-x)
                    + (p->midpoint.y-y)*(p->midpoint.y-y));

      if (dist > 16)          // quick check: we are near the polygon?
        continue;

      inside = quad_ops::quickPointInPoly(x,y,*p); 
      if (inside)
        {
          robot_polygon_ = *p;
        }
    }
}

/** @brief Publish rviz markers for obstacles in the road. */
void LaneObservations::publishObstacleVisualization()
{
  if (viz_pub_.getNumSubscribers()==0)
    return;

  ros::Time now = ros::Time::now();

  // clear message array, this is a class variable to avoid memory
  // allocation and deallocation on every cycle
  marks_msg_.markers.clear();

  int i=0;
  for (obs_it_=obs_quads_.polygons.begin();
       obs_it_!=obs_quads_.polygons.end(); obs_it_++)
    {
      visualization_msgs::Marker mark;
      mark.header.stamp = now;
      mark.header.frame_id = config_.map_frame_id;
    
      mark.ns = "obstacle_polygons";
      mark.id = (int32_t) i;
      mark.type = visualization_msgs::Marker::CUBE;
      mark.action = visualization_msgs::Marker::ADD;
    
      mark.pose.position = obs_it_->midpoint;
      mark.pose.orientation =
        tf::createQuaternionMsgFromYaw(obs_it_->heading);
    
      mark.scale.x = 1.5;
      mark.scale.y = 1.5;
      mark.scale.z = 0.1;
      mark.lifetime =  ros::Duration(0.2);
    
      mark.color.a = 0.8;       // way-points are slightly transparent
      mark.color.r = 0.0;
      mark.color.g = 0.0;
      mark.color.b = 1.0;
    
      marks_msg_.markers.push_back(mark);
      i++;
    }

  // Draw the polygon containing the robot
  visualization_msgs::Marker mark;
  mark.header.stamp = now;
  mark.header.frame_id = config_.map_frame_id;
  
  mark.ns = "obstacle_polygons";
  mark.id = (int32_t) i;
  mark.type = visualization_msgs::Marker::CUBE;
  mark.action = visualization_msgs::Marker::ADD;
    
  mark.pose.position = robot_polygon_.midpoint;
  mark.pose.orientation =
    tf::createQuaternionMsgFromYaw(robot_polygon_.heading);
    
  mark.scale.x = 1.5;
  mark.scale.y = 1.5;
  mark.scale.z = 0.1;
  mark.lifetime =  ros::Duration(0.2);
  
  mark.color.a = 0.8;           // way-points are slightly transparent
  mark.color.r = 0.3;
  mark.color.g = 0.7;
  mark.color.b = 0.9;
  
  marks_msg_.markers.push_back(mark);

  // Publish the markers
  viz_pub_.publish(marks_msg_);
}

/** @brief Handle incoming data. */
void LaneObservations::spin()
{
  ros::spin();
}
