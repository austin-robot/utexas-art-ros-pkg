/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology, Michael Quinlan
 *  Copyright (C) 2011 UT-Austin & Austin Robot Technology
 *  License: Modified BSD Software License 
 */

/** @file

    Lane observers class implementation.

    Creates and runs the observers that use map lanes for a pseudo
    occupancy grid. Publishes their observations.

    @author Michael Quinlan
    @author Jack O'Quin

*/

#include <algorithm>
#include <ros/ros.h>
#include <art_observers/lane_observations.h>

/** @brief run lane observers, publish their observations */
LaneObservations::LaneObservations(ros::NodeHandle &node):
  node_(node),
  tf_listener_(new tf::TransformListener())
{ 
 // subscribe to obstacle cloud
  obstacle_sub_ =
    node_.subscribe("velodyne/obstacles", 1,
                    &LaneObservations::processObstacles, this,
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
}

LaneObservations::~LaneObservations() {}

void LaneObservations::processObstacles(const sensor_msgs::PointCloud &msg) 
{
  observations_.header.stamp = msg.header.stamp;
  obs_quads_.polygons.clear();
  added_quads_.clear();
  transformPointCloud(msg);
  
  // skip the rest until the local road map has been received at least once
  if (local_map_.header.stamp > ros::Time())
    {
      filterPointsInLocalMap();
      runObservers();
      publishObstacleVisualization();
    }
}

void LaneObservations::processLocalMap(const art_msgs::ArtLanes &msg) 
{
  local_map_ = msg;
}

void LaneObservations::processPose(const nav_msgs::Odometry &odom)
{
  pose_ = MapPose(odom.pose.pose);
}

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

void LaneObservations::transformPointCloud(const sensor_msgs::PointCloud &msg) 
{
  try
    {
      tf_listener_->transformPointCloud("/map", msg, obstacles_);
      observations_.header.frame_id = obstacles_.header.frame_id;
      calcRobotPolygon();            // hopefully not needed in future
    }
  catch (tf::TransformException ex)
    {
      // only log tf error once every 20 times
      ROS_WARN_STREAM_THROTTLE(20, ex.what());
    }
}

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

void LaneObservations::runObservers() 
{
  // run all registered observers
  for (unsigned i = 0; i < observers_.size(); ++i)
    {
      observations_.obs[i] =
	observers_[i]->update(robot_polygon_, local_map_, obs_quads_);
    }

#if 0
  // update nearest rear observer
  art_msgs::ArtLanes nearest_rear_quads =
    quad_ops::filterLanes(robot_polygon_,local_map_,
                          *quad_ops::compare_backward_seg_lane);
  art_msgs::ArtLanes nearest_rear_obstacles =
    quad_ops::filterLanes(robot_polygon_,obs_quads_,
                          *quad_ops::compare_backward_seg_lane);

  // reverse the vectors because the observer experts polygons in
  // order of distance from base polygon
  std::reverse(nearest_rear_quads.polygons.begin(),
               nearest_rear_quads.polygons.end());
  std::reverse(nearest_rear_obstacles.polygons.begin(),
               nearest_rear_obstacles.polygons.end());

  observations_.obs[1] =
    nearest_rear_observer_.update(robot_polygon_.poly_id,
                                  nearest_rear_quads,
                               nearest_rear_obstacles);
#endif
  // Publish observations
  observations_pub_.publish(observations_);
}                                                            


void LaneObservations::calcRobotPolygon() 
{
  // --- Hack to get my own polygon
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "/vehicle";  
  laser_point.header.stamp = ros::Time();
  
  laser_point.point.x = 0.0;
  laser_point.point.y = 0.0;
  laser_point.point.z = 0.0;
  
  geometry_msgs::PointStamped robot_point;
  tf_listener_->transformPoint("/map", laser_point, robot_point);

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
      mark.header.frame_id = "/map";
    
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
  mark.header.frame_id = "/map";
  
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

/** Handle incoming data. */
void LaneObservations::spin()
{
  ros::spin();
}
