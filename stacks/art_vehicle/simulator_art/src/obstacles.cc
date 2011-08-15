#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h> 

#define NODE "art_simulator_obstalce_converter"

namespace {

  int qDepth = 1;

  ros::Subscriber subLaserScan_;
  ros::Publisher pubPointCloud_;

  sensor_msgs::PointCloud pc;

}

void processLaserScan(const sensor_msgs::LaserScan &laserScan) {

  // Pass the header information along
  pc.header = laserScan.header;

  int maxPoints = ceil((laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment) + 1;
  pc.points.resize(maxPoints);

  float currentAngle = laserScan.angle_min;
  int angleIndex = 0;
  int numPoints = 0;
  while(currentAngle <= laserScan.angle_max) {

    float distance = laserScan.ranges[angleIndex];
    
    // See if an obstacle was detected
    if (distance > laserScan.range_min && distance < laserScan.range_max) { 
      float xPt = cosf(currentAngle) * distance;
      float yPt = sinf(currentAngle) * distance;

      pc.points[numPoints].x = xPt;
      pc.points[numPoints].y = yPt;
      pc.points[numPoints].z = 0.25;
      numPoints++;
    }

    angleIndex++;
    currentAngle += laserScan.angle_increment;
  }

  pc.points.resize(numPoints);

  pubPointCloud_.publish(pc);
}

int getParameters(int argc, char *argv[]) {
  char ch;
  const char* optflags = "q:";
  while(-1 != (ch = getopt(argc, argv, optflags))) {
    switch(ch) {

      case 'q':
        qDepth = atoi(optarg);
        if (qDepth < 1) {
          qDepth = 1;
        }
        break;
    }
  }

  return 1;
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle node;
  
  if (!getParameters(argc, argv))
    return -1; 

  // Subscribers
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  subLaserScan_ =
      node.subscribe("laser/front", qDepth, &processLaserScan, noDelay);
  
  // Publishers
  pubPointCloud_ = node.advertise<sensor_msgs::PointCloud>("velodyne/obstacles", qDepth);

  ROS_INFO(NODE ": starting up");
  
  ros::spin();                          // handle incoming data

  ROS_INFO(NODE ": shutting down");
 
  return 0;

}
