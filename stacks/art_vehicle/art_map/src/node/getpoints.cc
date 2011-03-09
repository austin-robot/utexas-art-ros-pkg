#include <ros/ros.h>
#include <art_msgs/GpsInfo.h>
#include <boost/thread/mutex.hpp>

#define NODE "getpoints"

namespace {

  int qDepth = 1;
  ros::Subscriber gpsSubscriber_;

  enum {
    EMPTY, 
    LANE_POINT, 
    PERIM_POINT, 
    SPOT_POINT, 
    END, 
    QUIT
  };

  double lat;
  double lon;

  boost::mutex mGps; 
}

int getInstruction() {

  char instruction;
  std::cin.get(instruction);
  std::cin.ignore();

  if(!instruction) return EMPTY;
  
  switch(instruction) {
    case 'l':
    case 'L':
      return LANE_POINT;
    case 'p':
    case 'P':
      return PERIM_POINT;
    case 's':
    case 'S':
      return SPOT_POINT;
    case 'e':
    case 'E':
      return END;
    case 'q':
    case 'Q':
      return QUIT;
    default:
      return EMPTY;
  }
}

void getGpsData(const art_msgs::GpsInfo &gpsInfo) {
  mGps.lock();
  lat = gpsInfo.latitude;
  lon = gpsInfo.longitude;
  mGps.unlock();
}

int main(int argc, char** argv) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  gpsSubscriber_ = node.subscribe("gps", qDepth, getGpsData, noDelay);

  //Display instructions to user
  ROS_INFO("Valid instructions are:");
  ROS_INFO("L record lane point");
  ROS_INFO("P record zone perimeter point");
  ROS_INFO("S record parking spot point");
  ROS_INFO("E end the current lane or zone point list");
  ROS_INFO("Q quit");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Open waypt file
  FILE *waypoints = fopen("waypoints.txt", "w");
  if (waypoints == NULL) {
    ROS_ERROR("Couldn't open waypoints.txt\n");
    return 1;
  }

  fprintf(waypoints, "file_start\n");
  int lastCommand = EMPTY, instruction;
  int insertEnd = 0;
  bool expectNewCommand = true;

  while (ros::ok() && expectNewCommand) {

    instruction = getInstruction();

	  if ((lastCommand == LANE_POINT && (instruction == PERIM_POINT || instruction == SPOT_POINT)) ||
	     (instruction == LANE_POINT && (lastCommand == PERIM_POINT || lastCommand == SPOT_POINT))) {
      insertEnd = true;
    }
	  if (insertEnd) {
	    insertEnd = false;
	    fprintf(waypoints, "end\n"); 
	    ROS_INFO("end\n"); 
	  }

    mGps.lock();

	  switch(instruction) {

	    case LANE_POINT:
	      if(lastCommand == PERIM_POINT || lastCommand == SPOT_POINT || lastCommand == END || lastCommand == EMPTY) {
		      ROS_INFO("start_lane %i", lastCommand); 
	      }
	      fprintf(waypoints, "lane_point	%0.7lf	%0.7lf\n", lat, lon); 
	      ROS_INFO("\tlane_point	%0.7lf	%0.7lf", lat, lon); 
	      break;

	    case PERIM_POINT:
	      if(lastCommand == LANE_POINT || lastCommand == END || lastCommand == EMPTY) {
		      ROS_INFO("start_zone"); 
	      }
	      fprintf(waypoints, "perim_point	%0.7lf	%0.7lf\n", lat, lon); 
	      ROS_INFO("\tperim_point	%0.7lf	%0.7lf\n", lat, lon); 
	      break;	

	    case SPOT_POINT:
	      if(lastCommand == LANE_POINT || lastCommand == END || lastCommand == EMPTY) {
		      ROS_INFO("start_zone"); 
	      }
	      fprintf(waypoints, "spot_point	%0.7lf	%0.7lf\n", lat, lon); 
	      ROS_INFO("\tspot_point	%0.7lf	%0.7lf", lat, lon);
	      break;

	    case END:
	      if(lastCommand != END && lastCommand != EMPTY) {
		      fprintf(waypoints, "end\n"); 
		      ROS_INFO("end"); 
	      }
	      break;

	    case QUIT:
	      fprintf(waypoints, "file_end");
	      ROS_INFO("Quitting application.");
	      expectNewCommand = false;
	      break;

	    default:
	      break;
	  }

    mGps.unlock();

	  lastCommand = instruction;
	  fflush(waypoints);
  }

  spinner.stop();
  fclose(waypoints);

  return 0;
}
