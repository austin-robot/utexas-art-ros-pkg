#include "learned_controller.h"
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>

/** Acceleration matrix speed control constructor. */
LearnedSpeedControl::LearnedSpeedControl():
  SpeedControl(),numactions(5)
{
  LOADDEBUG = false;
  loaded = false;

  ROS_WARN("Learned Speed Controller, load policy");

  // create agent and load file
  std::string policyPath = (ros::package::getPath("art_pilot")
			    + "/src/pilot/control1400.pol");
  loadPolicy(policyPath.c_str());

  // init state vector
  s.resize(4,0);
}

/** LearnedSpeedControl destructor */
LearnedSpeedControl::~LearnedSpeedControl()
{
  statespace.clear();
  Q.clear();
  s.clear();
}

/** Adjust speed to match goal.

    Generate brake and throttle changes from velocity PID controller
    via an acceleration matrix.

    @param speed absolute value of current velocity in m/sec
    @param error immediate goal minus speed
    @param brake_req -> previous brake request (input),
                        updated brake request (output).
    @param throttle_req -> previous throttle request (input),
                        updated throttle request (output).
*/
void LearnedSpeedControl::adjust(float speed, float error,
                                float *brake_req, float *throttle_req)
{

  // lets get actual target vel
  float targetVel = speed + error;

  if (targetVel > 11.0 || targetVel < 0.0){
    ROS_DEBUG("Target Vel out of range: %f", targetVel);
  }

  ROS_DEBUG("Speed %f, error %f, target %f", speed, error, targetVel);
  ROS_DEBUG("Throt_pos %f, Throt_req %f, brake_pos %f, brake_req %f",
           throttle_position_, *throttle_req, brake_position_, *brake_req);

  // out of range
  if (targetVel < 0)
    targetVel = 0;
  if (targetVel > 11)
    targetVel = 11;
  if (speed < 0)
    speed = 0;
  if (speed > 12)
    speed = 12;

  // convert to discrete
  float f1 = 0.5;
  float f2 = 0.1;
  float f3 = 0.1;
  float f4 = 0.1;

  float EPSILON = 0.001;

  s[0] = (int)((targetVel+EPSILON) / f1);
  s[1] = (int)((speed+EPSILON) / f2);
  s[2] = (int)((*throttle_req+EPSILON) / f3);
  s[3] = (int)((*brake_req+EPSILON) / f4);

  ROS_DEBUG("State: %f, %f, %f, %f", s[0], s[1], s[2], s[3]);

  // get action from agent
  int act = getAction(s);
  
  // fix trouble starting from full brake
  if (speed < 0.01 && targetVel > 0 && *brake_req > 0.0 && act != 3){
    ROS_WARN("Chose bad accel from stop. State %f, %f, %f, %f, action %i",
             s[0], s[1], s[2], s[3], act);
    act = 3;
  }

  // set throttle and brake based on action
  if (act == 0){
    // no change
  } else if (act == 1){
    *throttle_req = 0;
    *brake_req += 0.1;
  } else if (act == 2){
    *throttle_req = 0;
    *brake_req -= 0.1;
  } else if (act == 3){
    *throttle_req += 0.1;
    *brake_req = 0;
  } else if (act == 4){
    *throttle_req -= 0.1;
    *brake_req = 0;
  } else {
    ROS_DEBUG("ERROR: invalid action: %i", act);
  }

  // dont allow throttle over 0.4
  if(*throttle_req > 0.4)
    *throttle_req = 0.4;

  ROS_DEBUG("action %i, throttle %f, brake %f", act, *throttle_req, *brake_req);

}

/** Configure controller parameters. */
void LearnedSpeedControl::configure(art_pilot::PilotConfig &newconfig)
{
}

/** Reset speed controller. */
void LearnedSpeedControl::reset(void)
{
}





int LearnedSpeedControl::getAction(const std::vector<float> &s) {

  // Get action values
  std::vector<float> &Q_s = Q[canonicalize(s)];
  const std::vector<float>::iterator max =
    std::max_element(Q_s.begin(), Q_s.end());

  // Choose an action
  const std::vector<float>::iterator a = max;

  return a - Q_s.begin();
}


LearnedSpeedControl::state_t LearnedSpeedControl::canonicalize(const std::vector<float> &s) {
  const std::pair<std::set<std::vector<float> >::iterator, bool> result =
    statespace.insert(s);
  state_t retval = &*result.first; // Dereference iterator then get pointer 
  if (result.second) { // s is new, so initialize Q(s,a) for all a
    if (loaded){
      ROS_ERROR("State unknown in policy: %f, %f, %f, %f", s[0], s[1], s[2], s[3]);
    }
    std::vector<float> &Q_s = Q[retval];
    Q_s.resize(numactions,0.0);
  }
  return retval; 
}


void LearnedSpeedControl::loadPolicy(const char* filename){

  using namespace std;

  ifstream policyFile(filename, ios::in | ios::binary);

  // first part, save the vector size
  int fsize;
  policyFile.read((char*)&fsize, sizeof(int));
  if (fsize != 4){
    ROS_WARN("this policy is not valid, loaded nfeat %i, instead of 4", fsize);
  }

  // save numactions
  int nact;
  policyFile.read((char*)&nact, sizeof(int));

  if (nact != 5){
    ROS_DEBUG("this policy is not valid, loaded nact %i, instead of 5", nact);
  }

  // go through all states, loading q values
  while(!policyFile.eof()){
    std::vector<float> state;
    state.resize(fsize, 0.0);

    // load state
    policyFile.read((char*)&(state[0]), sizeof(float)*fsize);
    if (LOADDEBUG){
      ROS_DEBUG("load policy for state %f, %f, %f, %f", state[0], state[1], state[2], state[3]); 
    }

    state_t s = canonicalize(state);

    if (policyFile.eof()) break;

    // load q values
    policyFile.read((char*)&(Q[s][0]), sizeof(float)*numactions);
    
    if (LOADDEBUG){
      ROS_DEBUG("Q values: %f, %f, %f, %f, %f", Q[s][0],Q[s][1],Q[s][2],Q[s][3],Q[s][4]);
    }
  }
  
  policyFile.close();
  ROS_DEBUG("Policy loaded!!!");
  loaded = true;
}
