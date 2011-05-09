/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2005, 2007, 2009, 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART autonomous vehicle speed controller interface.
 
        SpeedControl -- virtual base speed controller class
   
        SpeedControlMatrix -- speed control using acceleration matrix

        SpeedControlPID -- speed control using direct PID of throttle
                           and brake

     @author Jack O'Quin

 */

#ifndef __SPEED_H_
#define __SPEED_H_

#include <ros/ros.h>
#include <art/pid2.h>
#include <art_pilot/PilotConfig.h>

// epsilon values for brake and throttle requests
#define EPSILON_BRAKE 0.01
#define EPSILON_THROTTLE 0.01

/** Virtual base speed controller class */
class SpeedControl
{
 public:

  SpeedControl()
  {
    // initialize private node handle for getting parameter settings
    node_ = ros::NodeHandle("~");
  };

  /** Adjust speed to match goal.

      @param speed absolute value of current velocity in m/sec
      @param error immediate goal minus speed
      @param brake_req -> previous brake request (input),
                          updated brake request (output).
      @param throttle_req -> previous throttle request (input),
                          updated throttle request (output).
  */
  virtual void adjust(float speed, float error,
                      float *brake_req, float *throttle_req) = 0;

  /** Configure controller parameters. */
  virtual void configure(art_pilot::PilotConfig &newconfig) = 0;

  /** Reset speed controller. */
  virtual void reset(void) = 0;

  virtual void set_brake_position(float position)
  {
    brake_position_ = position;
  }
  virtual void set_throttle_position(float position)
  {
    throttle_position_ = position;
  }

protected:

  ros::NodeHandle node_;                // private node handle for parameters
  float brake_position_;
  float throttle_position_;
};

/** Acceleration matrix speed controller class */
class SpeedControlMatrix: public SpeedControl
{
 public:

  SpeedControlMatrix();
  virtual ~SpeedControlMatrix();
  virtual void adjust(float speed, float error,
                      float *brake_req, float *throttle_req);
  virtual void configure(art_pilot::PilotConfig &newconfig);
  virtual void reset(void);

 private:

  boost::shared_ptr<Pid> velpid_;       // velocity PID control
};

/** PID speed controller class */
class SpeedControlPID: public SpeedControl
{
 public:

  SpeedControlPID();
  virtual ~SpeedControlPID();
  virtual void adjust(float speed, float error,
                      float *brake_req, float *throttle_req);
  virtual void configure(art_pilot::PilotConfig &newconfig);
  virtual void reset(void);

 private:

  // When true, brake is the controlling device, otherwise throttle.
  bool braking_;

  boost::shared_ptr<Pid> brake_pid_;    // Brake_Pilot control PID
  boost::shared_ptr<Pid> throttle_pid_; // Throttle control PID
};

#endif // __SPEED_H_
