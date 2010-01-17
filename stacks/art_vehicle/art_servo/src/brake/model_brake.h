/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file
 
     Interface to model of ART autonomous vehicle brake actuator.

     \author Jack O'Quin

 */

#ifndef _MODEL_BRAKE_H_
#define _MODEL_BRAKE_H_ 1

// ROS interfaces
#include <ros/ros.h>

#include "animatics.h"

/** Actuator movement plan.

    This class is subordinate to ArtBrakeModel and not really
    independent of it, so it does not need a separate header.
 */
class ActuatorPlan
{
public:

  /** Constructor. */
  ActuatorPlan() {reset();}

  /** Add a step to the current plan. */
  void append(double t, double a)
  {
    if (nsteps_ < MAX_STEPS)
      {
        steps_[nsteps_].until = t;
        steps_[nsteps_].accel = a;
        ++nsteps_;
      }
  }

  /** Return true if no more movement planned. */
  bool finished() {return nsteps_ == 0;}

  /** Return smallest interval (in seconds) until end of:
                (1) finish time,
                (2) current plan step.
  */
  double interval(double t, double finish)
  {
    // (1) interval to finish
    double dt = finish - t;
    if (nsteps_ > 0)
      // (2) use interval in step (if smaller)
      dt = std::min(dt, steps_[0].until-t);
    return dt;
  }

  /** Log the next plan step. */
  void log(void)
  {
    if (nsteps_ > 0)
      ROS_DEBUG("next step: accelerate %.f until %.6f",
                steps_[0].accel, steps_[0].until);
  }

  /** Advance to next Plan Step, if it is now time.

      @returns next acceleration to use (0.0 if plan finished).
   */
  double next(double now)
  {
    // advance to next plan step, if time
    while ((nsteps_ > 0) && (steps_[0].until <= now))
      {
        steps_[0] = steps_[1];          // remove the first step
        --nsteps_;
      }

    // return acceleration to use
    if (nsteps_ > 0)
      return steps_[0].accel;
    else
      return 0.0;
  }

  /** Delete the current plan. */
  void reset(void) {nsteps_ = 0;}

private:
  int nsteps_;                          ///< number of steps remaining
  static const int MAX_STEPS = 2;

  struct {
    double accel;                       ///< acceleration to apply
    double until;                       ///< time when done
  } steps_[MAX_STEPS];
};

class ArtBrakeModel
{
public:
    
  ArtBrakeModel(float init_pos);
  ~ArtBrakeModel();

  /** Interpret actuator command and return response. */
  int  interpret(const char *string, char *status, int nbytes);
	
private:

  // ROS interfaces
  ros::NodeHandle node_;        // simulation node handle

  double last_update_time_;

  Animatics *am_;               // Animatics Smart Motor data

  float  brake_position_;       // simulated brake position
  int    sim_status_;           // simulated status word
  int    sim_encoder_;          // simulated encoder value
  int    encoder_goal_;         // encoder set point
  int    digital_pressure_;     // simulated pressure A/D value
  int    digital_potentiometer_;// simulated pot A/D value

  // configured actuator parameters
  double actuator_accel_;       // configured acceleration (ticks/s^2)
  double actuator_max_vel_;     // configured maximum velocity (ticks/s)

  // actuator state variables
  double x_;                    // current actuator position (ticks)
  double v_;                    // current actuator velocity (ticks/s)
  double a_;                    // current actuator acceleration (ticks/s^2)

  ActuatorPlan plan_;           // actuator movement plan


  /// Get acceleration constant from direction of movement (-, 0, +).
  inline double getAccel(double direction)
  {
    return (direction == 0.0? 0.0 :
            (direction > 0.0? actuator_accel_ : -actuator_accel_));
  }

  void move(double start, double finish);
  double plan(double dx);
  void update(void);
  void update_sensors(float position);

};

#endif // _MODEL_BRAKE_H_
