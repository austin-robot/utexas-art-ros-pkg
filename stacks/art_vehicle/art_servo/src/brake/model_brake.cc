/*
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file
 
     Model ART autonomous vehicle brake actuator.

     \author Jack O'Quin

 */

#include <errno.h>
#include <stdio.h>

#include <art/conversions.h>
#include "model_brake.h"

namespace
{
  static double EPSILON_SECONDS = 0.000001;
  static double EPSILON_TICKS = 0.5;
}

/** Constructor */
ArtBrakeModel::ArtBrakeModel(float init_pos)
{
  am_ = new Animatics;

  // use private node handle to get parameters
  ros::NodeHandle mynh("~");

  // configure actuator velocity and acceleration
  double accel_limit_in, max_vel_in;
  mynh.param("actuator_accel", accel_limit_in, 8.0);
  ROS_INFO("model actuator acceleration as %.2f in/s^2", accel_limit_in);
  mynh.param("actuator_max_vel", max_vel_in, 4.0);
  ROS_INFO("model max actuator velocity as %.2f in/s", max_vel_in);

  // convert configured values from inches to ticks
  double ticks_per_inch = 24000;
  actuator_accel_ = rint(accel_limit_in * ticks_per_inch);
  actuator_max_vel_ = rint(max_vel_in * ticks_per_inch);

  // initialize simulation data
  sim_status_ = 0;
  brake_position_ = init_pos;
  sim_encoder_ = encoder_goal_ = am_->pos2enc(brake_position_);
  x_ = (double) sim_encoder_;
  a_ = 0;
  v_ = 0;
  update_sensors(brake_position_);
  last_update_time_ = ros::Time::now().toSec();
}

ArtBrakeModel::~ArtBrakeModel()
{
  delete am_;
}

/** interpret brake command and return response
 *
 *  Only certain commands require interpretation.  All others are just
 *  accepted, acknowledged, and ignored.
 */
int ArtBrakeModel::interpret(const char *string, char *status, int nbytes)
{
  // preset empty status response
  if (nbytes > 0)
    status[0] = '\0';

  // avoid problems with empty command
  if (string == NULL)
    return EIO;

  // Bring the simulation up to date
  update();

  int retval = 0;                       // return value

  // Much simpler than a real parser: branch on first character.
  switch (string[0])
    {
    case 'c':
      if (0 == strcmp(string, "c=UEA Rc\n"))
        {
          // read potentiometer A/D value
          snprintf(status, nbytes, "%d\n", digital_potentiometer_);
        }
      else if (0 == strcmp(string, "c=UAA Rc\n"))
        {
          // read pressure A/D values
          snprintf(status, nbytes, "%d\n", digital_pressure_);
        }
      break;

    case 'D':
      // start relative actuator motion, read status word
      int encoder_delta;
      if (1 == sscanf(string, "D=%d RW G\n", &encoder_delta))
        {
          // TODO: force update() before changing goal or accel
          encoder_goal_ = am_->clamp_encoder(sim_encoder_ + encoder_delta);

          // Plan actuator relative move.
          plan(encoder_delta);

          ROS_DEBUG("simulated encoder delta: %d, goal: %d",
                    encoder_delta, encoder_goal_);
          sim_status_ |= Status_Bt;     // set trajectory busy
          snprintf(status, nbytes, "%d\n", sim_status_);
        }
      else
        retval = EIO;                   // parse failure
      break;

    case 'R':
      if (0 == strcmp(string, "RP\n"))
        {
          // read encoder position value
          snprintf(status, nbytes, "%d\n", sim_encoder_);
        }
      break;

    case 'Z':
      if (0 == strcmp(string, "ZS RW\n"))
        {
          // reset status word
          sim_status_ = 0;
          snprintf(status, nbytes, "%d\n", sim_status_);
        }
      break;

    default:
      // return "success" with empty status
      ;
    }

  return retval;
}

/** Move simulated actuator.

    @param start time for simulation
    @param finish time. 

    The Animatics Smart Motor provides a constant acceleration and
    deceleration up to a maximum velocity.
 */
void ArtBrakeModel::move(double start, double finish)
{
  double t = start;
  while (t < finish)
    {
      a_ = plan_.next(t);               // get next acceleration
      double dt = plan_.interval(t, finish);
      double v0 = v_;                   // previous velocity
      v_ += dt * a_;                    // velocity change
      x_ += dt * (v_ + v0)/2.0;         // advance by average velocity
      t += dt;                          // advance simulated time
#if 0
      // Warn if encoder output outside configured range (not really
      // an error, it can happen with the real device).
      if ((x_ < am_->encoder_min_ - EPSILON_TICKS)
          || (x_ > am_->encoder_max_ + EPSILON_TICKS))
        ROS_WARN("actuator out of range: (time %.6f): %.f, %.f, %.f",
                 t, x_, v_, a_);
#endif
      // Clamp encoder output to configured range (probably more
      // realistic not to do this).
      if (x_ < am_->encoder_min_ - EPSILON_TICKS)
        x_ = am_->encoder_min_;
      else if (x_ > am_->encoder_max_ + EPSILON_TICKS)
        x_ = am_->encoder_max_;
      ROS_DEBUG("actuator (time %.6f): %.f, %.f, %.f (%.6f)",
                t, x_, v_, a_, dt);
    }

  if (fabs(finish-t) >= EPSILON_SECONDS)
    ROS_WARN("ArtBrakeModel::move() finished early (%.6f < %.6f)", t, finish);
}

/** Plan actuator relative move.

    @param dx = encoder ticks to move.
    @returns time (in seconds) required to get there.
    
    This calculation assumes a triangular velocity profile with
    constant acceleration |a|, applied first in one direction,
    then in the other.

    Erase any previous plan, then move the actuator in two steps:

      (1) accelerate in direction requested;
      (2) decelerate to zero velocity
    
    The real device supports a trapezoidal velocity profile with a
    maximum velocity (actuator_max_vel_).  Since it takes a long time
    to reach actuator_max_vel_, this model ignores that detail, which
    does not seem to matter much in practice.
*/
double ArtBrakeModel::plan(double dx)
{
  double t0 = ros::Time::now().toSec();
  double a;                             // required actuator acceleration

  if (fabs(dx) > EPSILON_TICKS)         // nonzero move?
    a = getAccel(dx);                   // accelerate in new direction
  else if (fabs(v_) > EPSILON_TICKS)    // current velocity nonzero?
    a = getAccel(-v_);                  // cancel current velocity
  else                                  // holding in place
    {
      plan_.reset();                    // delete any existing plan
      return 0.0;
    }

  double dt_h = -v_/a;                  // time to halt current movement
  double dx_h = dx - 0.5 * v_ * dt_h;   // dx remaining after halt
  double dt2 = sqrt(dx_h/a);            // deceleration time
  double dt1 = dt2 + dt_h;              // time moving towards goal
  double dt = dt1 + dt2;                // total move duration

  // make a new plan
  plan_.reset();                        // delete current plan
  if (dt1 > EPSILON_SECONDS)
    plan_.append(t0 + dt1, a);
  if (dt2 > EPSILON_SECONDS)
    plan_.append(t0 + dt, -a);

  // log some debug information
  plan_.log();
  ROS_DEBUG("plan(%.f) time %.6f, dt: %.6f, dt1: %.6f, dt2: %.6f",
            dx, t0, dt, dt1, dt2);

  return dt;
}

/** update brake actuator model for start of cycle */
void ArtBrakeModel::update(void)
{
  double now = ros::Time::now().toSec();

  // model actuator movement since last update
#if 1
  move(last_update_time_, now);
  sim_encoder_ = rint(x_);
#else
  sim_encoder_ = encoder_goal_;         // arm movement finished
#endif
  if (plan_.finished())                 // finished moving?
    sim_status_ &= (~Status_Bt);        // reset trajectory busy

  // compute fractional position from encoder value
  brake_position_ = limit_travel(am_->enc2pos(sim_encoder_));

  if (sim_encoder_ <= am_->encoder_min_)
    {
      sim_status_ |= (Status_Bm | Status_Bl);
      ROS_DEBUG("-limit reached, status: 0x%.02x", sim_status_);
    }
  else
    sim_status_ &= (~Status_Bm);

  if (sim_encoder_ >= am_->encoder_max_)
    {
      sim_status_ |= (Status_Bp | Status_Br);
      ROS_DEBUG("+limit reached, status: 0x%.02x", sim_status_);
    }
  else
    sim_status_ &= (~Status_Bp);

  update_sensors(brake_position_);

  ROS_DEBUG("Brake model: 0x%02x [%.3f, %d, %d, %d] %.6f",
            sim_status_, brake_position_, digital_potentiometer_,
            digital_pressure_, sim_encoder_, now);

  last_update_time_ = now;
}

void ArtBrakeModel::update_sensors(float position)
{
  digital_pressure_ =
    analog_to_digital(am_->pos2press(position), 5.0, 10); 
  digital_potentiometer_ =
    analog_to_digital(am_->pos2pot(position), 5.0, 10);
}
