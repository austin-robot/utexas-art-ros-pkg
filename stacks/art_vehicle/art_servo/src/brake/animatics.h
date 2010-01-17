/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file 

    ART brake servo controller Animatics Smart Motor interface

    \author Jack O'Quin
 */

#ifndef _ANIMATICS_H_
#define _ANIMATICS_H_

#include <stdint.h>

#include <ros/ros.h>

#include "../servo.h"
#include "model_brake.h"

// Animatics Smart Motor Status word bit definitions:
typedef enum
  {
    Status_Bt	=	0x0001,		// busy trajectory
    Status_Br	=	0x0002,		// historical right (+, on) limit
    Status_Bl	=	0x0004,		// historical left (-, off) limit
    Status_Bi	=	0x0008,		// index report available
    Status_Bw	=	0x0010,		// wraparound occurred
    Status_Be	=	0x0020,		// excessive position error
    Status_Bh	=	0x0040,		// excessive temperature
    Status_Bo	=	0x0080,		// motor is off
    Status_Bx	=	0x0100,		// hardware index input asserted
    Status_Bp	=	0x0200,		// right (+, on) limit asserted
    Status_Bm	=	0x0400,		// left (-, off) limit asserted
    Status_Bd	=	0x0800,		// user math overflow
    Status_Bu	=	0x1000,		// user array index range error
    Status_Bs	=	0x2000,		// syntax error occurred
    Status_Ba	=	0x4000,		// over current state occurred
    Status_Bk	=	0x8000		// user program check sum error
  } brake_status_t;

// One per brake hardware device.
class Animatics
{
public:

  Animatics()
    {
      // Set brake parameters -- make sure the defaults won't strip
      // the servo hardware gears.  These values will be used for
      // /dev/null, in training mode, or in case of failure.  They may
      // be updated by devbrake when it detects calibration changes.

      ros::NodeHandle node("~");

      node.param("encoder_min", encoder_min_, 0);
      node.param("encoder_max", encoder_max_, 50000);
      encoder_range_ = encoder_max_ - encoder_min_;
      ROS_INFO("Animatics encoder range [%d, %d]", encoder_min_, encoder_max_);

      node.param("pot_off", pot_off_, 4.9);
      node.param("pot_full", pot_full_, 0.49);
      pot_range_ = pot_full_ - pot_off_;
      ROS_INFO("Animatics potentiometer range [%.3f, %.3f]",
               pot_full_, pot_off_);

      node.param("pressure_min", pressure_min_, 0.85);
      node.param("pressure_max", pressure_max_, 4.5);
      pressure_range_ = pressure_max_ - pressure_min_;
      ROS_INFO("Animatics pressure range [%.3f, %.3f]",
               pressure_min_, pressure_max_);
    }
  ~Animatics() {};

  // Configuration options:

  // potentiometer limits, normally pot_off_ > pot_full_
  double pot_off_;
  double pot_full_;
  double pot_range_;

  // pressure sensor limits
  double pressure_min_;
  double pressure_max_;
  double pressure_range_;

  // brake motor encoder limits
  int encoder_min_;
  int encoder_max_;
  double encoder_range_;

  // Convert encoder readings to and from float positions.
  inline float enc2pos(int encoder_val)
    {return ((encoder_val - encoder_min_) / encoder_range_);}
  inline int   pos2enc(float position)  /* simulation */
    {return (int) rintf(encoder_min_ + position * encoder_range_);}

  // Clamp encoder value to range.
  inline int clamp_encoder(int value)
  {
    if (value > encoder_max_)           value = encoder_max_;
    else if (value < encoder_min_)	value = encoder_min_;
    return value;
  }

  // Convert potentiometer voltages to and from float positions.
  // Warning! pot_range is typically negative in these calculations.
  inline float pot2pos(float pot_volts)
    {return ((pot_volts - pot_off_) / pot_range_);}
  inline float pos2pot(float position)  /* simulation */
    {return position * pot_range_ + pot_off_;}

  // Convert pressure sensor voltages to and from float positions.
  inline float press2pos(float pressure_volts)
    {return ((pressure_volts - pressure_min_) / pressure_range_);}
  inline float pos2press(float position) /* simulation */
    {return pressure_min_ + position * pressure_range_;}
};

// limit position value to normal range
static inline float limit_travel(float position)
{
  if (position > 1.0)	position = 1.0;
  else if (position < 0.0)	position = 0.0;
  return position;
}

#endif // _ANIMATICS_H_
