/*
 *  ART steering servo controller device interface
 *
 *  Copyright (C) 2008 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */


/** @{ */
/** @defgroup driver_steering steering servo driver
 @ingroup driver_hardware
 @brief ART steering servo driver

Configuration options for steering servo device:

- port (string)
  - tty port name for the steering servo
  - default: "/dev/null"

- center_on_exit (integer)
  - unless 0, center steering during shutdown.
  - default: 1

- training (integer)
  - if not 0, log steering data, but do not send it any commands
  - default: 0

- steering_rate (float)
  - rate at which steering wheel turns (degrees/sec)
  - default: 14.5

Training mode collects data while a human driver operates the vehicle.
*/
/** @} */


#ifndef _DEVSTEER_HH_
#define _DEVSTEER_HH_

#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <art/conversions.h>		// A/D conversion
#include <art_msgs/ArtVehicle.h>

#include "../servo.h"

#define DEVICE "Quicksilver"

// QuickSilver Stepper (31 degrees at 310,000 ticks)
//
// The vehicle's steering linkage has an actual range of plus or minus
// 31 degrees.  We limit our requests to plus or minus 29 degrees, to
// prevent mechanical damage.
//
// theoretical value: (-10000.0)
//#define TICKS_PER_DEGREE ((float)(-310000.0 / 31.0))
//
// more exact value determined by curve fit of measurements:
#define TICKS_PER_DEGREE ((float) -9841.65332031)

#define DBG(format,args...) ROS_DEBUG(format, ## args)

// One per steering hardware device.
class devsteer: Servo
{
public:

  devsteer();
  ~devsteer() {};

  int	Open();
  int	Close();
  int	Configure();

  // Quicksilver command methods
  int	get_angle(float &degrees);
  int	get_encoder(float &ticks);
  int	get_encoder(long &iticks);
  int	set_initial_angle(float position);
  int	steering_absolute(float position);
  int	steering_relative(float position);

  bool  simulate_;

 private:

  int64_t GetTime();

  // configuration options
  bool	center_on_exit;			// center steering during shutdown()
  std::string port;			// tty port name
  bool	training;			// use training mode
  double steering_rate;			// steering velocity (deg/sec)

  int	verbose;			// log output verbosity
  float	req_angle;			// requested angle (absolute)
  float	starting_angle;			// starting wheel angle
  long	starting_ticks;			// starting wheel encoder ticks
  long	center_ticks;			// center wheel encoder ticks

  int	configure_steering(void);
  int	encoder_goto(float degrees);
  int	send_cmd(const char *string);
  int	servo_cmd(const char *string);
  int	write_register(int reg, long val);

  // convert steering angle to encoder ticks
  inline long degrees2ticks(float degrees)
  {
    return (long) lrint(degrees * TICKS_PER_DEGREE) + center_ticks;
  }

};

// limit position value to normal range
static inline float limit_travel(float position)
{
  if (position > art_msgs::ArtVehicle::max_steer_degrees)
    position = art_msgs::ArtVehicle::max_steer_degrees;
  else if (position < -art_msgs::ArtVehicle::max_steer_degrees)
    position = -art_msgs::ArtVehicle::max_steer_degrees;
  return position;
}

#endif // _DEVSTEER_HH_
