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
#include <art_msgs/SteeringDiagnostics.h>

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
#define TICKS_PER_DEGREE ((double) -9841.65332031)

#define DBG(format,args...) ROS_DEBUG(format, ## args)

// One per steering hardware device.
class devsteer: Servo
{
public:

  devsteer(int32_t center=0);
  ~devsteer() {};

  int	Open();
  int	Close();
  int	Configure();

  // Quicksilver command methods
  int	check_status(void);
  int	get_angle(float &degrees);
  void  publish_diag(const ros::Publisher &diag_pub);
  int	set_initial_angle(float position);
  int	steering_absolute(float position);
  int	steering_relative(float position);

  // convert steering angle to encoder ticks
  inline int32_t degrees2ticks(float degrees)
  {
    return (int32_t) lrint(degrees * TICKS_PER_DEGREE) + center_ticks_;
  }

  // convert encoder ticks to steering angle
  inline float ticks2degrees(int32_t ticks)
  {
    return (ticks - center_ticks_) / TICKS_PER_DEGREE;
  }

 private:

  int64_t GetTime();

  // configuration options
  bool	center_on_exit_;          // center steering during shutdown()
  std::string port_;              // tty port name
  bool	training_;                // use training mode
  bool	simulate_moving_errors_;  // simulate intermittent moving errors
  double steering_rate_;          // steering velocity (deg/sec)

  float	req_angle_;                    // requested angle (absolute)
  float	starting_angle_;               // starting wheel angle
  int32_t starting_ticks_;             // starting wheel encoder ticks
  int32_t center_ticks_;               // center wheel encoder ticks

  art_msgs::SteeringDiagnostics diag_msg_;

  int	configure_steering(void);
  int	encoder_goto(float degrees);
  int	get_encoder(int32_t &iticks);
  int	get_status_word(uint16_t &status);
  int	send_cmd(const char *string);
  int	servo_cmd(const char *string);
  void	servo_write_only(const char *string);
  int	write_register(int reg, int32_t val);

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
