/*
 *  ART throttle servo controller device interface
 *
 *  Copyright (C) 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

#ifndef _DEVTHROTTLE_H_
#define _DEVTHROTTLE_H_

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <ros/ros.h>

#include "../servo.h"
#include "avr_controller.h"

#define DBG(format,args...) ROS_DEBUG(format, ## args)

// One per throttle hardware device.
class devthrottle: Servo
{
public:

  /* These public data are initialized by throttle.cc before calling
   * Open(), so initial values can be set from the .cfg file.
   */
  double rpm_redline;                   /* engine rev limit (RPM) */
  double throttle_limit;                /* throttle position limit (%) */
  int avr_kp;				/* AVR proportional PID */
  int avr_ki;				/* AVR integral PID */
  int avr_kd;				/* AVR derivative PID */
  int avr_out_max;			/* AVR max PWM output */

  /* Actual sensor limit positions are determined during Open(),
   * having been set to reasonable initial values by .cfg.
   */
  double avr_pos_max;
  int    avr_pos_min;
  double avr_pos_range;
  int    avr_pos_epsilon;		/* trivial difference value */

  devthrottle(bool train);
  ~devthrottle() {};

  int Open(const char *device);
  int Close(void);

  int64_t GetTime();

  // accessor method for current position
  float	get_position(void) {return cur_position;}

  // query methods for current status
  bool	query_estop(void);
  int	query_pid(float *pwm, float *dstate, float *istate);
  int	query_rpms(float *data);
  int	query_status();

  // throttle command methods
  int	throttle_absolute(float position);
  int	throttle_relative(float delta);

 private:

  bool	training;			// use training mode
  bool	already_configured;		// throttle already configured once

  struct avr_cmd stat;			/* latest throttle status */
  struct avr_cmd cmd;			/* outgoing AVR command */
  struct avr_cmd resp;			/* AVR command response */

  /* byte pointers to these structures */
  uint8_t *cmd_p;
  uint8_t *resp_p;

  float	cur_position;			// last position read
  float	last_req;			// last position requested

  int	  calibrate_idle(void);
  uint8_t cmd_compute_csum(uint8_t *buffer, int len);
  int	  configure_controller(void);
  void	  decode_char(char c, int *resp_bytes, int *resp_digits);
  int	  format_cmd(char *cmdstr);
  int	  read_byte(int linelen);
  int     send_cmd(int ccode);
  int     send_cmd08(int ccode, uint8_t data);
  int     send_cmd16(int ccode, uint16_t data);
  int     send_cmd32(int ccode, uint32_t data);
  int	  send_goto(uint8_t pos);
  int     servo_cmd(void);
  int	  validate_response(int resp_bytes, int linelen);

  // return position valued constrained to permitted range
  inline float limit_travel(float position)
    {
      if (position > throttle_limit)	position = throttle_limit;
      else if (position < 0.0)		position = 0.0;
      return position;
    }

  // Conversions between throttle percentages and controller values.
  inline uint8_t pos2avr(float position)
    {return (int) rintf(avr_pos_min + position * avr_pos_range);}

  inline float avr2pos(uint8_t avr_val)
    {return (avr_val - avr_pos_min) / avr_pos_range;}
};

#endif // _DEVTHROTTLE_H_
