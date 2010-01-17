/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file 

    ART brake servo controller device interface.

    \author Jack O'Quin
 */

#ifndef _DEVBRAKE_H_
#define _DEVBRAKE_H_

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "../servo.h"
#include "model_brake.h"

// One per brake hardware device.
class devbrake: Servo
{
public:

  devbrake(bool train);
  ~devbrake();

  int	Open(const char *port_name);
  int	Close();

  // brake command methods
  int	brake_absolute(float position);
  int	brake_relative(float position);

  // accessor method for current position
  float	get_position(void) {return cur_position;}

  // read the primary hardware sensor status
  int	get_state(float *position, float *potentiometer,
                  float *encoder, float *pressure);

  // These public data are initialized by brake.cc before calling
  // setup(), so initial values can be set from the .cfg file.
  // Afterwards, the devbrake class updates them whenever it detects
  // calibration changes.

  // Configuration options:

  // potentiometer limits, normally pot_off > pot_full
  double pot_off;
  double pot_full;
  double pot_range;			// for calculating fractions

  // pressure sensor limits
  double pressure_min;
  double pressure_max;
  double pressure_range;                // for calculating fractions

  // brake motor encoder limits
  double encoder_min;
  double encoder_max;			// determined by setup()
  double encoder_range;			// for calculating fractions

 private:

  // configuration options:
  bool	 training;			// use training mode
  bool   apply_on_exit;			// apply brake during shutdown()
  double deceleration_threshold;
  double deceleration_limit;
  double pressure_filter_gain;
  bool   use_pressure;

  // current brake status
  bool	already_configured;		// brake already configured once
  brake_status_t cur_status;		// last status from servo_cmd()
  float	cur_pot;			// last potentiometer voltage
  float	cur_pressure;			// last brake pressure read
  float	prev_pressure;			// last brake pressure read
  float	cur_encoder;			// last encoder value read

  float	cur_position;			// last position read

  ArtBrakeModel *sim;                   // brake simulation model

  // pointer to any of the private query_* methods
  typedef int (devbrake::*query_method_t)(float *);

  // query methods
  int	query_amps(float *data);
  int	query_encoder(float *data);
  int	query_pot(float *data);
  int	query_pressure(float *data);
  int	query_volts(float *data);

  int	calibrate_brake(void);
  void	check_encoder_limits(void);
  int	configure_brake(void);
  int	encoder_goto(int enc_delta);
  int	query_cmd(const char *string, char *status, int nbytes);
  int	read_stable_value(query_method_t query_method,
			  double *status, float epsilon);
  int	servo_cmd(const char *string);
  void	servo_write_only(const char *string);

  // Convert encoder readings to and from float positions.
  inline float enc2pos(int encoder_val)
    {return ((encoder_val - encoder_min) / encoder_range);}
  inline int   pos2enc(float position)  /* simulation */
    {return (int) rintf(encoder_min + position * encoder_range);}

  // Convert potentiometer voltages to and from float positions.
  // Warning! pot_range is typically negative in these calculations.
  inline float pot2pos(float pot_volts)
    {return ((pot_volts - pot_off) / pot_range);}
  inline float pos2pot(float position)  /* simulation */
    {return position * pot_range + pot_off;}

  // Convert pressure sensor voltages to and from float positions.
  inline float press2pos(float pressure_volts)
    {return ((pressure_volts - pressure_min) / pressure_range);}
  inline float pos2press(float position) /* simulation */
    {return pressure_min + position * pressure_range;}
};

#if 0
// limit position value to normal range
static inline float limit_travel(float position)
{
  if (position > 1.0)	position = 1.0;
  else if (position < 0.0)	position = 0.0;
  return position;
}
#endif

#endif // _DEVBRAKE_H_
