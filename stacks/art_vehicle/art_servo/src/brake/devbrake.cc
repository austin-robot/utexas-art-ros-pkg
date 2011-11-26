/*
 *  ART brake servo controller device interface
 *
 *  Copyright (C) 2005, 2009 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>

#include <ros/ros.h>

#include <art/conversions.h>

#include "devbrake.h"


/////////////////////////////////////////////////////////////////
// public methods
/////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Get the time (in ms)
//
int64_t GetTime()
{
  ros::Time tv = ros::Time::now();
  return (int64_t) tv.sec * 1000 + (int64_t) tv.nsec / 1000000;
}



devbrake::devbrake(bool train)
{
  training = train;
  already_configured = false;
  cur_position = 0.0;
  prev_pressure = 0.0;
  sim = NULL;

  // use private node handle to get parameters
  ros::NodeHandle mynh("~");
  mynh.param("apply_on_exit", apply_on_exit, false);

  mynh.param("pressure_filter_gain", pressure_filter_gain, 0.4);
  ROS_INFO("brake pressure RC filter gain is %.3f", pressure_filter_gain);

  mynh.param("use_pressure", use_pressure, true);
  ROS_INFO("use %s sensor to control brake",
	  (use_pressure? "pressure": "position"));

  // A deceleration_limit of 0.025 corresponds to approximately
  // 20Hz * 2250 (=0.025*90000) tics/cycle = 45,000 tics/sec.
  //
  // Assuming a typical encoder range of 0-90,000 tics and a 
  // threshold of 0.7 (= 63,000 tics), 0.6 seconds are
  // required for the final 30% of brake travel for full brake.
  //
  // At least one brake motor has failed repeatedly at limit=0.04
  // and once out of 100 attempts at limit=0.015. Replaced motor.

  mynh.param("deceleration_threshold", deceleration_threshold, 0.7);
  mynh.param("deceleration_limit", deceleration_limit, 0.025);
  ROS_INFO("brake deceleration threshold, limit is [%.3f, %.3f]",
           deceleration_threshold, deceleration_limit);
}

devbrake::~devbrake()
{
  if (sim)
    delete sim;
};

int devbrake::Open(const char *port_name)
{
  // retry count in case calibration fails
  int retries = 7;

  // open the serial port
  int rc = this->Servo::Open(port_name, (O_RDWR|O_NOCTTY|O_NONBLOCK));
  if (fd < 0) {
    ROS_FATAL("Couldn't open %s (%s)", port_name, strerror(errno));
    return -1;
  }  

  // set initial baud rate
  rc = configure_raw_port(B9600 | CS8, 0);
  if (rc != 0) goto fail;
    
  // tell device to run at 38400; command does not and cannot respond
  servo_write_only("BAUD38400\n");
  sleep(3);
    
  // set actual baud rate
  rc = configure_raw_port(B38400 | CS8, 0);
  if (rc != 0) goto fail;

  // Initialize brake simulation before calibration.
  if (!have_tty)
    sim = new ArtBrakeModel(cur_position);

  // No need to configure or calibrate brake if already done.
  // Must avoid touching the brake when in training mode.
  while (!already_configured && !training)
    {
      rc = configure_brake();
      if (rc != 0) goto fail;

      // For some reason (probably clutch slippage) calibrate brake
      // sometimes fails, yet retrying may succeed.
      rc = calibrate_brake();
      if (rc == 0)
	already_configured = true;
      else if (--retries <= 0)
	goto fail;
    }

  // set brake on unless in training mode
  if (!training)
    rc = brake_absolute(1.0);		// set brake fully on

  return rc;

 fail:
  this->Servo::Close();
  return rc;				// Open() failed
}

int devbrake::Close()
{
  if (apply_on_exit && !training)
    {
      ROS_INFO("setting brake fully on for shutdown");
      brake_absolute(1.0);		// set brake fully on
      sleep(1);                         // give it a second to work
    }
  return this->Servo::Close();
}

int devbrake::brake_absolute(float position)
{
  ROS_DEBUG("brake_absolute(%.3f)", position);
  return brake_relative(position - cur_position);
}
 
int devbrake::brake_relative(float delta)
{
  ROS_DEBUG("brake_relative(%.3f)", delta);

  float	new_pos = limit_travel(cur_position + delta);
  if ((delta > 0.0) && (new_pos > deceleration_threshold))
    {
      // limit rate of deceleration, near full brake, to minimize
      // brake overcurrent errors
      bool limiter_engaged = true;
      if (cur_position < deceleration_threshold)
        // threshold crossing. Take minimum of new_pos and (threshold
        // + limit)
        new_pos = limit_travel(fminf(new_pos,
                                     deceleration_threshold
                                     + deceleration_limit));
      else if (delta > deceleration_limit)
        // both new and old position beyond threshold AND requested
        // delta greater than limit. Use limit
        new_pos = limit_travel(cur_position + deceleration_limit);
      else
        limiter_engaged = false;

      if (limiter_engaged)
        ROS_DEBUG("deceleration limiter engaged. cur_position: %.3f "
                  "requested delta: %.3f actual delta: %.3f",
                  cur_position, delta, new_pos-cur_position);
    }

  ROS_DEBUG("changing brake position to %.3f", new_pos);
  return encoder_goto((int) rintf((new_pos-cur_position)*encoder_range));
}

/// Get current values of all the important hardware sensors.
//
//  Called once per cycle before other commands.
//
//  \returns 0, if all sensors are read successfully;
//           errno value of last sensor that returned an error otherwise.
int devbrake::get_state(float *position, float *potentiometer,
                        float *encoder, float *pressure)
{
  // Read the primary hardware sensors, set rc to either zero or the
  // last error return code.  The query methods only modify their data
  // parameter if successful.
  int rc = query_encoder(encoder);
  int qrc = query_pot(potentiometer);
  if (qrc) rc = qrc;
  qrc = query_pressure(pressure);
  if (qrc) rc = qrc;

  // cur_position was set as a side-effect of the queries above
  *position = cur_position;

  return rc;
}

int devbrake::query_amps(float *data)
{
  char string[MAX_SERVO_CMD_BUFFER];

  int rc = query_cmd("c=UIA Rc\n", string, MAX_SERVO_CMD_BUFFER);
  if (rc == 0)			// only update if successful
    {
      *data = strtof(string, NULL);
      // TODO: convert *data to a voltage
      ROS_DEBUG("query amperage returns %.1f", *data);
    }

  return rc;
}

int devbrake::query_encoder(float *data)
{
  char string[MAX_SERVO_CMD_BUFFER];

  int rc = query_cmd("RP\n", string, MAX_SERVO_CMD_BUFFER);
  if (rc == 0)			// only update if successful
    {
      *data = cur_encoder = strtof(string, NULL);
      check_encoder_limits();		// adjust limits if motor slipped
      ROS_DEBUG("query encoder returns %.1f", *data);
    }

  return rc;
}

int devbrake::query_pot(float *data)
{
  char string[MAX_SERVO_CMD_BUFFER];

  int rc = query_cmd("c=UEA Rc\n", string, MAX_SERVO_CMD_BUFFER);
  if (rc == 0)				// only update if successful
    {
      // A/D value of potentiometer
      float pot_val = strtof(string, NULL);

      // convert A/D input to voltage (10-bit converter with 5-volt range)
      *data = cur_pot = analog_volts(pot_val, 5.0, 10);
      ROS_DEBUG("brake potentiometer voltage is %.3f (A/D %.f)",
                cur_pot, pot_val);

      if (!use_pressure && already_configured)
	// use potentiometer to estimate current position
	cur_position = limit_travel(pot2pos(cur_pot));
    }

  return rc;
}

// RC filter transfer function: y[k] = (1-a)*x[k] + a*y[k-1]
static inline float RC_filter(float a, float xk, float yk1)
{
  return (1-a)*xk + a*yk1;
}

int devbrake::query_pressure(float *data)
{
  char string[MAX_SERVO_CMD_BUFFER];

  int rc = query_cmd("c=UAA Rc\n", string, MAX_SERVO_CMD_BUFFER);
  if (rc == 0)			// only update if successful
    {
      // A/D value of pressure sensor
      float pressure_val = strtof(string, NULL);

      // convert A/D input to voltage (10-bit converter with 5-volt range)
      cur_pressure = analog_volts(pressure_val, 5.0, 10);

      // smooth the data using an RC low-pass filter to eliminate
      // small fluctuations.
      cur_pressure = RC_filter(pressure_filter_gain,
			       cur_pressure, prev_pressure);
      ROS_DEBUG("RC filter output = %.3f, previous = %.3f",
                cur_pressure, prev_pressure);

      ROS_DEBUG("brake pressure voltage is %.3f (A/D %.f)",
                cur_pressure, pressure_val);
      *data = cur_pressure;		// return result to caller

      if (use_pressure && already_configured)
	// use pressure to estimate current position
	cur_position = limit_travel(press2pos(cur_pressure));

      prev_pressure = cur_pressure;
    }

  return rc;
}

int devbrake::query_volts(float *data)
{
  char string[MAX_SERVO_CMD_BUFFER];

  int rc = query_cmd("c=UJA Rc\n", string, MAX_SERVO_CMD_BUFFER);
  if (rc == 0)			// only update if successful
    {
      *data = strtof(string, NULL);
      // TODO: convert *data to a voltage
      ROS_DEBUG("query voltage returns %.3f", *data);
    }

  return rc;
}


/////////////////////////////////////////////////////////////////
// private methods
/////////////////////////////////////////////////////////////////


#define ENC_EPSILON 2			// trivial position difference
#define POT_EPSILON 0.002		// trivial potentiometer difference
#define PRESS_EPSILON 0.005		// trivial pressure difference

// calibrate brake position limits
//
//  On entry, brake is off; on exit fully engaged.
//
//  Use potentiometer to estimate position, determining experimentally
//  the achievable full brake limit.  Make this result available to
//  translate potentiometer values to fractional position information.
//
// returns: 0 if successful, updates limits and ranges for encoder,
//			     potentiometer, and pressure
//          nonzero if failure, causing the setup() to fail
//
//   Experiments suggest that the brake is off when the pressure
//   sensor is below 1.001v.  The corresponding potentiometer sensor
//   value is around 3.8v.  Full brake is about 2.64v on the pot
//   (values get smaller as the actuator retracts).  The pressure
//   sensor is quite noisy.  A reasonable curve fit for pressure
//   values above 1 volt as a function of pot voltage is:
//
//      pressure = 0.40480 pot^2 - 3.56259 pot + 8.69659
//
int devbrake::calibrate_brake(void)
{
  int rc;

  rc = read_stable_value(&devbrake::query_pot, &pot_off, POT_EPSILON);
  if (rc != 0) return rc;

  rc = read_stable_value(&devbrake::query_pressure, &pressure_min,
			 PRESS_EPSILON);
  if (rc != 0) return rc;

  // Minimum brake pressure needs to be below 1 volt or both 
  // throttle and brake will be active simultaneously (very bad).
  if (pressure_min > 1.0) {
    ROS_ERROR("Minimum brake pressure too high: %.3f", pressure_min);
    return (1);
  }

  // apply full brake: pull cable for 8 sec, stop if limit reached
  for (int timeout = 8*10; timeout > 0; --timeout)
    {
      rc = encoder_goto(5000);          // relative goto
      if (rc != 0) return rc;
      if (cur_status & Status_Bp)	// +limit reached?
        break;
      usleep(50*1000);                  // wait 0.05 sec
    }

  // log whether or not brake reached +limit within 4 sec.
  if (cur_status & Status_Bp)	// +limit reached?
    ROS_INFO("Good: +limit reached during calibration");
  else
    ROS_WARN("Bad: +limit not reached within 4 sec. calibration");

  rc = read_stable_value(&devbrake::query_encoder, &encoder_max, ENC_EPSILON);
  if (rc != 0) return rc;

  rc = read_stable_value(&devbrake::query_pot, &pot_full, POT_EPSILON);
  if (rc != 0) return rc;

  rc = read_stable_value(&devbrake::query_pressure, &pressure_max,
			 PRESS_EPSILON);
  if (rc != 0) return rc;

  cur_position = 1.0;			// consider this position fully on
  encoder_range = encoder_max - encoder_min;
  pot_range = pot_full - pot_off;	// this will be negative
  pressure_range = pressure_max - pressure_min;

  ROS_INFO("calibrated encoder range is [%.f, %.f]", encoder_min, encoder_max);
  ROS_INFO("calibrated potentiometer range is [%.3f, %.3f]", pot_off, pot_full);
  ROS_INFO("calibrated pressure range is [%.3f, %.3f]",
           pressure_min, pressure_max);

  return 0;
}

// Sometimes the motor slips, forcing us to adjust the encoder limits
// used for braking requests.  Since we assume any out of range value
// is due to slippage, always adjust both max and min whenever either
// is exceeded, leaving the encoder_range the same.
//
// We could request the motor to set a new origin (O=val), but this
// implementation just adjusts the limits used when calculating
// position requests.
inline void devbrake::check_encoder_limits(void)
{
  // Make no adjustments before calibration is complete,
  // calibrate_brake() determines these limits.
  if (already_configured == false)
    return;

  if (cur_encoder > encoder_max)
    {
      encoder_min += cur_encoder - encoder_max;
      encoder_max = cur_encoder;
    }
  else if (cur_encoder < encoder_min)
    {
      encoder_max -= encoder_min - cur_encoder;
      encoder_min = cur_encoder;
    }
  else					// still inside normal range
    return;

  ROS_INFO("Brake motor slipped!  New encoder range is [%.f, %.f]",
           encoder_min, encoder_max);
}

// Configure the brake controller using position mode.
//
// Interesting facts:
//   The brake actuator has a four-inch throw.
//   There is about a two-inch range with the cable connected.
//   The worm gear has a 3:1 ratio.
//   The worm screw advances the actuator 1/4 inch per rev.
//   The motor has 2000 position ticks per revolution.
//   Therefore, the full range is about 96000 ticks.
//   The 10-bit potentiometer on A/D port E has a 5-volt range.
//   The velocity parameter units are rev/sec * 32,212
//   The acceleration parameter units are rev/sec/sec * 7.91
//
// NOTE: the potentiometer is returning high values (near 1000) when
// the brake is off, and low values (near 100) when fully applied.
//
// To remove brake spring use 3/16" allen wrench and 11mm open end wrench.
//
int devbrake::configure_brake(void)
{
  int rc;

  rc = servo_cmd("ZS RW\n");		// reset all status bits
  if (rc != 0) return rc;
    
  rc = servo_cmd("MP D=0 G RW\n");	// ensure position mode
  if (rc != 0) return rc;
    
  rc = servo_cmd("UAI RW\n");		// set Port A as an input
  if (rc != 0) return rc;
    
  rc = servo_cmd("O=0 RW G\n");		// set temporary origin
  if (rc != 0) return rc;
    
  rc = servo_cmd("A=8*96 RW G\n");	// set acceleration: 8 in/sec/sec
  if (rc != 0) return rc;
    
  rc = servo_cmd("V=32212*48 RW G\n");	// set velocity: 4" in 1 sec
  if (rc != 0) return rc;
    
  rc = servo_cmd("LIMD RW\n");		// enable directional limits
  if (rc != 0) return rc;
    
  rc = servo_cmd("LIML RW\n");		// set limits active low
  if (rc != 0) return rc;
    
  rc = servo_cmd("UCP RW\n");		// set pin C to +limit
  if (rc != 0) return rc;
    
  rc = servo_cmd("UDM RW\n");		// set pin D to -limit
  if (rc != 0) return rc;
    
  rc = servo_cmd("F=1 RW\n");		// stop after limit fault
  if (rc != 0) return rc;
    
  // find negative limit position
  for (int timeout = 4*10; timeout > 0; --timeout)
    {
      rc = encoder_goto(-10000);	// relative goto
      if (rc != 0) return rc;
      if (cur_status & Status_Bm)	// -limit reached?
        {
          ROS_INFO("-limit reached during configuration");
          break;
        }
      usleep(100*1000);                 // wait 0.1 sec
    }

  if ((cur_status & Status_Bm) == 0)    // -limit not reached?
    {
      ROS_ERROR("Brake failure: "
                "unable to reach negative limit in 4 seconds.");
      return ENODEV;
    }

  rc = servo_cmd("O=0 RW\n");		// set the origin here
  if (rc != 0) return rc;

  // initialize position values
  encoder_min = 0.0;
  encoder_range = encoder_max - encoder_min;
  cur_position = 0.0;

  return 0;
}

// send encoder position relative displacement command
//
// When the brake clutch slips, encoder values for a given position
// change.  Therefore, we avoid using absolute encoder position
// commands (P=?).  If a positive (full brake) or negative (brake off)
// limit has been reached, recalibrate the encoder limits to agree
// with the physical position switches.
//
// TODO: at some point, the encoder could overflow if it keeps
// slipping.  Resetting the origin would prevent that.
int devbrake::encoder_goto(int enc_delta)
{
  sprintf(buffer, "D=%d RW G\n", enc_delta);
  int rc = servo_cmd(buffer);
  if (rc == 0)
    {
      if ((cur_status & Status_Bp) && (enc_delta >= 0)) // +limit reached?
	{
	  float encoder_val;
	  int qrc = query_encoder(&encoder_val);
	  if (qrc != 0) return rc;
	  if (already_configured)
	    encoder_min += encoder_val - encoder_max;
	  encoder_max = encoder_val;
	  encoder_range = encoder_max - encoder_min;
          ROS_DEBUG("Brake +limit reached, status: 0x%04x, "
                    "encoder range [%.f, %.f]",
                    cur_status, encoder_min, encoder_max);

	  // see if pot_full needs adjustment
	  float pot_val;
	  qrc = query_pot(&pot_val);
	  if (qrc != 0) return rc;
	  if (fabs(pot_val-pot_full) > POT_EPSILON)
	    {
	      pot_full = pot_val;
	      pot_range = pot_full - pot_off; // this will be negative
              ROS_DEBUG("   new potentiometer range is [%.3f, %.3f]",
                        pot_off, pot_full);
	    }
	}
      if ((cur_status & Status_Bm) && (enc_delta <= 0)) // -limit reached?
	{
	  float encoder_val;
	  int qrc = query_encoder(&encoder_val);
	  if (qrc != 0) return rc;
	  if (already_configured)
	    encoder_max += encoder_val - encoder_min;
	  encoder_min = encoder_val;
	  encoder_range = encoder_max - encoder_min;

	  // TODO: reset encoder origin if motor has slipped
	  //if (encoder_min != 0.0) {
	  //  servo_cmd("MP D=0 O=%d RW G\n", (int) rintf(encoder_min));
	  //  encoder_max += encoder_min;
	  //  encoder_min = 0.0;
	  //}

          ROS_DEBUG("Brake -limit reached, status: 0x%04x, "
                    "encoder range [%.f, %.f]",
                    cur_status, encoder_min, encoder_max);

	  // see if pot_off needs adjustment
	  float pot_val;
	  qrc = query_pot(&pot_val);
	  if (qrc != 0) return rc;
	  if (fabs(pot_val-pot_off) > POT_EPSILON)
	    {
	      pot_off = pot_val;
	      pot_range = pot_full - pot_off; // this will be negative
              ROS_DEBUG("   new potentiometer range is [%.3f, %.3f]",
                        pot_off, pot_full);
	    }
	}
    }
  return rc;
}

/*  Query status from the brake controller.
 *
 *   Writes a command, then reads up to nbytes of status from the device.
 *
 *  returns: 0 if successful, errno value otherwise.
 */
int devbrake::query_cmd(const char *string, char *status, int nbytes)
{
  ROS_DEBUG("query_cmd %s", string);
  int len = strlen(string);
  memset(status, 0, nbytes);

  // interpret actuator command and response, if simulated device
  if (sim)
    return sim->interpret(string, status, nbytes);

  int rc=0;
  int attempts = 3;			// number of times to try command
  do
    {
      // Flush the I/O buffers to ensure nothing is left over from any
      // previous command.
      tcflush(fd, TCIOFLUSH);

      // There is not much point in checking for errors on the
      // write().  If something went wrong, we'll find out by reading
      // the device status.
      int res = write(fd, string, len);
      if (res < 0)
        ROS_ERROR_THROTTLE(100, "write() error: %d", errno);

      int timeout = 16;			// set timeout in msecs
      int linelen = 0;

      int64_t start_time = GetTime();
      int64_t stop_time = start_time + timeout;

      while (true)
	{
	  if (timeout >= 0) {
	    struct pollfd fds[1];
	    fds[0].fd=fd;
	    fds[0].events = POLLIN;
	    int64_t delay = stop_time - GetTime();
	    if (delay < 0) delay = 0;
	    int retval = ::poll(fds, 1, delay);
	    if (retval < 0)
	      {
		if (errno == EINTR)
		  continue;
		ROS_ERROR("error returned on poll");
		rc= errno;
		break;
	      }
	    else if (!retval)
	      {
		ROS_INFO("timeout on poll");
		rc= EBUSY;
		break;
	      }
	    else {
	      if ((fds[0].revents & POLLERR) ||
		  (fds[0].revents & POLLHUP) ||
		  (fds[0].revents & POLLNVAL))
		{
		  ROS_ERROR("Device error on poll");
		  rc= EIO;
		  break;
		}
	    }
	  }

	  int bytes = read(fd, status + linelen, 1);
	  if (bytes < 0)
	    {
	      if (errno == EINTR)
		continue;
	      ROS_ERROR("error: %s", strerror(rc));
	      rc = errno;
	      break;			// retry servo command
	    }

	  ROS_DEBUG("read() returns %d, 0x%02X (%c)",
                    bytes, (bytes > 0? status[linelen]: 0),
                    (bytes > 0? status[linelen]: '-'));
	  
	  if (bytes==0)
	    continue;
	  else if (status[linelen] != '\r') // not end of line?
	    {
	      // have a new character
	      if (++linelen >= nbytes)
		{
		  ROS_ERROR("buffer overflow: %s", status);
		  rc = ENOSPC;
		  break;		// retry servo command
		}
	    }
	  else				// have a complete line
	    {
	      return 0;			// success
	    }
	}
    }
  while (--attempts > 0);		// retry, if error
      
  return rc;
}

// read a stable brake sensor value SLOWLY -- use only during setup()
//
//  Note that the potentiometer returns smaller values when the brake
//  is full engaged; the actuator retracts while pulling on the cable.
//
//  Beware that these sensor readings will often increase or decrease
//  after reaching a minimum due to slippage or hitting the limit
//  switch.  The readings will not always change monotonically.
//
//  returns: 0 if successful, with value updated;
//	     errno value otherwise, value unchanged
int devbrake::read_stable_value(query_method_t query_method,
				double *value, float epsilon)
{
  float prev_filter;
  float prev_val;
  int rc = (this->*query_method)(&prev_val);
  if (rc != 0) goto ioerror;
  prev_filter = prev_val;		// initial filter state

  // loop until value settles
  for (int timeout = 4*10; timeout > 0; --timeout)
    {
      usleep(100*1000);			// wait 0.1 sec

      float cur_val;
      rc = (this->*query_method)(&cur_val);
      if (rc != 0) goto ioerror;

      // Experimental: smooth the data using an RC low-pass filter to
      // eliminate small fluctuations.
      float cur_filter = RC_filter(0.5, cur_val, prev_filter);
      ROS_DEBUG("RC filter output = %.3f, previous = %.3f",
                cur_filter, prev_filter);

      // Once in a while, the sensor may return a bogus value.  Only
      // consider cur_val valid when it is within epsilon of the
      // previous reading, indicating the brake has settled to a
      // stable, achievable reading.
#undef USE_FILTER
#ifdef USE_FILTER
      if (fabs(cur_filter - prev_filter) <= epsilon)
#else
      if (fabs(cur_val - prev_val) <= epsilon)
#endif
	{
	  *value = cur_val;
	  return 0;			// success
	}
      prev_val = cur_val;
      prev_filter = cur_filter;
    }

  ROS_ERROR("brake calibration failed! (4 sec. timeout)");
  return EBUSY;

 ioerror:
  ROS_ERROR("brake calibration failed! (%s)", strerror(rc));
  return rc;
}

/* Write a command to the brake controller, log status response.
 *
 *  The command *must* include a status request (RS or RW).
 *
 *  returns: 0 if successful, updating cur_status;
 *	     errno value otherwise.
 */
int devbrake::servo_cmd(const char *string)
{
  char response[MAX_SERVO_CMD_BUFFER];
  int rc;

  // use query_cmd() to read the status response correctly
  rc = query_cmd(string, response, MAX_SERVO_CMD_BUFFER);
  if (rc == 0)
    {
      cur_status = (brake_status_t) atoi(response);
      // check for really bad status like Ba, Be, Bh
      if (cur_status & (Status_Ba|Status_Be|Status_Bh))
        ROS_WARN("bad brake status = %d (0x%02x)", cur_status, cur_status);
      else
        ROS_DEBUG("brake status = %d (0x%02x)", cur_status, cur_status);
    }

  return rc;
}

/* Write a command to the brake controller, no response expected.
 *
 *  The command *must* *not* include a status request (RS or RW).
 *
 *  returns: no indication of whether it worked.
 */
void devbrake::servo_write_only(const char *string)
{
  ROS_DEBUG("servo_write_only %s", string);

  // Flush the I/O buffers to ensure nothing is left over from any
  // previous command.
  tcflush(fd, TCIOFLUSH);

  // There is not much point in checking for errors on the write().
  // If something went wrong, we'll find out later on some command
  // that reads status.
  int res = write(fd, string, strlen(string));
  if (res < 0)
    ROS_ERROR_THROTTLE(100, "write() error: %d", errno);
}
