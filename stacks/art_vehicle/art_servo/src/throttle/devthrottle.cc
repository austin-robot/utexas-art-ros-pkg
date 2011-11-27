/*
 *  Description:  Throttle servo controller.
 *
 *  Copyright (C) 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

extern "C" {
#include <string.h>
#include <errno.h>
#include <ctype.h>
}

#include <poll.h>
#include "devthrottle.h"

/**  \file

     @brief ROS driver for the ART throttle servo controller.

     @author Jack O'Quin, Patrick Beeson
*/

#define CLASS "devthrottle"
#define DEVICE "Throttle"

/////////////////////////////////////////////////////////////////
// public methods
/////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Get the time (in ms)
//
int64_t devthrottle::GetTime()
{
  ros::Time t = ros::Time::now();
  return (int64_t) t.sec * 1000 + (int64_t) t.nsec / 1000000;
}

devthrottle::devthrottle(bool train)
{
  training = train;
  already_configured = false;
  last_req = cur_position = 0.0;
  cmd_p = (uint8_t *) &cmd;
  resp_p = (uint8_t *) &resp;
  memset(&stat, 0, sizeof(stat));

  // Set throttle parameters -- make sure the defaults won't overrev
  // the engine.  These values will be used for /dev/null, in training
  // mode, or in case of failure.  They may be updated later if
  // calibration changes are detected.

  // use private node handle to get parameters
  ros::NodeHandle mynh("~");

  throttle_limit = 0.40;
  mynh.getParam("throttle_limit", throttle_limit);
  rpm_redline = 2750.0;
  mynh.getParam("rpm_redline", rpm_redline);
  ROS_INFO("throttle_limit %.f %%, rpm_redline %.f",
           throttle_limit*100.0, rpm_redline);

  // AVR controller parameters
  avr_pos_max = 204.0;
  mynh.getParam("avr_pos_max", avr_pos_max);
  avr_pos_min = 31;
  mynh.getParam("avr_pos_min_est", avr_pos_min);
  avr_pos_range = avr_pos_max - avr_pos_min;
  avr_pos_epsilon = 1;
  mynh.getParam("avr_pos_epsilon", avr_pos_epsilon);
  avr_out_max = 1023;
  mynh.getParam("avr_out_max", avr_out_max);
  ROS_INFO("AVR max, estimated min, max output, epsilon: %.1f, %d, %d, %d",
           avr_pos_max, avr_pos_min, avr_out_max, avr_pos_epsilon);

  // get PID parameters for controller
  double throttle_kp = 512.0;
  mynh.getParam("pid/p", throttle_kp);
  double throttle_ki = 0.04;
  mynh.getParam("pid/i", throttle_ki);
  double throttle_kd = 64.0;
  mynh.getParam("pid/d", throttle_kd);
  ROS_INFO("throttle ctlr PID gains (%.3f, %.3f, %.3f)",
           throttle_kp, throttle_ki, throttle_kd);

  // translate PID values into AVR controller encodings
  avr_kp = (int) (throttle_kp * 256.0);
  avr_ki = (int) (throttle_ki * 256.0);
  avr_kd = (int) (throttle_kd * 256.0);
}


int devthrottle::Open(const char *device)
{
  int rc;
  memset(&stat, 0, sizeof(stat));	// clear status response

  rc = this->Servo::Open(device, (O_RDWR|O_NOCTTY|O_NDELAY|O_NONBLOCK));
  if (rc != 0) return rc;		// Open() failed

  rc = this->configure_raw_port(B115200 | CS8, 0);
  if (rc != 0) goto fail;

  // no need to configure or calibrate throttle if already done or
  // when in training mode
  if (already_configured || training) return 0;

  rc = configure_controller();
  if (rc != 0) goto fail;

  if (have_tty)
    {
      rc = calibrate_idle();
      if (rc != 0) goto fail;
    }

  rc = throttle_absolute(0.0);		// go to idle position
  if (rc != 0) goto fail;

  already_configured = true;

  return 0;				// successful Open()

 fail:
  this->Servo::Close();
  return rc;				// Open() failed
}

int devthrottle::Close(void)
{
  throttle_absolute(0.0);		// go to idle position
  return this->Servo::Close();
}

bool devthrottle::query_estop(void)
{
  return (avr_get_estop(&stat) != 0);
}

int devthrottle::query_pid(float *pwm, float *dstate, float *istate)
{
  if (avr_get_len(&stat) < avr_offset(data.status.pwm_count))
    return EAGAIN;			// no PID data available

  // packet has PID data, unpack it
  uint8_t direction = avr_get_direction(&stat);
  *dstate = avr_get_dstate(&stat);
  *istate = avr_get_istate(&stat);
  *pwm = avr_get_pwm(&stat);

  ROS_DEBUG(CLASS ": throttle PID status: dir=%u ds=%.f, is=%.f pwm=%.f",
	    direction, *dstate, *istate, *pwm);

  if (direction == 0)		// motor reversing?
    *pwm = - *pwm;

  return 0;
}

int devthrottle::query_rpms(float *data)
{
  if (avr_get_rvld(&stat) != 0)		// no RPM data?
    return EAGAIN;

  *data = avr_get_rpms(&stat);
  ROS_DEBUG(DEVICE " engine speed = %.f RPM", *data);
  return 0;
}

// query throttle controller status
//
// updates cur_position to best available estimate
//
// returns:	0 if successful, updating stat
//		nonzero otherwise, stat unchanged
int devthrottle::query_status()
{
  int rc = send_cmd(STATUS_CMD);
  if (have_tty && rc == 0)
    {
      ROS_DEBUG(DEVICE " status: [G%d P%d R%d E%d]",
		avr_get_gen(&resp), avr_get_pvld(&resp),
		avr_get_rvld(&resp), avr_get_estop(&resp));
      if (avr_get_gen(&resp) == 0)	// general status error?
	{
	  ROS_WARN(DEVICE " reports general error status 0x%x",
		   avr_get_diag(&resp));
	}
      stat = resp;			// save STATUS response
    }

  cur_position = last_req;		// good approximation

  // Use actual throttle position, if valid
  if (have_tty && avr_get_pvld(&stat))
    {
      int avr_pos = avr_get_pos(&stat);

      ROS_DEBUG(CLASS ": AVR throttle position = %d", avr_pos);

      // check that throttle position sensor is really working
      if (avr_pos > AVR_POS_ABSURD)	// throttle sensor working?
	cur_position = avr2pos(avr_pos);
      else
	{
	  ROS_ERROR(CLASS ": AVR throttle sensor failure!");
	  rc = EBUSY;			// device not responding
	}
    }

  return rc;
}

int devthrottle::throttle_absolute(float position)
{
  DBG("throttle_absolute(%.3f)", position);
  last_req = limit_travel(position);
  /// @todo once controller returns RPMs correctly, check rpm_redline
  ROS_DEBUG("requesting throttle position %.3f", last_req);
  return send_goto(pos2avr(last_req));
}
 
int devthrottle::throttle_relative(float delta)
{
  DBG("throttle_relative(%.3f)", delta);
  return throttle_absolute(cur_position + delta);
}


/////////////////////////////////////////////////////////////////
// private methods
////////////////////////////////////////////////////////////////

// determine current actual idle throttle position (SLOWLY)
//
// - for use only during Open().
//
// returns: 0 if successful, updating avr_pos_min and avr_pos_range
//	    nonzero if failure, causing the Open() to fail
int devthrottle::calibrate_idle(void)
{
  int actual_pos_min = (int) rintf(avr_pos_max); // start with a high value

  int rc = send_goto(0);		// set idle throttle
  if (rc != 0) return rc;
  cur_position = last_req = 0.0;

  int prev_pos = -(avr_pos_epsilon*2);	// significantly below zero
  for (;;)				// loop until position settles
    {
      rc = send_cmd(STATUS_CMD);
      if ((rc != 0) || !avr_get_pvld(&resp))
	{
	  if (rc == 0) rc = ENODEV;
	  ROS_ERROR(DEVICE " calibration failed! (%s)",
		    strerror(rc));
	  return rc;
	}

      int cur_pos = resp.data.status.pos; // current sensor reading

      // Once in a while, the sensor returns a bogus value.  Only
      // consider cur_pos to be valid when it is within epsilon of the
      // previous reading.  This indicates the throttle has settled to
      // a stable, achievable value.
      if (abs(cur_pos - prev_pos) <= avr_pos_epsilon)
	{
	  if (cur_pos <= AVR_POS_ABSURD)
	    {
	      ROS_ERROR(DEVICE " position sensor failure: %d",
			cur_pos);
	      return ENODEV;
	    }

	  if (cur_pos < actual_pos_min)
	    {
	      actual_pos_min = cur_pos;
              ROS_DEBUG(DEVICE " intermediate avr_pos_min value: %d\n", cur_pos);
	    }
	  else if (cur_pos < actual_pos_min + avr_pos_epsilon)
	    {
	      // This is close enough to the achievable min.  While
	      // updating the class variables, add epsilon to the
	      // computed pos min so the motor need not work to
	      // maintain an idle setting.  Tell the controller what
	      // we discovered.
	      // (EXPERIMENTAL: try not adding epsilon to idle speed)
	      //avr_pos_min = actual_pos_min + avr_pos_epsilon;
	      avr_pos_min = actual_pos_min;
	      avr_pos_range = avr_pos_max - avr_pos_min;
              ROS_DEBUG(DEVICE " actual avr_pos_min = %d\n", avr_pos_min);
	      send_cmd08(SET_IDLE_CMD, avr_pos_min);
	      return 0;
	    }
	}

      usleep(50000);		// wait 50 milliseconds
      prev_pos = cur_pos;
    }
}

// utility adapted from art_command_protocol.c (in firmware)
//   (len parm should not include the final csum byte)
uint8_t devthrottle::cmd_compute_csum(uint8_t *buffer, int len)
{
  int raw_sum = 0;
  int index;
  uint8_t csum;

  for (index=0; index < len; index++) {
    raw_sum += buffer[index];
  }

  csum = (uint8_t) (raw_sum & 0xff);
  return -csum;
}

// configure the throttle controller
int devthrottle::configure_controller()
{
  // make sure controller is not in CLI mode
  int rc = send_cmd08(CLI_SET_CMD, 0);
  if (rc != 0) return rc;

  // send PID parameters to controller
  rc = send_cmd32(PID_KP_CMD, avr_kp);
  if (rc != 0) return rc;
  rc = send_cmd32(PID_KI_CMD, avr_ki);
  if (rc != 0) return rc;
  rc = send_cmd32(PID_KD_CMD, avr_kd);
  if (rc != 0) return rc;
  rc = send_cmd16(PID_LIMITS_CMD, avr_out_max);

  return rc;
}

// hex conversion utilities
static char int2hex[] = "0123456789abcdef";

static inline uint8_t hex2int(char c)
{
  uint8_t val;

  if ((c >= 'a') && (c <= 'f')) {
    val = 10 + c - 'a';
  } else if ((c >= 'A') && (c <= 'F')) {
    val = 10 + c - 'A';
  } else {
    val = c - '0';
  }
  return val;
}

// decode response character
//
//	unpacks hex char into resp struct
//	updates resp_bytes and resp_digits as needed
//
inline void devthrottle::decode_char(char c,
					  int *resp_bytes,
					  int *resp_digits)
{
  if (isxdigit(c))
    {
      // store hex nibble in resp struct
      uint8_t val = hex2int(c);
      if (++*resp_digits & 0x01) {	// odd-numbered digit?
	resp_p[*resp_bytes] = (val << 4);
      } else {
	resp_p[(*resp_bytes)++] |= val;
      }
    }
  else if (!isspace(c))			// invalid hex digit?
    {
      // The controller may return a '\n' after the '\r'.  If this or
      // any other whitespace is received just ignore it, but complain
      // about anything else.
      ROS_WARN(DEVICE " non-hex char code (0x%02x) received", c);
    }
}

// format the cmd string
inline int devthrottle::format_cmd(char *cmdstr)
{
  static int next_seq = 0;

  // Compute sequence number and checksum.  Any retries should use the
  // same sequence number.  We avoid using 0xFF so the controller can
  // use that as an "unknown" sequence number.
  if (++next_seq >= 0xFF)
    {
      next_seq = 0;
    }
  cmd.seq = next_seq;
  int len = avr_get_len(&cmd);
  cmd_p[len-1] = cmd_compute_csum(cmd_p, len-1);

  // convert command string to hex characters (and log)
  int cmdlen = 0;
  for (int i = 0; i < len; ++i)
    {
      cmdstr[cmdlen++] = int2hex[(cmd_p[i]>>4) & 0xF];
      cmdstr[cmdlen++] = int2hex[cmd_p[i] & 0xF];
    }

  cmdstr[cmdlen] = '\0';		// add null for log message
  ROS_DEBUG(DEVICE " command: `%s\\r\\n'", cmdstr); 
  cmdstr[cmdlen++] = '\r';		// replace with CR for device
  cmdstr[cmdlen++] = '\n';		// add a newline, too

  // buffer overflow should not ever happen
  ROS_ASSERT(cmdlen <= MAX_SERVO_CMD_BUFFER);

  return cmdlen;
}

// read one response byte from the controller
//
//  linelen: number of bytes already received for this packet
//  returns: 0 if successful; errno value otherwise.
//
inline int devthrottle::read_byte(int linelen)
{
      int timeout = 15;			// timeout in msecs
      
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
		return errno;
	      }
	    else if (!retval)
	      {
		ROS_WARN("timeout on poll");
		return EBUSY;
	      }
	    else {
	      if ((fds[0].revents & POLLERR) ||
		  (fds[0].revents & POLLHUP) ||
		  (fds[0].revents & POLLNVAL))
		{
		  ROS_ERROR("Device error on poll");
		  return EIO;
		}
	    }
	  }

	  int bytes = read(fd, &buffer[linelen], 1);
	  if (bytes < 0)
	    {
	      if (errno == EINTR)
		continue;
	      ROS_WARN(DEVICE " error: \"%s\" after %d bytes received",
		       strerror(errno), linelen);
	      return errno;		// read failed
	    }
	  
	  if (bytes > 0)		// got something?
	    {
              ROS_DEBUG(DEVICE " read() returns 0x%02x", buffer[linelen]);
	      return 0;
	    }
	  
          ROS_DEBUG(DEVICE " read() returns no data");
	}
}

// Send various-length commands to the throttle controller.
//
//  returns: 0 if successful, errno value otherwise.
//
int devthrottle::send_cmd(int ccode)
{
  avr_set_com_len(&cmd, ccode, 3);
  return servo_cmd();
}

int devthrottle::send_cmd08(int ccode, uint8_t data)
{
  avr_set_com_len(&cmd, ccode, 4);
  cmd.data.data08 = data;
  return servo_cmd();
}

int devthrottle::send_cmd16(int ccode, uint16_t data)
{
  avr_set_com_len(&cmd, ccode, 5);
  cmd.data.data16 = htons(data);
  return servo_cmd();
}

int devthrottle::send_cmd32(int ccode, uint32_t data)
{
  avr_set_com_len(&cmd, ccode, 7);
  cmd.data.data32 = htonl(data);
  return servo_cmd();
}

// Send goto command to the controller.
inline int devthrottle::send_goto(uint8_t pos)
{
  ROS_DEBUG("setting AVR throttle position to %d", pos);
  return send_cmd08(GOTO_CMD, pos);
}

/*  Write formatted command to the throttle controller.
 *
 *  input:   cmd struct to be sent
 *  returns: 0 if successful, errno value otherwise.
 *	     resp struct returned from controller
 */
int devthrottle::servo_cmd(void)
{
  char cmdstr[MAX_SERVO_CMD_BUFFER];
  int cmdlen = format_cmd(cmdstr);	// format the cmd string

  if (!have_tty) return 0;		// /dev/null always succeeds

  memset(&resp, 0xFF, sizeof(resp));	// for debug purposes
  int rc;
  int attempts = 3;			// number of times to try command
  do
    {
      // Flush the I/O buffers to ensure nothing is left over from any
      // previous command.
      tcflush(fd, TCIOFLUSH);

      // There is not much point in checking for errors on the
      // write().  If something went wrong, we'll find out while
      // reading the response.
      int res = write(fd, cmdstr, cmdlen);	// send the command
      if (res < 0)
        ROS_ERROR_THROTTLE(100, "write() error: %d", errno);

      ROS_DEBUG(DEVICE " write() %s\n",cmdstr);

      int resp_bytes = 0;
      int resp_digits = 0;
      int linelen = 0;

      for (;;)				// read the response packet
	{
	  rc = read_byte(linelen);
	  if (rc != 0) break;		// retry command

	  // ignore any leading newline left from a previous packet
	  if (buffer[linelen] == '\n' && linelen == 0)
	    continue;

	  // If we have a complete packet, validate it and return.
	  // Normally, we only see the carriage return, but will honor
	  // a newline, if the '\r' got lost in transmission.
	  if (buffer[linelen] == '\r' || buffer[linelen] == '\n')
	    {
	      rc = validate_response(resp_bytes, linelen);
	      if (rc != 0) break;	// retry command
	      return 0;			// success
	    }

	  // Unpack the data from each packet char as it arrives.
	  decode_char(buffer[linelen], &resp_bytes, &resp_digits);

	  if (++linelen >= MAX_SERVO_CMD_BUFFER)
	    {
	      char c = buffer[MAX_SERVO_CMD_BUFFER-1];
	      buffer[MAX_SERVO_CMD_BUFFER-1] = '\0';
	      ROS_WARN(DEVICE " buffer overflow: %s%c", buffer, c);
	      rc = ENOSPC;
	      break;			// retry command
	    }
	}
    }
  while (--attempts > 0);		// retry after error

  ROS_WARN(DEVICE " command failed (%s)", strerror(rc));
  return rc;
}

// validate response packet
inline int devthrottle::validate_response(int resp_bytes, int linelen)
{
  buffer[linelen] = '\0';		// EOL is end of string
  ROS_DEBUG(DEVICE " \"%s\" response received", buffer);

  // check for errors
  if (resp_bytes != avr_get_len(&resp)) // packet length mismatch
    {
      ROS_WARN(DEVICE " packet length error: %s", buffer);
      return EIO;
    }

  uint8_t resp_csum = resp_p[resp_bytes-1];
  uint8_t csum = cmd_compute_csum(resp_p, resp_bytes-1);
  if (csum != resp_csum)
    {
      ROS_WARN(DEVICE " checksum 0x%02x should be 0x%02x", csum, resp_csum);
      return EIO;
    }

  if (resp.seq != cmd.seq)
    {
      ROS_WARN(DEVICE " sequence number 0x%02x should be 0x%02x",
		resp.seq, cmd.seq);
      return EIO;
    }

  if (avr_get_com(&resp) == NAK_CMD)
    {
      ROS_WARN(DEVICE " responded with NAK error %d", resp.data.data08);
      return EIO;
    }

  return 0;
}
