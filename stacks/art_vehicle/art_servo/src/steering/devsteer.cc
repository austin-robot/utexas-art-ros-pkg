/*
 *  ART steering servo controller device interface
 *
 *  Copyright (C) 2008 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <fcntl.h>
#include <termios.h>
#include <poll.h>

#include <ros/ros.h>
#include <art_msgs/ArtHertz.h>
#include <art_msgs/ArtVehicle.h>
#include "devsteer.h"

/////////////////////////////////////////////////////////////////
// public methods
/////////////////////////////////////////////////////////////////


#define CLASS "devsteer"

// Get the time (in ms)
int64_t devsteer::GetTime()
{
  ros::Time t = ros::Time::now();
  return (int64_t) t.sec * 1000 + (int64_t) t.nsec / 1000000;
}

devsteer::devsteer()
{
  verbose = 0;
  req_angle = 0.0;
  simulate_ = false;
}

int devsteer::Open()
{
  // open the serial port
  int rc = this->Servo::Open(port.c_str(), (O_RDWR|O_NOCTTY|O_NONBLOCK));
  if (fd < 0) {
    ROS_ERROR("Couldn't open %s (%s)", port.c_str(), strerror(errno));
    return -1;
  }

  simulate_ = !have_tty;
    
  // set actual baud rate
  rc = configure_raw_port((B57600|CS8), 0);
  if (rc != 0) goto fail;

  rc = configure_steering();
  if (rc != 0) goto fail;

  req_angle = starting_angle = 0.0;	// initialize position
  starting_ticks = 0;			//   assuming wheel is centered

  return rc;

 fail:
  this->Servo::Close();
  return rc;				// Open() failed
}

int devsteer::Close()
{
  if (center_on_exit)
    steering_absolute(0.0);		// center steering wheel
  return this->Servo::Close();
}

int devsteer::Configure()
{
  // use private node handle to get parameters
  ros::NodeHandle mynh("~");

  port = "/dev/null";
  mynh.getParam("port", port);
  ROS_INFO_STREAM("steering port = " << port);

  center_on_exit = false;
  mynh.getParam("center_on_exit", center_on_exit);
  if (center_on_exit)
    ROS_INFO("center steering on exit");
  else
    ROS_INFO("do not center steering on exit");

  training = false;
  mynh.getParam("training", training);
  if (training)
    ROS_INFO("using training mode");

  steering_rate = art_msgs::ArtVehicle::max_steer_degrees / 2.0;
  mynh.getParam("steering_rate", steering_rate);
  ROS_INFO("steering rate is %.2f degrees/sec.", steering_rate);

  return 0;
}

// get steering angle
//
// When running without actual device, simulate wheel motion.
//
// returns: 0 if successful, errno value otherwise
//
// degrees = current steering angle, if I/O successful
//
int devsteer::get_angle(float &degrees)
{
  int rc = 0;

  if (have_tty)				// using actual device?
    {
      // This is OK, we use the position sensor instead...
      ROS_WARN("steering angle not available from device (not implemented)");
      return ENOSYS;
    }
  else
    {
      // simulate steering motion as a constant angular velocity
      float remaining_angle = req_angle - degrees;
      float degrees_per_cycle = (steering_rate /
                                 art_msgs::ArtHertz::STEERING);

      DBG("remaining angle %.3f, degrees per cycle %.3f",
          remaining_angle, degrees_per_cycle);

      if (fabs(remaining_angle) <= degrees_per_cycle)
	{
	  degrees = req_angle;
	}
      else
	{
	  degrees += ((remaining_angle >= 0.0)?
		      degrees_per_cycle: -degrees_per_cycle);
	}
      ROS_DEBUG("simulated angle = %.2f degrees", degrees);
    }

  return rc;
}

// get steering encoder value -- converted to float
int devsteer::get_encoder(float &ticks)
{
  long iticks;
  int rc = get_encoder(iticks);
  if (rc == 0)
    ticks = (float) iticks;
  return rc;
}

// get steering encoder value
//
// returns: 0 if successful, errno value otherwise
//
// iticks = current encoder position, if I/O successful
//
int devsteer::get_encoder(long &iticks)
{
  int rc = send_cmd("@16 12 1\r");
  if (rc == 0 && have_tty)
    {
      // stage unit test: initialize buffer to encoder 329379.0 response
      //strncpy(buffer, "# 10 000C 0005 06A3", MAX_SERVO_CMD_BUFFER);
      unsigned int pos_high, pos_low;
      if (2 == sscanf(buffer, "# 10 000C %4x %4x", &pos_high, &pos_low))
	{
	  iticks = (pos_high << 16) + pos_low;
          ROS_DEBUG(" " DEVICE " response: `%s'", buffer);
	}
      else
	{
	  ROS_INFO(" " DEVICE " unexpected response: `%s'", buffer);
	  rc = EINVAL;
	}
    }
  return rc;
}

// set the initial steering wheel angle
//
// On entry: the controller is *not* in Profile Move mode, but data
// registers are set to the basic movement parameters.
//
// On exit (if successful): The controller is in Profile Move
// Continuous mode.  The position is set to reflect the initial wheel
// angle, and the soft limits are set in registers 39 and 40.
//
// returns: 0 if successful;
//	    errno value otherwise.
//
int devsteer::set_initial_angle(float position)
{
  int rc = 0;

#if 1

  // Set starting_ticks to current encoder position.  This will be
  // (approximately) zero after initial power-on of the controller,
  // but may vary in subsequent runs if the controller has not been
  // reset.  That variable is used by degrees2ticks() as a conversion
  // offset.
  rc = get_encoder(starting_ticks);
  if (rc == 0)
    {
      // Since starting_angle is the our current wheel angle, its
      // negative is the offset of the center wheel position.
      starting_angle = position;
      center_ticks = 
	(long) lrint(-starting_angle * TICKS_PER_DEGREE) + starting_ticks;

      ROS_INFO("starting ticks = %ld, center ticks = %ld",
               starting_ticks, center_ticks);

      // Attempt to set encoder soft stop limits.  If that fails, run
      // without them.  They are a safety net, limiting travel to 30
      // degrees from the center position.  This driver should never
      // request a position more than 29 degrees from center.  The
      // mechanical steering limit is about 31 degrees.  We avoid
      // hitting that limit lest it damage the stepper motor or its
      // linkage.
      write_register(39, center_ticks - 300001); // soft stop lower
      write_register(40, center_ticks + 300001); // soft stop upper

      // Send Profile Move Continuous (PMC) command.  If successful,
      // from now on any value written to data register 20 will
      // immediately cause the wheel to seek that position.  If it
      // fails, the device is inoperable and the driver shuts down.
      rc = servo_cmd("@16 240 0 0\r");
      if (rc != 0)
	ROS_WARN(DEVICE " failed to enter PMC mode.");
    }
  else
    ROS_WARN(DEVICE " failed reading steering encoder: "
             "initial position may be wrong!");

#else

  // Since starting_angle is our current wheel angle, write the
  // corresponding encoder position to register 20.  Since the PMC
  // command has not been issued, this does not cause any movement.
  // Subsequently, encoder position zero will correspond to wheel
  // angle zero.
  starting_angle = position;
  center_ticks = 0;			// TODO: remove obsolete variable
  starting_ticks = degrees2ticks(starting_angle);
  ROS_INFO("starting ticks = %ld, center ticks = %ld",
           starting_ticks, center_ticks);
  rc = write_register(20, starting_ticks); // initial wheel position
  if (rc != 0)
    {
      ROS_ERROR(DEVICE " failed to set initial wheel encoder position.");
      return rc;			// device failure
    }

  // Attempt to set encoder soft stop limits, but if it fails run
  // without them.  They are a safety net, limiting travel to 30
  // degrees from the center position.  This driver should never
  // request a position more than 29 degrees from center.  The
  // mechanical steering limit is about 31 degrees.  We avoid hitting
  // that limit lest it damage the stepper motor or its linkage.
  write_register(39, center_ticks - 300001); // soft stop lower
  write_register(40, center_ticks + 300001); // soft stop upper

  // Send Profile Move Continuous (PMC) command.  If successful, from
  // now on any value written to data register 20 will immediately
  // cause the wheel to seek that position.  If this command fails,
  // the device is inoperable and the driver will shut down.
  rc = servo_cmd("@16 240 0 0\r");
  if (rc != 0)
    ROS_ERROR(DEVICE " failed to enter PMC mode.");

#endif  

  return rc;
}

int devsteer::steering_absolute(float position)
{
  DBG("steering_absolute(%.3f)", position);
  req_angle = limit_travel(position);
  return encoder_goto(req_angle);
}
 
int devsteer::steering_relative(float delta)
{
  DBG("steering_relative(%.3f)", delta);
  return steering_absolute(req_angle + delta);
}

/////////////////////////////////////////////////////////////////
// private methods
/////////////////////////////////////////////////////////////////

// Configure the steering controller for Profile Move Operation
//
int devsteer::configure_steering(void)
{
  int rc;

  // If the controller has not been reset since the last run, it will
  // probably still be in Profile Move Continuous (PMC) mode.  If so,
  // setting data registers 20 through 24 will have immediate and
  // undesired effects.  To avoid that, issue a PMX command first.
  //
  // If the controller was not in PMC mode, the PMX will fail, so
  // ignore any NAK response from that command.
  servo_cmd("@16 242\r");		// PMX: exit profile move mode

  rc = write_register(20, 0);		// Reg 20 (Position) = 0
  if (rc != 0) return rc;

  // Max Acceleration 3865 = 1 RPS/s
  rc = write_register(21, 193274);     // Reg 21 (Accelera) = 25 RPS/s
  if (rc != 0) return rc;

  // Max Velocity (2^31/16,000) * (60RPM/RPS) = 8,053,064 = 1 RPS

  // Max Velocity: 2**31-1 == 4000RPM, /16,000) * (60RPM/RPS) = 8,053,064 = 1 RPS
  rc = write_register(22, 322122547);	// Reg 22 (Velocity) = 1 RPS
  if (rc != 0) return rc;

  rc = write_register(23, 193274);	// Reg 23 (Declerat) = -25 RPS/s
  if (rc != 0) return rc;

  rc = write_register(24, 0);		// Reg 24 (Offset) = 0
  return rc;
}

// send encoder position absolute steering command
int devsteer::encoder_goto(float degrees)
{
  long ticks = degrees2ticks(degrees);

  ROS_DEBUG("setting steering angle to %.3f (%ld ticks)", degrees, ticks);

  // Send Position to Stepper (Register 20)
  return write_register(20, ticks);
}

// Write an integer value to a Quicksilver register.
int devsteer::write_register(int reg, long val)
{
  char string[MAX_SERVO_CMD_BUFFER];
  ROS_DEBUG("writing %ld to register %d", val, reg);
  snprintf(string, MAX_SERVO_CMD_BUFFER, "@16 11 %d %ld\r", reg, val);
  return servo_cmd(string);
}

/*  Write a command to the Quicksilver.  The device returns a status
 *  string after receiving it.  Skip I/O if /dev/null.
 *
 *  returns: 0 if successful, errno value otherwise.
 */
int devsteer::send_cmd(const char *string)
{
  int len = strlen(string);

  // TODO: trailing \r in string messes up this message...
  ROS_DEBUG(" " DEVICE " command: `%s'", string);

  if (!have_tty)			// null device will not respond
    {
      strncpy(buffer, "* 10", 4);	// fake acknowledgement response
      return 0;
    }

  int rc=0;
  int attempts = 3;			// number of times to try command
  do
    {
      // There is not much point in checking for errors on the
      // write().  If something went wrong, we'll find out by reading
      // the device status.
      int res;
      res=write(fd, string, len);

      // Set timeout in msecs.  The device normally responds in two.
      int timeout = 20;
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
		ROS_WARN("timeout on poll");
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

	  int bytes = read(fd, buffer + linelen, 1);
	  if (bytes < 0)
	    {
	      if (errno == EINTR)
		continue;
	      rc = errno;
	      ROS_ERROR("error: %s", strerror(rc));
	      break;			// retry servo command
	    }
          ROS_DEBUG_NAMED("details",
                          DEVICE " read() returns %d %c(0x%02X)",
                          bytes, buffer[linelen], buffer[linelen]);
	  if (bytes==0)
	    continue;
	  else if (buffer[linelen] != '\r') // not end of line?
	    {
	      // have a new character
	      if (++linelen >= MAX_SERVO_CMD_BUFFER)
		{
		  ROS_ERROR(DEVICE " buffer overflow: %s", buffer);
		  rc = ENOSPC;
		  break;		// retry servo command
		}
	    }
	  else				// have a complete line
	    {
	      buffer[linelen] = '\0';	// EOL is end of string
	      return 0;			// success
	    }
	}

      // operation failed, flush the I/O buffers
      tcflush(fd, TCIOFLUSH);
    }
  while (--attempts > 0);		// retry, if error
  
  return rc;
}

/*  Write a command to the Quicksilver.  Check that the status string
 *  returned is an acknowledgement.  Do nothing when in training mode.
 */
int devsteer::servo_cmd(const char *string)
{
  if (training)
    return 0;				// send no commands

  int rc = send_cmd(string);
  if ((rc == 0)				  // I/O successful?
      && strncmp(buffer, "* 10", 4) != 0) // no acknowledgement?
    {
      ROS_INFO(DEVICE " returned error: %s", buffer);
      rc = EIO;
    }
  return rc;
}
