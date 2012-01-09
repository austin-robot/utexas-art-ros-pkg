/*
 *  ART steering servo controller device interface
 *
 *  Copyright (C) 2008 Austin Robot Technology
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
#include "silverlode.h"

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

devsteer::devsteer(int32_t center):
  req_angle_(0.0),
  center_ticks_(center)                 // for unit testing
{}

int devsteer::Open()
{
  // open the serial port
  int rc = this->Servo::Open(port_.c_str(), (O_RDWR|O_NOCTTY|O_NONBLOCK));
  if (fd < 0) {
    ROS_ERROR("Couldn't open %s (%s)", port_.c_str(), strerror(errno));
    return -1;
  }
    
  // set actual baud rate
  rc = configure_raw_port((B57600|CS8), 0);
  if (rc != 0) goto fail;

  rc = configure_steering();
  if (rc != 0) goto fail;

  req_angle_ = starting_angle_ = 0.0;	// initialize position
  starting_ticks_ = 0;			//   assuming wheel is centered

  return rc;

 fail:
  this->Servo::Close();
  return rc;				// Open() failed
}

int devsteer::Close()
{
  if (center_on_exit_)
    steering_absolute(0.0);		// center steering wheel
  return this->Servo::Close();
}

int devsteer::Configure()
{
  // use private node handle to get parameters
  ros::NodeHandle private_nh("~");

  private_nh.param("port", port_, std::string("/dev/steering"));
  ROS_INFO_STREAM("steering port = " << port_);

  private_nh.param("center_on_exit", center_on_exit_, false);
  if (center_on_exit_)
    ROS_INFO("center steering on exit");
  else
    ROS_INFO("do not center steering on exit");

  private_nh.param("training", training_, false);
  if (training_)
    ROS_INFO("using training mode");

  private_nh.param("simulate_moving_errors",
                   simulate_moving_errors_, false);
  if (simulate_moving_errors_)
    ROS_INFO("simulating intermittent moving errors");

  if (training_)
    ROS_INFO("using training mode");

  steering_rate_ = art_msgs::ArtVehicle::max_steer_degrees / 2.0;
  private_nh.getParam("steering_rate", steering_rate_);
  ROS_INFO("steering rate is %.2f degrees/sec.", steering_rate_);

  return 0;
}

/** check device status
 *
 *  @return 0 if device seems to be working correctly
 *  @todo make a header to define the status bits
 */
int devsteer::check_status(void)
{
  int rc = get_status_word(diag_msg_.status_word);

  if (rc == 0)
    {
      // internal status register bits to check
      uint16_t bad_status = (silverlode::isw::moving_error);
      uint16_t required_status = (silverlode::isw::temp_driver_en);
      if ((diag_msg_.status_word & bad_status) != 0
	  || (diag_msg_.status_word & required_status) != required_status)
	{
	  ROS_ERROR("SilverLode internal status error: 0x%04x",
		   diag_msg_.status_word);
	  rc = EIO;
	}
    }

  return rc;
}

/** get steering angle
 *
 *  When running without actual steering controller, simulate the
 *  wheel motion.
 *
 *  @pre not using the real wheel position sensor
 *  @pre center_ticks_ set correctly
 *
 *  @param degrees current steering angle, if I/O successful
 *  @return 0 if successful, errno value otherwise
 *
 *  @post sets diag_msg_.encoder value (if successful)
 */
int devsteer::get_angle(float &degrees)
{
  int rc = 0;

  if (have_tty)				// using actual device?
    {
      int32_t iticks;
      rc = get_encoder(iticks);
      if (rc == 0)
	{
	  degrees = ticks2degrees(iticks);
	}
      else
	{
	  ROS_WARN("encoder read failure, cannot estimate position");
	}
    }
  else
    {
      // simulate steering motion as a constant angular velocity
      float remaining_angle = req_angle_ - degrees;
      float degrees_per_cycle = (steering_rate_ /
                                 art_msgs::ArtHertz::STEERING);

      DBG("remaining angle %.3f, degrees per cycle %.3f",
          remaining_angle, degrees_per_cycle);

      if (fabs(remaining_angle) <= degrees_per_cycle)
	{
	  degrees = req_angle_;
	}
      else
	{
	  degrees += ((remaining_angle >= 0.0)?
		      degrees_per_cycle: -degrees_per_cycle);
	}
      ROS_DEBUG("simulated angle = %.2f degrees", degrees);

      // set corresponding (simulated) encoder value
      diag_msg_.encoder = degrees2ticks(degrees);
    }

  return rc;
}

/** get steering encoder value
 * 
 *  @param iticks set to encoder position, if I/O successful
 *  @return 0 if successful, errno value otherwise
 *
 *  @post diag_msg_.encoder updated, if successful and using real
 *                          device
 */
int devsteer::get_encoder(int32_t &iticks)
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
	  diag_msg_.encoder = iticks;
	}
      else
	{
	  ROS_INFO(" " DEVICE " unexpected response: `%s'", buffer);
	  rc = EINVAL;
	}
    }
  return rc;
}

// get steering status word
//
// returns: 0 if successful, errno value otherwise
//
// isw = current status, if I/O successful
//
int devsteer::get_status_word(uint16_t &status)
{
  int rc = send_cmd("@16 20\r");
  if (rc == 0)
    {
      if (have_tty)
        {
          unsigned int isw;
          if (1 == sscanf(buffer, "# 10 0014 %4x", &isw))
            {
              status = isw;
              ROS_DEBUG(" " DEVICE " status: `%s'", buffer);
            }
          else
            {
              ROS_WARN(" " DEVICE " unexpected response: `%s'", buffer);
              rc = EINVAL;
            }
        }
      else
	{
          status = silverlode::isw::temp_driver_en;
	}

      if (simulate_moving_errors_)
	{
	  // Hack: set moving error for four of every 64 seconds
          ros::Time now = ros::Time::now();
	  if ((now.sec & 0x003C) == 0)
	    status |= silverlode::isw::moving_error;
	}
    }
  return rc;
}

/** publish current diagnostic information
 *
 *  @param diag_pub ROS publish object for SteeringDiagnostics message.
 *  @pre diag_msg_ already updated for this cycle
 */
void devsteer::publish_diag(const ros::Publisher &diag_pub)
{
  get_encoder(diag_msg_.encoder);
  diag_msg_.header.stamp = ros::Time::now();
  diag_pub.publish(diag_msg_);
}

/** set the initial steering wheel angle
 * 
 *  @pre the controller is *not* in Profile Move mode, but data
 *  registers are set to the basic movement parameters.
 * 
 *  @post (if successful): The controller is in Profile Move
 *  Continuous mode.  The position is set to reflect the initial wheel
 *  angle, and the soft limits are set in registers 39 and 40.
 * 
 *  @return 0 if successful; errno value otherwise.
*/
int devsteer::set_initial_angle(float position)
{
  int rc = 0;

#if 0

  // Set starting_ticks to current encoder position.  This will be
  // (approximately) zero after initial power-on of the controller,
  // but may vary in subsequent runs if the controller has not been
  // reset.  That variable is used by degrees2ticks() as a conversion
  // offset.

  // BUG: should not rely on get_encoder() before setting
  // center_ticks_ when using real device with simulated sensor.
  rc = get_encoder(starting_ticks_);
  if (rc == 0)
    {
      // Since starting_angle is the current wheel angle, its negative
      // is the offset of the center wheel position.
      starting_angle_ = position;
      center_ticks_ = 
	(int32_t) lrint(-starting_angle_ * TICKS_PER_DEGREE) + starting_ticks_;
      diag_msg_.center_ticks = center_ticks_;

      ROS_INFO("starting ticks = %d, center ticks = %d",
               starting_ticks_, center_ticks_);

      // Attempt to set encoder soft stop limits.  If that fails, run
      // without them.  They are a safety net, limiting travel to 30
      // degrees from the center position.  This driver should never
      // request a position more than 29 degrees from center.  The
      // mechanical steering limit is about 31 degrees.  We avoid
      // hitting that limit lest it damage the stepper motor or its
      // linkage.
      write_register(39, center_ticks_ - 300001); // soft stop lower
      write_register(40, center_ticks_ + 300001); // soft stop upper

      // Send Profile Move Continuous (PMC) command.  If successful,
      // from now on any value written to data register 20 will
      // immediately cause the wheel to seek that position.  If it
      // fails, the device is inoperable and the driver shuts down.
      rc = servo_cmd("@16 240 0 0\r");
      if (rc != 0)
	ROS_WARN(DEVICE " failed to enter PMC mode.");

      // TODO check if KMC and KMR should be moved here
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
  starting_angle_ = position;
  center_ticks_ = 0;			// TODO: remove obsolete variable?
  diag_msg_.center_ticks = center_ticks_;
  starting_ticks_ = degrees2ticks(starting_angle_);
  ROS_INFO("starting ticks = %d, center ticks = %d",
           starting_ticks_, center_ticks_);
  rc = write_register(20, starting_ticks_); // initial wheel position
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
  write_register(39, center_ticks_ - 300001); // soft stop lower
  write_register(40, center_ticks_ + 300001); // soft stop upper

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
  req_angle_ = limit_travel(position);
  return encoder_goto(req_angle_);
}
 
int devsteer::steering_relative(float delta)
{
  DBG("steering_relative(%.3f)", delta);
  return steering_absolute(req_angle_ + delta);
}

/////////////////////////////////////////////////////////////////
// private methods
/////////////////////////////////////////////////////////////////

// Configure the steering controller for Profile Move Operation
//
int devsteer::configure_steering(void)
{
  int rc;

#if 0 // does not seem good (before or after RST)...
  rc = servo_cmd("@16 2\r");		// HLT: shut down all motion
  if (rc != 0) return rc;
#endif

#if 0 // should clear persistent KMC status, but does not work
  rc = servo_cmd("@16 163\r");		// CIS: clear internal status
  if (rc != 0) return rc;
  rc = servo_cmd("@16\r");		// POL: read polling status
  if (rc != 0) return rc;
  rc = servo_cmd("@16 1 65535\r");	// CPL: clear polling status
  if (rc != 0) return rc;
  rc = servo_cmd("@16 146\r");		// TTP: target to position
  if (rc != 0) return rc;

  // set KMC (Kill Motor Conditions) to stop motor for moving error
  rc = servo_cmd("@16 167 256 256\r"); 
  if (rc != 0) return rc;

  rc = servo_cmd("@16 227\r");		// enable motor drivers (EMD)
  if (rc != 0) return rc;
#endif

#if 1
  // The restart command branches to microcode address zero.
  // It never responds, and nothing works for a while afterward.
  servo_write_only("@16 4\r");          // RST: restart
  usleep(1000000);			// wait for that to finish
#endif

  // If the controller is in Profile Move Continuous (PMC) mode,
  // setting data registers 20 through 24 will have immediate and
  // undesired effects.  To avoid that, issue a PMX command first.
  //
  // If the controller was not in PMC mode, the PMX will fail, so
  // ignore any NAK response from that command.  (Should no longer be
  // a problem because of the RST.)
  // TODO remove this...
  servo_cmd("@16 242\r");		// PMX: exit profile move mode

  rc = write_register(20, 0);		// Reg 20 (Position) = 0
  if (rc != 0) return rc;

  // Max Acceleration 3865 = 1 RPS/s
  rc = write_register(21, 193274);     // Reg 21 (Accelera) = 25 RPS/s
  if (rc != 0) return rc;

  // Max Velocity (2^31/16,000) * (60RPM/RPS) = 8,053,064 = 1 RPS
  rc = write_register(22, 322122547);	// Reg 22 (Velocity) = 1 RPS
  if (rc != 0) return rc;

  rc = write_register(23, 193274);	// Reg 23 (Declerat) = -25 RPS/s
  if (rc != 0) return rc;

  rc = write_register(24, 0);		// Reg 24 (Offset) = 0
  if (rc != 0) return rc;

#if 1
  // set KMC (Kill Motor Conditions) to stop motor for moving error
  servo_cmd("@16 167 256 256\r"); 
  //rc = servo_cmd("@16 167 0 0\r"); 

  // set KMR (Kill Motor Recovery) to "do nothing"
  servo_cmd("@16 181 0\r"); 
#endif

  return rc;
}

// send encoder position absolute steering command
int devsteer::encoder_goto(float degrees)
{
  int32_t ticks = degrees2ticks(degrees);
  diag_msg_.last_request = ticks;
  ROS_DEBUG("setting steering angle to %.3f (%d ticks)", degrees, ticks);

  // Send Position to Stepper (Register 20)
  return write_register(20, ticks);
}

// Write a 32-bit integer value to a Quicksilver register.
int devsteer::write_register(int reg, int32_t val)
{
  char string[MAX_SERVO_CMD_BUFFER];
  ROS_DEBUG("writing %d to register %d", val, reg);
  snprintf(string, MAX_SERVO_CMD_BUFFER, "@16 11 %d %d\r", reg, val);
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
      int res = write(fd, string, len);
      if (res < 0)
        ROS_ERROR_THROTTLE(100, "write() error: %d", errno);

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
  if (training_)
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

/** Write a command to the Quicksilver, no response expected.
 *
 *  @return no indication of whether it worked.
 */
void devsteer::servo_write_only(const char *string)
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
