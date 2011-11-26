/*
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file

     National Control Devices ioadr8x device I/O.

     \todo set O_NONBLOCK and check for device timeout

     \author Jack O'Quin
 */

#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>			// for sleep
#include <string.h>
#include <fcntl.h>
#include <termios.h>

#include <ros/ros.h>

#include "dev8x.h"

dev8x::dev8x(const char *pn)
{
  state = UNREADY;

  // handle config parameters
  strncpy(port_name, pn, sizeof(port_name));
  have_tty = (strcmp(port_name, "/dev/null") != 0);
  DBG("dev8x port: %s", port_name);

  // initialize simulated ports
  sim_relays = 0;
  for (int i = 0; i < IOADR_MAX_INPUTS; i++)
    sim_port[i] = 0;
}

int dev8x::setup()
{
  DBG("setup()");
  // open the serial port
  fd = open(port_name, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    ROS_WARN("Couldn't open %s", port_name);
    return -1;
  }  

  relay_wait_time = 0.0;
  state = READY;

  // Configure port 19.2k baud 8n1
  DBG("Configuring the ioadr8x.");
  configure_port(B19200 | CS8, 0);
  DBG("DONE: Configuring the ioadr8x.");
  
  return 0;
}

int dev8x::shutdown()
{
  DBG("shutdown()");
  int rc = close(fd);
  state = UNREADY;
  return rc;
}

int dev8x::configure_port (int cflags, int iflags) 
{
  assert(state == READY);

  struct termios newtio;
  memset(&newtio, 0, sizeof(newtio));
        
  /* 
     BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
     CRTSCTS : output hardware flow control (only used if the cable has
     all necessary lines. See sect. 7 of Serial-HOWTO)
     CS8     : 8n1 (8bit,no parity,1 stopbit)
     CLOCAL  : local connection, no modem contol
     CREAD   : enable receiving characters
  */
  newtio.c_cflag = cflags | CLOCAL | CREAD;
  
  /*
    IGNPAR  : ignore bytes with parity errors
    ICRNL   : map CR to NL (otherwise a CR input on the other computer
    will not terminate input)
    otherwise make device raw (no other input processing)
  */
  newtio.c_iflag = iflags | IGNPAR; // Need ICRNL for QuickSilver Motor
  

  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  
  /* 
     initialize all control characters 
     default values can be found in /usr/include/termios.h, and are given
     in the comments, but we don't need them here
  */
  newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
  newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character received */
  
  /* 
     now clean the modem line and activate the settings for the port
  */
  tcflush(fd, TCIOFLUSH);

  return tcsetattr(fd,TCSANOW,&newtio);
}

// get relay values from dev8x
int dev8x::query_relays(uint8_t *relays)
{
  if (relays_busy())
    return EBUSY;

  // send Status command
  buffer[0] = 254;
  buffer[1] = 14;
  buffer[2] = 24;			// Get Status
  DBG("query_relays: writing command.");
  int res = write(fd, buffer, 3);
  if (res < 0)
    ROS_ERROR_THROTTLE(100, "write() error: %d", errno);

  // read result
  DBG("query_relays: reading result.");
  int rc = read(fd, buffer, 1);
  if (rc == 1)
    *relays = buffer[0];
  else if (have_tty == false)		// using /dev/null
    *relays = sim_relays;
  else					// I/O error
    {
      if (rc == 0) rc = EBUSY;
      ROS_WARN("ioadr8x read error: %s", strerror(rc));
      return rc;
    }

  DBG("relay status: 0x%02x", *relays);
  return 0;
}

int dev8x::read_8bit_port(int ch, int *data)
{
  if (relays_busy())
    return EBUSY;

  // Send Command
  buffer[0] = 254;
  buffer[1] = 6;
  buffer[2] = ch;
  int res = write(fd, buffer, 3);
  if (res < 0)
    ROS_ERROR_THROTTLE(100, "write() error: %d", errno);

  int rc = read(fd, buffer, 1);
  if (rc == 1)				// success?
    *data = buffer[0];
  else if (have_tty == false)		// simulated port?
    *data = sim_sawtooth(ch, 1<<8);
  else					// I/O error
    {
      ROS_WARN("ioadr8x read error: ch = %d, rc = %d", ch, rc);
      if (rc < 0)
	return errno;
      else
	return EIO;
    }

  DBG("port %d returns 0x%02x", ch, *data);
  return 0;
}

int dev8x::read_10bit_port(int ch, int *data)
{
  if (relays_busy())
    return EBUSY;

  // can only read 10-bit data from analog ports (3-7)
  assert(ch >= 3);

  // Send Command
  buffer[0] = 254;
  buffer[1] = 6;
  buffer[2] = 8 + ch - 3;
  int res = write(fd, buffer, 3);
  if (res < 0)
    ROS_ERROR_THROTTLE(100, "write() error: %d", errno);

  int rc = read(fd, &buffer[1], 1);
  if (rc != 1 && have_tty)		// failure?
    {
      ROS_WARN("ioadr8x 10-bit read error: ch = %d, rc = %d", ch, rc);
      if (rc < 0)
	return errno;
      else
	return EIO;
    }

  rc = read(fd, buffer, 1);
  if (rc == 1)				// success?
    *data = (buffer[1]<<8) | buffer[0];
  else if (have_tty == false)		// simulated port?
    *data = sim_sawtooth(ch, 1<<10);
  else					// I/O error
    {
      ROS_WARN("ioadr8x 10-bit read error: ch = %d, rc = %d", ch, rc);
      if (rc < 0)
	return errno;
      else
	return EIO;
    }

  DBG("port %d returns 0x%04x", ch, *data);
  return 0;
}

// returns true if last relay write has not completed
bool dev8x::relays_busy(void)
{
  if (state == RELAY_WAIT)
    {
      double now = ros::Time::now().toSec();
      if ((now - relay_wait_time) > MIN_RELAY_WAIT)
	{
	  state = READY;
	  DBG("accessing relays (time %.6f)", now);
	}
      else
	{
	  DBG("tried to access relays too soon after setting them"
              " (time %.6f)", now);
	  return true;
	}
    }
  return false;
}

// send new relay values to ioadr8x
int dev8x::set_relays(uint8_t bitmask)
{
  if (relays_busy())
    return EBUSY;

  DBG("setting relays to 0x%02x", bitmask);

  if (have_tty == false)
    {
      sim_relays = bitmask;
      return 0;
    }

  assert(state == READY);
  buffer[0] = 254;
  buffer[1] = 14;
  buffer[2] = 40;
  buffer[3] = bitmask;
  int rc = write(fd, buffer, 4);
  if (rc == 4)
    {
      // The device requires a wait after setting all relays before
      // accessing it again.  Since we return immediately, the ioadr
      // driver must leave them alone until its next cycle, which
      // should be long enough for the device to finish.  If it isn't,
      // the next operation will return EBUSY.
      relay_wait_time = ros::Time::now().toSec();
      state = RELAY_WAIT;
      return 0;
    }
  else					// I/O failed
    {
      if (rc < 0)
	rc = errno;
      else
	rc = EIO;
      ROS_WARN("write error: %s", strerror(rc));
      return rc;
    }
}

// Simulate analog port returning a sawtooth wave, making it easy to
// see if messages get lost.
//
// entry: sim_port[ch] contains next value to use
// exit:  sim_port[ch] updated
uint16_t dev8x::sim_sawtooth(int ch, uint16_t limit)
{
  uint16_t data = sim_port[ch]++;
  if (sim_port[ch] >= limit)
    sim_port[ch] = 0;
  return data;
}
