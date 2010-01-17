/*
 *  Description:  Generic servo controller interface.
 *
 *  The actual servo device controllers are subclasses of this one.
 *
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology                    
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _SERVO_H_
#define _SERVO_H_

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <ros/ros.h>

#define MAX_SERVO_CMD_BUFFER 48

class Servo 
{
 public:

  virtual ~Servo() {};

  virtual int Open(const char *device)
  {return Open(device, (O_RDWR|O_NOCTTY));}

  virtual int Open(const char *device, int flags)
  {
    int rc = 0;
    have_tty = (strcmp(device, "/dev/null") != 0);
    strncpy(devName, (char *) device, sizeof(devName));
    fd = open(devName, flags);
    if (fd < 0)
      {
	rc = errno;
	ROS_ERROR("Servo::Open(%s) error: %s", devName, strerror(rc));
      }
    else
      {
	ROS_DEBUG("Servo::Open(%s) successful", devName);
      }
    return rc;
  }

  virtual int Close(void)
  {
    int rc = close(fd);
    if (rc < 0)
      {
	rc = errno;
	ROS_ERROR("Servo::Close() error: %s", strerror(rc));
      }
    return rc;
  }

 protected:

  int fd;
  bool have_tty;
  char devName[FILENAME_MAX];
  char buffer[MAX_SERVO_CMD_BUFFER];

  /* We prefer to use the raw serial port for servo devices.  That
   * minimizes processing in the kernel tty driver, which might get in
   * the way of an accurate perception of the true state of the device.
   */
  virtual int configure_raw_port(int cflags, int iflags)
  {
    if (!have_tty) return 0;		// no config for /dev/null

    int rc;
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio)); /* clear struct for new port settings */
        
    /* 
       BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
       CRTSCTS : output hardware flow control (only used if the cable has
       all necessary lines. See sect. 7 of Serial-HOWTO)  Note: this fails
       badly for the ART brake servo (maybe the cable was built wrong).
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
    newtio.c_iflag = iflags | IGNPAR;
    newtio.c_oflag = ONLCR;		// QuickSilver TODO: is this relevant??
    newtio.c_lflag = 0;
  
    /* 
       initialize all control characters 
       default values can be found in /usr/include/termios.h, and are given
       in the comments, but we don't need them here
    */
    newtio.c_cc[VTIME]    = 0;    /* timeout in units of .1 sec (unused) */
    newtio.c_cc[VMIN]     = 0;    /* non-blocking read */
  
    /* 
       now clean the modem line and activate the settings for the port
    */
    tcflush(fd, TCIOFLUSH);
    rc = tcsetattr(fd,TCSANOW,&newtio);
    if (rc < 0)
      {
	ROS_ERROR("tcsetattr() error: %s", strerror(errno));
      }
    return rc;
  }
};

#endif
