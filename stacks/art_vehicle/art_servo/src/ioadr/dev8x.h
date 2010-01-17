/*
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file

     National Control Devices ioadr8x device interface.

     \author Jack O'Quin
 */

#ifndef _DEV8X_H_
#define _DEV8X_H_

#define IOADR_MAX_INPUTS  8
#define IOADR_MAX_OUTPUTS 8

#include <stdio.h>
#include <stdint.h>

#define DBG(format,args...) ROS_DEBUG(format, ## args)

#define BUFSIZE 8			// dev8x port buffer size

// One per IOADR8x hardware device.
class dev8x
{
public:

  dev8x(const char *pn);
  ~dev8x() {};

  int	query_relays(uint8_t *relays);
  int   read_8bit_port(int ch, int *data);
  int   read_10bit_port(int ch, int *data);
  bool	relays_busy(void);
  int	set_relays(uint8_t bitmask);
  int	setup();
  int	shutdown();
  uint16_t sim_sawtooth(int ch, uint16_t limit);

private:
  int	configure_port(int cflags, int iflags);

  // device state:
  enum {UNREADY, READY, RELAY_WAIT} state;

  double relay_wait_time;		// time we last set relays
  //#define  MIN_RELAY_WAIT 0.16	// minimum wait in seconds
  #define  MIN_RELAY_WAIT 0.08		// minimum wait in seconds

  // device I/O control
  char	port_name[FILENAME_MAX];	// serial port name
  bool	have_tty;			// have real tty, not /dev/null
  uint8_t buffer[BUFSIZE];		// IOADR8x device buffer
  int	fd;				// file descriptor

  // simulated analog ports and relays (when have_tty == false)
  uint16_t sim_port[IOADR_MAX_OUTPUTS];
  uint8_t  sim_relays;
};

#endif // _DEV8X_H_
