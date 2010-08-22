/*
 * Odometry class interface for observers driver.
 *
 *  Copyright (C) 2007 Austin Robot Technology
 *  All Rights Reserved. Licensed Software.
 *
 *  This is unpublished proprietary source code of Austin Robot
 *  Technology, Inc.  The copyright notice above does not evidence any
 *  actual or intended publication of such source code.
 *
 *  PROPRIETARY INFORMATION, PROPERTY OF AUSTIN ROBOT TECHNOLOGY
 *
 *  If this is ever released publicly, the requirements of the GPL
 *  will apply, due to Player header and library dependencies.
 *
 *  $Id$
 *
 *  Authors: Jack O'Quin
 */

#ifndef __ODOMETRY_HH__
#define __ODOMETRY_HH__

class Odometry
{
public:

  // public data
  player_position2d_data_t curr_pos;	// current position
  double time;				// last message timestamp
    
  Odometry(int _verbose);
  ~Odometry();

  int configure(ConfigFile* cf, int section);
  bool match(player_msghdr *hdr, void *data);
  int subscribe(QueuePointer myQ);
  void unsubscribe()
  {
    if (have_odom)
      odom->Unsubscribe(myqueue);
  }

  player_pose2d_t* poselist;
  double *timelist;
  
  unsigned int listsize,list_start,list_end, list_curr;

  bool have_odom;

private:


  // .cfg variables:
  int verbose;				// log level verbosity

  QueuePointer myqueue;		// subscriber's message queue

  // odometry interface
  Device *odom;				// odometry device
  player_devaddr_t odom_addr;		// odometry device address
};

#endif
