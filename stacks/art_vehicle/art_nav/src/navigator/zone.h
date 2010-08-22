//
// Navigator zone controller
//
//  Copyright (C) 2007 Austin Robot Technology
//  All Rights Reserved. Licensed Software.
//
//  This is unpublished proprietary source code of Austin Robot
//  Technology, Inc.  The copyright notice above does not evidence any
//  actual or intended publication of such source code.
//
//  PROPRIETARY INFORMATION, PROPERTY OF AUSTIN ROBOT TECHNOLOGY
//
//  $Id$
//
//  Author: Jack O'Quin
//

#ifndef __ZONE_HH__
#define __ZONE_HH__

class Safety;

class Zone: public Controller
{
public:

  Zone(Navigator *navptr, int _verbose);
  ~Zone();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:

  // .cfg variables
  float zone_speed_limit;

  Safety *safety;

  void set_heading(pilot_command_t &pcmd);
};

#endif // __ZONE_HH__
