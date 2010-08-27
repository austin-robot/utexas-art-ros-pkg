/* -*- mode: C++ -*-
 *
 *  Navigator zone controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __ZONE_HH__
#define __ZONE_HH__

class Safety;

class Zone: public Controller
{
public:

  Zone(Navigator *navptr, int _verbose);
  ~Zone();
  void configure();
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:

  // .cfg variables
  float zone_speed_limit;

  Safety *safety;

  void set_heading(pilot_command_t &pcmd);
};

#endif // __ZONE_HH__
