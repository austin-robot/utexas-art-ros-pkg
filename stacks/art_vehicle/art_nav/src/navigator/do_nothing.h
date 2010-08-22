//
// Navigator example controller
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
//  Author: Mickey Ristroph
//

#ifndef __DO_NOTHING_HH__
#define __DO_NOTHING_HH__

class DoNothing: public Controller
{
public:

  DoNothing(Navigator *navptr, int _verbose);
  ~DoNothing();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd);
  void reset(void);
};

#endif // __DO_NOTHING_HH__
