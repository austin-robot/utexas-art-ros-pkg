//
// Navigator examplex controller
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

#include "navigator_internal.h"
#include "Controller.h"
#include "course.h"
#include "do_nothing.h"

DoNothing::DoNothing(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
}

DoNothing::~DoNothing()
{
}

void DoNothing::configure(ConfigFile* cf, int section)
{
}

Controller::result_t DoNothing::control(pilot_command_t &pcmd)
{
  trace("do_nothing controller", pcmd);
  return OK;				// always successful
}

// reset all subordinate controllers
void DoNothing::reset(void)
{
  trace_reset("DoNothing");
}
