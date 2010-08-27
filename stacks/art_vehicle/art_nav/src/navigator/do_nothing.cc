/*
 *  Navigator example controller
 *
 *  Copyright (C) 2007, Mickey Ristroph
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

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
