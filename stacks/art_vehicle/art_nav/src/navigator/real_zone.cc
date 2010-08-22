//
// Navigator real zone controller
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
#include "halt.h"
#include "real_zone.h"
#include "voronoi_zone.h"

RealZone::RealZone(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  halt = new Halt(navptr, _verbose);
  voronoi = new VoronoiZone(navptr, _verbose);
}

RealZone::~RealZone()
{
  delete halt;
  delete voronoi;
}

// configuration method
void RealZone::configure(ConfigFile* cf, int section)
{
  voronoi->configure(cf, section);
  halt->configure(cf, section);
}

// go to next zone way-point
//
// returns: OK.
//
Controller::result_t RealZone::control(pilot_command_t &pcmd)
{
  result_t result = OK;

  if(order->waypt[0].id.seg == order->waypt[1].id.seg &&
     (order->waypt[1].is_spot || order->waypt[1].is_perimeter)) {
    result = voronoi->control(pcmd);
  } else {
    halt->control(pcmd);
    result = NotApplicable;
  }

  trace("real_zone controller", pcmd, result);

  return result;
}

// reset all subordinate controllers
void RealZone::reset(void)
{
  trace_reset("RealZone");
  halt->reset();
  voronoi->reset();
}
