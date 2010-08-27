/*
 *  Navigator real zone controller
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
