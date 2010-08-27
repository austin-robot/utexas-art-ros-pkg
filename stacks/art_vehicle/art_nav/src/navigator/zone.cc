/*
 *  Navigator zone controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include "navigator_internal.h"
#include "Controller.h"
#include "course.h"
#include "safety.h"
#include "zone.h"

Zone::Zone(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  safety = new Safety(navptr, _verbose);
}

Zone::~Zone()
{
  delete safety;
}

// configuration method
void Zone::configure()
{
  // maximum speed when inside a zone
  zone_speed_limit = cf->ReadFloat(section, "zone_speed_limit", 3.0);
  ART_MSG(2, "\tspeed limit in a zone is %.1f m/s", zone_speed_limit);

  safety->configure(cf, section);
}

// go to next zone way-point
//
// returns: OK.
//
Controller::result_t Zone::control(pilot_command_t &pcmd)
{
  result_t result = OK;

  if (verbose >= 2)
    {
      ART_MSG(5, "Go to zone waypoint %s",
	      order->waypt[1].id.name().str);
    }

  // limit speed inside zones
  pcmd.velocity = fminf(pcmd.velocity, zone_speed_limit);

  // return desired heading for this cycle
  set_heading(pcmd);

  // check off way-point, if reached
  course->zone_waypoint_reached();

  // must always run this on the final command
  result = safety->control(pcmd);

  // TODO: implement obstacle evasion if Unsafe or Blocked

  trace("zone controller", pcmd, result);
  return result;
}

// reset all subordinate controllers
void Zone::reset(void)
{
  trace_reset("Zone");
  safety->reset();
}

// set heading to next way-point
void Zone::set_heading(pilot_command_t &pcmd)
{
  // STUB: this is just a place-holder that steers directly for the
  // next way-point.  The real version will use parking controller.

  // egocentric polar aim point
  Polar aim_polar =
    Coordinates::MapXY_to_Polar(order->waypt[1].map, estimate->pos);

  if (verbose >= 3)
    {
      ART_MSG(8, "target, current positions: (%.3f, %.3f), (%.3f, %.3f)",
	      order->waypt[1].map.x, order->waypt[1].map.y,
	      estimate->pos.px, estimate->pos.py);
      ART_MSG(8, "relative heading: %.3f radians, distance: %.3f meters",
	      aim_polar.heading, aim_polar.range);
    }

  pcmd.yawRate = aim_polar.heading/fmax(1.0,estimate->vel.px/2);
  trace("zone set_heading", pcmd);
}
