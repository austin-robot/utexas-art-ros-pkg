/* -*- mode: C++ -*-
 *
 *  Navigator real zone controller
 *
 *  Copyright (C) 2007, Mickey Ristroph
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __REAL_ZONE_HH__
#define __REAL_ZONE_HH__

class Halt;
class VoronoiZone;

class RealZone: public Controller
{
public:

  RealZone(Navigator *navptr, int _verbose);
  ~RealZone();
  void configure();
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:

  // .cfg variables

  Halt *halt;
  VoronoiZone *voronoi;
};

#endif // __REAL_ZONE_HH__
