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

#ifndef __REAL_ZONE_HH__
#define __REAL_ZONE_HH__

class Halt;
class VoronoiZone;

class RealZone: public Controller
{
public:

  RealZone(Navigator *navptr, int _verbose);
  ~RealZone();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:

  // .cfg variables

  Halt *halt;
  VoronoiZone *voronoi;
};

#endif // __REAL_ZONE_HH__
