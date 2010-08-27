/* -*- mode: C++ -*-
 *
 *  Finite state machine interface
 *
 *  Copyright (C) 2007, Mickey Ristroph
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __DO_NOTHING_HH__
#define __DO_NOTHING_HH__

class DoNothing: public Controller
{
public:

  DoNothing(Navigator *navptr, int _verbose);
  ~DoNothing();
  void configure();
  result_t control(pilot_command_t &pcmd);
  void reset(void);
};

#endif // __DO_NOTHING_HH__
