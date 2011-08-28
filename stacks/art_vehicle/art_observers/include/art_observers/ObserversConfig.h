/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     Hand-written configuration class for observers.

     This is compatible with a dynamic reconfigure interface, so one
     could easily be provided, if needed.

     @author Jack O'Quin

 */

#ifndef _OBSERVERS_CONFIG_H_
#define _OBSERVERS_CONFIG_H_

namespace art_observers
{

  // Hand-written configuration class for observers.
  class ObserversConfig
  {
  public:
    ObserversConfig() {};

    std::string map_frame_id;		///< frame ID of map
    std::string robot_frame_id;		///< frame ID of robot
    double	max_range;		///< maximum obstacle range
  };

}; // namespace art_observers

#endif // _OBSERVERS_CONFIG_H_
