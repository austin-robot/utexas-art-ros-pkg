/*
 *  Description: global ART infinity definitions
 *
 *  Copyright (C) 2009 Austin Robot Technology                    
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _INFINITY_H_
#define _INFINITY_H_

#include <math.h>

/** @brief global ART infinity definitions.  These constants represent
 *  effectively infinite distances, times, etc.
 */
namespace Infinite
{
  const float distance = 1000000.0;	//< far off distance, in meters
  const float time = 1000000.0;		//< forever, in seconds
}

#endif // _INFINITY_H_ //
