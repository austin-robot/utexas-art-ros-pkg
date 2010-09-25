/* -*- mode: C++ -*-
 *
 *  Digital filter functions
 *
 *  Copyright (C) 2008 Austin Robot Technology                    
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _FILTER_H
#define _FILTER_H

/**  @file
   
     @brief digital filter functions
 */

/**
 * Exponentially weighted moving average filter.  Transfer function:
 *
 *	y[k] = a*x[k] + (1-a)*y[k-1]
 *
 * The smoothing factor, "a", is how much weight to assign the latest
 * observation.  Values of "a" close to one have less of a smoothing
 * effect and give greater weight to recent changes in the data, while
 * values of "a" closer to zero have a greater smoothing effect and
 * are less responsive to recent changes.[1]
 *
 * [1] http://en.wikipedia.org/wiki/Exponential_smoothing 
 */
static inline float EWMA_filter(float a, float xk, float yk1)
{
  return a*xk + (1-a)*yk1;
}

/**
 * Exponential smoothing factor corresponding to N periods.
 */
static inline float EWMA_smoothing(int N)
{
  return 2.0/(N + 1.0);
}

#endif /* _FILTER_H */
