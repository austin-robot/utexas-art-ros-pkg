/* -*- mode: C++ -*- 
 *
 *  Description:  Quicksilver SilverLode hardware interface
 *
 *  Copyright (C) 2011 Austin Robot Technology                    
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _SILVERLODE_H_
#define _SILVERLODE_H_

namespace silverlode
{

namespace isw
{
  enum isw
    {
      index_found =       0x0001,
      last_calc_zero =    0x0002,
      last_calc_pos =     0x0004,
      last_calc_neg =     0x0008,
      io_1_high =         0x0010,
      io_2_high =         0x0020,
      io_3_high =         0x0040,
      temp_driver_en =    0x0080,
      moving_error =      0x0100,
      holding_error =     0x0200,
      halt_sent =         0x0400,
      input_found =       0x0800,
      delay_exhausted =   0x1000,
      over_voltage =      0x2000,
      low_voltage =       0x4000,
    };

} // namespace isw

} // namespace silverlode

#endif // _SILVERLODE_H_
