/*
 *  Description: GPS information published by Applanix driver.
 *
 *  Copyright (C) 2005, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 */

#ifndef _APPLANIX_INFO_H
#define _APPLANIX_INFO_H

namespace applanix_info
{

  typedef struct {
    double  gps_latitude;               // degrees (south is negative)
    double  gps_longitude;              // degrees (west is negative)
  } gps_info;

} // namespace applanix

#endif
