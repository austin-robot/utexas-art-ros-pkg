/*
 *  Odometry class implementation
 *
 *  Copyright (C) 2007, 2010 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <iostream>

#include <art/error.h>
#include <art_nav/odometry.h>
#include <art_map/rotate_translate_transform.h>

Odometry::Odometry(int _verbose)
{
  verbose = _verbose;
  memset(&curr_pos, 0, sizeof(curr_pos));
  time = 0.0;

  listsize=20;
  
  poselist = new Position::Pose3D[listsize];  
  timelist = new double[listsize];
  
  list_start=list_curr=0;
  list_end=listsize-1;
}

Odometry::~Odometry()
{
  delete poselist;
  delete timelist;
}

int Odometry::configure()
{
  return 0;
}


// Subscribe to odometry device
int Odometry::subscribe()
{
  return 0;
}

