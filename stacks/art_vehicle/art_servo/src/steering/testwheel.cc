/*
 *  ART steering self-test
 *
 *  Copyright (C) 2008, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <errno.h>

#include <ros/ros.h>

#include "testwheel.h"
#include "devsteer.h"
#include "dtimer.h"

/////////////////////////////////////////////////////////////////
// public methods
/////////////////////////////////////////////////////////////////


#define CLASS "testwheel"

testwheel::testwheel(const boost::shared_ptr<devsteer> &_dev):
  state(Begin),
  dev(_dev),
  timer(new DriverTimer())
{}

testwheel::~testwheel()
{}

int testwheel::Configure()
{
  // use private node handle to get parameters
  ros::NodeHandle mynh("~");

  test_wheel = true;
  mynh.getParam("test_wheel", test_wheel);
  if (!test_wheel)
    return 0;
  ROS_INFO("wheel test configured");

  test_angle = 3.0;
  mynh.getParam("test_angle", test_angle);
  ROS_INFO("wheel test angle is %.1f degrees", test_angle);

  timeout = 2.0;
  mynh.getParam("test_timeout", timeout);
  ROS_INFO("wheel test timeout is %.1f seconds", timeout);

  tolerance = 2.0;
  mynh.getParam("test_tolerance", tolerance);
  ROS_INFO("wheel test tolerance is %.1f degrees", tolerance);

  return 0;
}

// Run steering wheel self-test
//
// Called once per cycle until test completed.
//
// returns:	0 if test still running
//		< 0 if test failed (negative of errno value),
//		> 0 if test completed successfully,
//
int testwheel::Run(float steering_angle)
{
  int result = 0;			// test still running

  switch(state)
    {
    case Begin:
      {
	if (test_wheel)			// test configured?
	  {
	    start_angle = steering_angle;
	    target_angle = test_angle + steering_angle;
	    int rc = dev->steering_absolute(target_angle);
	    if (rc == 0)
	      {
		state = Move;
                ROS_INFO("beginning steering self-test");
		timer->Start(timeout);
	      }
	    else
	      {
		ROS_ERROR("steering self-test failed (%s)", strerror(rc));
		result = -rc;
	      }
	  }
	else				// nothing to do
	  {
	    ROS_INFO("steering self-test: nothing to do, returning");
	    result = 1;			// success
	  }
	break;
      }
    case Move:
      {
	if (timer->Check())
	  {
	    ROS_ERROR("steering self-test move failed (timeout)");
	    result = -EBUSY;
	  }
	else if (fabs(steering_angle - target_angle) <= tolerance)
	  {
	    // go back to starting position
	    int rc = dev->steering_absolute(start_angle);
	    if (rc == 0)
	      {
		state = Back;
                ROS_INFO("self-test angle reached, turning back");
		timer->Start(timeout);
	      }
	    else
	      {
		ROS_ERROR("steering self-test failed (%s)", strerror(rc));
		result = -rc;
	      }
	  }
	break;
      }
    case Back:
      {
	if (timer->Check())
	  {
	    ROS_ERROR("steering self-test move back failed (timeout)");
	    result = -EBUSY;
	  }
	else if (fabs(steering_angle - start_angle) <= tolerance)
	  {
	    ROS_INFO("steering self-test successful");
	    result = 1;			// success
	  }
	break;
      }
    case Done:
      {
	result = 1;			// nothing left to do
	break;
      }
    default:
      // invalid state -- should not happen
      ROS_ERROR("steering self-test failed (invalid state)");
      result = -EINVAL;
    }

  if (result != 0)
    state = Done;

  return result;
}
