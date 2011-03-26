/*
 *  ART steering self-test
 *
 *  Copyright (C) 2008 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */


/** @{ */
/** @defgroup driver_steering steering servo driver
 @ingroup driver_hardware
 @brief ART steering servo driver

Configuration options for steering servo device:

- test_wheel (bool)
  - true if driver should test wheel movement before normal operation
  - default: true

- test_timeout (float)
  - test fails if goal not reached in timeout seconds
  - default: 2 sec

- test_angle (float)
  - amount to move wheel relative to starting position (degrees)
  - default: 2 degrees left

- test_tolerance (float)
  - minimum accuracy for successful test (degrees)
  - default: 0.5 degrees
*/
/** @} */


#ifndef _TESTWHEEL_H_
#define _TESTWHEEL_H_

class devsteer;
class DriverTimer;

class testwheel
{
public:

  testwheel(const boost::shared_ptr<devsteer> &_dev);
  ~testwheel();

  int	Configure();
  int	Run(float steering_angle);

  // configuration options

 private:

  typedef enum
    {
      Begin,				// begin wheel test
      Move,				// turn wheel to test position
      Back,				// turn wheel back
      Done				// test completed
    } state_t;

  state_t state;			// current state of test
  boost::shared_ptr<devsteer> dev;
  boost::shared_ptr<DriverTimer> timer;

  // configuration options
  bool	test_wheel;
  double test_angle;
  double timeout;
  double tolerance;

  float start_angle;
  float target_angle;
};

#endif // _TESTWHEEL_H_
