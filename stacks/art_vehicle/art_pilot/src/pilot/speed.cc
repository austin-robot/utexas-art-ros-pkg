/*
 *  Copyright (C) 2005, 2007, 2009, 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART autonomous vehicle speed controller.

     @author Jack O'Quin
 */

#include <art/conversions.h>
#include <art_msgs/ArtHertz.h>
#include "speed.h"

/**
 @brief ART autonomous vehicle speed controller.
*/

namespace ControlMatrix
{

  // acceleration matrix dimensions
  static const int N_DELTAS = 13;
  static const int DELTA_0 = (N_DELTAS-1)/2; // middle row: delta == 0
  static const int N_SPEEDS = 6;

  // Velocity control
  typedef struct
  {
    float       brake_delta;
    float       throttle_delta;
  } accel_parms_t;

  // Acceleration matrix: rows are indexed according to the requested
  // speed delta in MPH, columns are indexed by current speed in MPH.
  // The main reason for making all this table-driven is so we can
  // adjust some values without affecting all the others.  
  //
  // NOTE: values in this table represent a percentage change per second,
  // so their effect builds up quickly.

  static const accel_parms_t accel_matrix[N_DELTAS][N_SPEEDS] =
    { /*              0        <=4       <=8       <=16      <=32      more  */
      /* -32 */ {{1000,-90}, {12,-30}, {12,-30}, {12,-40}, {12,-50}, {20,-70}},
      /* -16 */ {{1000,-60}, { 6,-20}, { 6,-20}, { 6,-30}, { 6,-40}, {10,-55}},
      /*  -8 */ {{1000,-40}, { 4,-15}, { 4,-15}, { 4,-20}, { 4,-30}, { 7,-40}},
      /*  -4 */ { {500,-20}, { 3,-10}, { 3,-10}, { 3,-10}, { 3,-20}, { 5,-25}},
      /*  -2 */ { {200,-10},  {2,-5},   {2,-5},   {2,-5},   {2,-10},  {3,-10}},
      /*  -1 */ { {100,-5},   {1,-2},   {1,-2},   {1,-2},   {1,-5},   {1,-5} },
      /*   0 */ { {100,0},    {0,0},    {0,0},    {0,0},    {0,0},    {0,0}  },
      /*   1 */ {{-100,1},    {-2,2},   {-2,2},   {-2,2},   {-2,5},   {-2,5} },
      /*   2 */ {{-200,2},    {-5,5},   {-5,5},   {-5,5},   {-5,10},  {-5,10}},
      /*   4 */ {{-300,3},   {-10,10}, {-10,10}, {-10,10}, {-10,20}, {-10,25}},
      /*   8 */ {{-500,4},   {-15,15}, {-15,15}, {-15,20}, {-15,30}, {-15,40}},
      /*  16 */ {{-1000,5},  {-20,20}, {-20,20}, {-20,30}, {-20,40}, {-20,55}},
      /*  32 */ {{-1000,6},  {-30,30}, {-30,30}, {-30,40}, {-30,50}, {-30,70}}
    };

  // If it turns out that the matrix indices should be different, adjust
  // the conversions in these functions.  The logarithmic intervals are
  // based on intuition.
  static inline int
  delta_row(float mph)
  {
    int row;
    static float row_limits[DELTA_0+1] = {0.0, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0};
    int mph_sign = 1;

    if (mph < 0)
      mph_sign = -1;

    float abs_mph = mph * mph_sign;

    for (row = 0; row < DELTA_0+1; ++row)
      {
        if (abs_mph <= row_limits[row])
          return DELTA_0 + mph_sign * row;
      }

    // if delta outside the table, use the first or last entry
    return DELTA_0 + mph_sign * DELTA_0;
  }

  static inline int
  speed_col(float mph)
  {
    int col;
    static float col_limits[N_SPEEDS] = {0.0, 4.0, 8.0, 16.0, 32.0, 64.0};

    for (col = 0; col < N_SPEEDS; ++col)
      {
        if (mph <= col_limits[col])
          return col;
      }

    // if speed beyond the table, use the last entry
    return N_SPEEDS-1;
  }
};

/** Acceleration matrix speed control constructor. */
SpeedControlMatrix::SpeedControlMatrix():
  SpeedControl(),
  velpid_(new Pid("speed", 2.0, 0.0, 32.0))
{
  // Allocate speed PID class with default parameters, then
  // configure control constants.
  //
  // Default PD parameters: make Kd proportional to cycle rate, higher
  // frequencies will yield lower deltaV per cycle.
  //
  reset();
}

/** SpeedControlMatrix destructor */
SpeedControlMatrix::~SpeedControlMatrix()
{}

/** Adjust speed to match goal.

    Generate brake and throttle changes from velocity PID controller
    via an acceleration matrix.

    @param speed absolute value of current velocity in m/sec
    @param error immediate goal minus speed
    @param brake_req -> previous brake request (input),
                        updated brake request (output).
    @param throttle_req -> previous throttle request (input),
                        updated throttle request (output).
*/
void SpeedControlMatrix::adjust(float speed, float error,
                                float *brake_req, float *throttle_req)
{
  using namespace ControlMatrix;

  float delta = velpid_->Update(error, speed);
  float delta_mph = mps2mph(delta);

  // index into the acceleration matrix
  int row = delta_row(delta_mph);
  int col = speed_col(mps2mph(speed));
  float brake_delta =
    (accel_matrix[row][col].brake_delta / art_msgs::ArtHertz::PILOT)
    / 100.0;
  float throttle_delta =
    (accel_matrix[row][col].throttle_delta / art_msgs::ArtHertz::PILOT)
    / 100.0;

  ROS_DEBUG("accel_matrix[%d][%d] contains {%.3f, %.3f}",
            row, col, brake_delta, throttle_delta);

  // Do not add braking unless nearly idle throttle was previously
  // requested, or throttle unless the brake is nearly off.
  if (*throttle_req > 0.0 && brake_delta > 0.0)
    brake_delta = 0.0;
  if (*brake_req > 0.0 && throttle_delta > 0.0)
    throttle_delta = 0.0;
  *brake_req += brake_delta;
  *throttle_req += throttle_delta;
}

/** Configure controller parameters. */
void SpeedControlMatrix::configure(art_pilot::PilotConfig &newconfig)
{
  velpid_->Configure(node_);
}

/** Reset speed controller. */
void SpeedControlMatrix::reset(void)
{
  velpid_->Clear();
}

/** PID speed control constructor.
 *
 *  Allocate brake and throttle PID classes with default parameters,
 *  then configure control constants.
 *
 *  Throttle gains came from Jesse Tannahill's pilot prototype.
 *  Brake gains selected by experiment.
 */
SpeedControlPID::SpeedControlPID():
  SpeedControl(),
  braking_(true),                       // begin with brake on
  brake_pid_(new Pid("brake", -0.2, -0.0002, -1.6, 1.0, 0.0, 5000.0)),
  throttle_pid_(new Pid("throttle", 0.12, 0.001, 0.54, 0.4, 0.0, 5000.0))
{
  reset();
}

/** PID speed control destructor. */
SpeedControlPID::~SpeedControlPID()
{};

/** Adjust speed to match goal.

    Generate brake and throttle changes directly from separate PID
    controllers.  Uses a state machine to avoid applying simultaneous
    brake and throttle.

    @param speed absolute value of current velocity in m/sec
    @param error immediate goal minus speed
    @param brake_req -> previous brake request (input),
                        updated brake request (output).
    @param throttle_req -> previous throttle request (input),
                        updated throttle request (output).
*/
void SpeedControlPID::adjust(float speed, float error,
                             float *brake_req, float *throttle_req)
{
  if (braking_)
    {
      // controlling with brake:
      *brake_req = brake_pid_->Update(error, speed);
      *throttle_req = 0.0;
      
      // If requesting brake off, switch to throttle control.
#if 1
      // Must check reported brake position, too.  Otherwise there
      // will be considerable overlap, applying throttle while the
      // brake is still on.  That can cause mechanical damage to the
      // transmission.
      if ((brake_position_ < EPSILON_BRAKE) && (*brake_req < EPSILON_BRAKE))
        {
          *brake_req = 0.0;             // brake off
          braking_ = false;             // using throttle now
          throttle_pid_->Clear();       // reset PID controller
        }
#else
      // Allow more overlap, to damp the oscillations that occur when
      // switching back and forth between throttle and brake.
      if ((*brake_req < EPSILON_BRAKE))
        {
          *brake_req = 0.0;             // brake off
          braking_ = false;             // using throttle now
          throttle_pid_->Clear();       // reset PID controller
        }
#endif
    }
  else
    {
      // controlling with throttle:
      *throttle_req = throttle_pid_->Update(error, speed);
      *brake_req = 0.0;

      // If requesting throttle off, switch to brake control.

      // Since throttle responds much faster than brake, it will reach
      // idle before the brake really starts engaging.  So, not
      // checking throttle_position_ here is an option, which reduces
      // latency when slowing down.
      if (*throttle_req < EPSILON_THROTTLE)
        {
          *throttle_req = 0.0;          // throttle off
          braking_ = true;              // using brake now
          brake_pid_->Clear();          // reset PID controller
        }
    }
}

/** Configure controller parameters. */
void SpeedControlPID::configure(art_pilot::PilotConfig &newconfig)
{
  brake_pid_->Configure(newconfig.brake_kp,
                        newconfig.brake_ki,
                        newconfig.brake_kd);
  throttle_pid_->Configure(newconfig.throttle_kp,
                           newconfig.throttle_ki,
                           newconfig.throttle_kd);
}

/** Reset speed controller. */
void SpeedControlPID::reset(void)
{
  brake_pid_->Clear();
  throttle_pid_->Clear();
}
