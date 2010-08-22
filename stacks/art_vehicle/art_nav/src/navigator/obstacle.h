/*
 *  Description:  Navigator obstacle class
 *
 *  Copyright Austin Robot Technology                    
 *  All Rights Reserved. Licensed Software.
 *
 *  This is unpublished proprietary source code of Austin Robot
 *  Technology, Inc.  The copyright notice above does not evidence any
 *  actual or intended publication of such source code.
 *
 *  PROPRIETARY INFORMATION, PROPERTY OF AUSTIN ROBOT TECHNOLOGY
 *
 *  $Id$
 */

#ifndef _OBSTACLE_HH_
#define _OBSTACLE_HH_

#include <vector>
#include <art/observers.h>
#include <art/lasers.h>
#include "ntimer.h"


/** @brief Navigator obstacle class. */
class Obstacle
{
 public:

  /** @brief Constructor */
  Obstacle(Navigator *_nav, int _verbose);

  /** @brief Destructor */
  ~Obstacle()
  {
    if (lasers)
      delete lasers;
    delete blockage_timer;
  };

  /** @brief update blockage_timer (do once per run cycle)
   *  
   *  @returns true if timeout reached
   */
  bool blockage_timeout(void)
  {
    return blockage_timer->Check();
  }

  /** @brief mark car blocked, (re)start timeout. */
  void blocked(void)
  {
    if (verbose >= 5)
      ART_MSG(3, "Starting blockage timer");
    blockage_timer->Start(blockage_timeout_secs);
  }

  /** @brief is there a car approaching from ahead in our lane? */
  bool car_approaching();

  /** @brief set configuration variables. */
  void configure(ConfigFile* cf, int section);

  /** @brief return distance to closest obstacle ahead in course plan */
  float closest_ahead_in_plan(void);

  /** Return distances of closest obstacles ahead and behind in a lane. */
  void closest_in_lane(const poly_list_t &lane, float &ahead, float &behind);

  /** @brief return true if class initialized */
  bool initialized(void) {return lasers->have_ranges;};

  /** @brief handle intersection driver message
   *
   *  TEMPORARY: bridge old driver to observers interface
   *
   *  Called from the driver ProcessMessage() handler when new
   *  intersection data arrive.
   *
   * @param hdr the player message header pointer
   * @param opaque pointer to the opaque data struct in the player
   * message queue.  Must copy the data before returning.
   *
   * @returns 0 if message OK, -1 if invalid.
   */
  int intersection_message(player_msghdr *hdr,
			   player_opaque_data_t *opaque);


  /** @brief maximum scan range accessor. */
  float maximum_range(void) {return lasers->max_range;}

  /** @brief return current observation state */
  Observation observation(ObserverID::observer_id_t oid)
  {
    return obstate.obs[oid];
  }

  /** @brief return true when observer reports clear to go */
  bool observer_clear(ObserverID::observer_id_t oid)
  {
#if 1
    bool clear = obstate.obs[oid].clear && obstate.obs[oid].applicable;
#else
    bool clear = obstate.obs[oid].clear;
#endif
    // if waiting on observers, reset the blockage_timer
    if (!clear)
      blocked();
    return clear;
  }

  /** @brief handle observers driver message
   *
   *  Called from the driver ProcessMessage() handler when new
   *  observers data arrive.
   *
   * @param hdr the player message header pointer
   * @param obs_msg pointer to the observer state message struct in
   * the player message queue.  Must copy the data before returning.
   */
  void observers_message(player_msghdr *hdr,
			 observers_state_msg_t *obs_msg);

  /** @brief return true when observer reports passing lane clear */
  bool passing_lane_clear(void);

  /** @brief reset obstacles. */
  void reset(void);

  /** @brief mark car not blocked, cancel timeout. */
  void unblocked(void)
  {
    if (verbose >= 5)
      ART_MSG(3, "Canceling blockage timer");
    blockage_timer->Cancel();
  }
  
  /** @brief update blockage timer state. */
  void update_blockage_state(void)
  {
    if (navdata->stopped != was_stopped ||
	!navdata->stopped)
      {
	if (navdata->stopped)
	  blocked();
	else
	  unblocked();
	if (verbose >= 4 &&
	    navdata->stopped != was_stopped)
	  ART_MSG(8, "vehicle %s moving",
		  (navdata->stopped? "stopped": "started"));
	was_stopped = navdata->stopped;
      }
  }

  Lasers* lasers;

 private:

  // laser data state
  float max_range;			//< maximum scan range

  // observers data
  observers_state_msg_t obstate;	//< current observers state
  double observers_time;		//< timestamp of observers data
  observers_state_msg_t prev_obstate;	//< previous observers state

  // blockage timer
  NavTimer *blockage_timer;
  bool was_stopped;			// previous cycle's stop state

  // .cfg variables
  float  blockage_timeout_secs;
  float  lane_width_ratio;
  float  lane_scan_angle;
  float  max_obstacle_dist;
  float  min_approach_speed;
  bool   offensive_driving;

  // constructor parameters
  int verbose;				// message verbosity level
  Navigator *nav;			// internal navigator class

  // convenience pointers to Navigator class data
  PolyOps* pops;			// polygon operations class
  Course* course;			// course planning class
  Order *order;				// current commander order
  nav_state_msg_t *navdata;		// current navigator state data
  Odometry *odom;			// current odometry position
  player_position2d_data_t *estimate;	// estimated control position

  // returns true if obstacle is within the specified lane
  bool in_lane(MapXY location, const poly_list_t &lane, int start_index);

  // conversions between laser scan indices and bearings
  float laser_scan_bearing(unsigned index, const player_laser_data_t &scan)
  {
    return scan.min_angle + index * scan.resolution;
  }
  unsigned laser_scan_index(float bearing, const player_laser_data_t &scan)
  {
    unsigned index =
      (unsigned) rintf((bearing - scan.min_angle) / scan.resolution);
    if (index < 0)
      index = 0;
    else if (index > scan.ranges_count)
      index = scan.ranges_count;
    return index;
  }
};

#endif // _OBSTACLE_HH_
