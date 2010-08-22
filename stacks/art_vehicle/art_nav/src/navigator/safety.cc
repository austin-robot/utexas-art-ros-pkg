#include "safety.h"
#include "Controller.h"
#include "obstacle.h"
#include <art/steering.h>
#include <art/Safety.h>

Safety::Safety(Navigator *navptr, int _verbose, int mode):
  Controller(navptr, _verbose)
  {
    _mode=mode; // 0 for normal, 1 for zone, 2 for parking
    safety = new Safety_Distance(_verbose);
  }

Safety::~Safety()
  {
    delete safety;
  }

void Safety::configure(ConfigFile* cf, int section)
{

  if (_mode==0)
    {
      far_slow_ratio = cf->ReadFloat(section, "far_slow_ratio", 0.75);
      near_slow_ratio = cf->ReadFloat(section, "near_slow_ratio", 0.5);
      
      far_safety_time = cf->ReadFloat(section, "far_safety_time", 3);
      near_safety_time = cf->ReadFloat(section, "near_safety_time", 2);
      collision_safety_time = cf->ReadFloat(section, "collision_safety_time", 1);
    }
  else if (_mode==1)
    {
      far_slow_ratio = cf->ReadFloat(section, "zone_far_slow_ratio", 0.75);
      near_slow_ratio = cf->ReadFloat(section, "zone_near_slow_ratio", 0.5);
      
      far_safety_time = cf->ReadFloat(section, "zone_far_safety_time", 3);
      near_safety_time = cf->ReadFloat(section, "zone_near_safety_time", 2);
      collision_safety_time = cf->ReadFloat(section, "zone_collision_safety_time", 1.0);
    }
  else{
    far_slow_ratio = cf->ReadFloat(section, "parking_far_slow_ratio", 0.75);
    near_slow_ratio = cf->ReadFloat(section, "parking_near_slow_ratio", 0.5);
    
    far_safety_time = cf->ReadFloat(section, "parking_far_safety_time", 1.5);
    near_safety_time = cf->ReadFloat(section, "parking_near_safety_time", 1);
    collision_safety_time = cf->ReadFloat(section, "parking_collision_safety_time", 1.0);
  }
  
  safety_speed = cf->ReadFloat(section, "safety_speed", 2.0);
  
  ART_MSG(2, "Safety is in mode %d",_mode);
  ART_MSG(2, "\tMinimum nonzero safety speed is %.3f m/s", safety_speed);
  
  ART_MSG(2, "\tMultiply speed by %.2f if collision in %f seconds", 
	  far_slow_ratio, far_safety_time);
  ART_MSG(2, "\tMultiply speed by %.2f if collision in %f seconds", 
	  near_slow_ratio, near_safety_time);
  ART_MSG(2, "\tBrake immediately if collision in %f seconds", 
	  near_safety_time);
}

// safety controller -- always runs last in every road state
//
// This controller is stateless, with no side-effects.  It may be
// called more than once in a single cycle to explore avoidance
// alternatives.  It may reduce the requested speed, but requests no
// heading changes.
//
// returns:
//	OK, if no immediate danger;
//	Caution, if obstacle far away and reducing speed;
//	Beware, if obstacle near and reducing speed;
//	Unsafe, if collision imminent and stopping;
//	Blocked, if stopped and unable to proceed.
//
Controller::result_t Safety::control(pilot_command_t &retpcmd)
{
  pilot_command_t pcmd=retpcmd;
  if (navdata->reverse)
    pcmd.velocity*=-1;

  // Look to see if current velocity is going to hit anything.  If so
  // slam on brakes.
  player_pose2d_t tmp=estimate->pos;
  float min_val=Infinite::distance;
    


  float ret_val=
    safety->minimum_obstacle_to_arc(tmp,
				    estimate->vel.px*collision_safety_time,
				    estimate->vel.pa*collision_safety_time,
				    obstacle->lasers->all_obstacle_list,
				    tmp);
  if (Epsilon::equal(ret_val,0.0))
    {
      return halt_immediately(retpcmd);
    }    

  // While our current trajectory will hit something nearby, keep
  // reducing the speed.
  bool collision_obst=false;
  bool collision;
  result_t result = OK;

  do {
    tmp=estimate->pos;
    min_val=Infinite::distance;
    
    collision=false;
    
    float ret_val=
      safety->minimum_obstacle_to_arc(tmp,
				      pcmd.velocity*near_safety_time,
				      pcmd.yawRate*near_safety_time,
				      obstacle->lasers->all_obstacle_list,
				      tmp);
    if (Epsilon::equal(ret_val,0.0))
      {
	// collision imminent
	collision_obst=true;
	collision=true;
	result = Beware;
	if (pcmd.velocity > safety_speed/near_slow_ratio)
	  {
	    if (verbose >= 3)
	      ART_MSG(2, "Collision potential, multiply by %.3f: "
		      "from %.3f, %.3f",
		      near_slow_ratio, pcmd.velocity, pcmd.yawRate);
	    pcmd.velocity*=near_slow_ratio;
	    //pcmd.yawRate*=near_slow_ratio;
	  }
	else if (pcmd.velocity > safety_speed)
	  {
	    float ratio=safety_speed/pcmd.velocity;
	    if (verbose >= 3)
	      ART_MSG(2, "Collision potential, multiply by %.3f: "
		      "from %.3f, %.3f",
		      ratio, pcmd.velocity, pcmd.yawRate);
	    pcmd.velocity*=ratio;
	    //pcmd.yawRate*=ratio;
	  }
	else
	  {
	    return halt_immediately(retpcmd);
	  }
      }
  } while (collision);
  
  if (collision_obst)
    {
      retpcmd=pcmd;
      retpcmd.velocity=fabsf(retpcmd.velocity);
      trace("safety controller", retpcmd, result);
      return result;
    }

  // If something out ahead, slow down a bit.  

  // tmp should already be at point near_safety_time seconds ahead of
  // current position

  ret_val=
    safety->minimum_obstacle_to_arc(tmp,
				    pcmd.velocity*(far_safety_time-near_safety_time),
				    pcmd.yawRate*(far_safety_time-near_safety_time),
				    obstacle->lasers->all_obstacle_list,
				    tmp);
  if (Epsilon::equal(ret_val,0.0))
    {
      // no immediate collision, but danger ahead
      result = Caution;
      if (pcmd.velocity > safety_speed/far_slow_ratio)
	{
	  if (verbose >= 3)
	    ART_MSG(2, "Danger ahead, slowing by %.3f times: from %.3f, %.3f ", 
		    far_slow_ratio, pcmd.velocity, pcmd.yawRate);
	  pcmd.velocity=pcmd.velocity*far_slow_ratio;
	  //pcmd.yawRate=pcmd.yawRate*far_slow_ratio;
	}
      else if (pcmd.velocity > safety_speed)
	{
	  float ratio=safety_speed/pcmd.velocity;
	  if (verbose >= 3)
	    ART_MSG(2, "Danger ahead, slowing by %.3f times: from %.3f, %.3f ", 
		    ratio, pcmd.velocity, pcmd.yawRate);
	  pcmd.velocity=pcmd.velocity*ratio;
	  //pcmd.yawRate=pcmd.yawRate*ratio;
	}
    }    
  retpcmd=pcmd;
  retpcmd.velocity=fabsf(retpcmd.velocity);
  trace("safety controller", pcmd, result);
  return result;
}

// return result and Pilot command for an immediate halt
//
// returns:
//	Unsafe, if car still moving
//	Blocked, if car already stopped
Controller::result_t Safety::halt_immediately(pilot_command_t &pcmd)
{
  if (verbose >= 3)
    ART_MSG(2, "Collision imminent, halting: from %.3f, %.3f",
	    pcmd.velocity, pcmd.yawRate);
  pcmd.velocity=0.0;
  //pcmd.yawRate=0.0;

  // return Blocked if already stopped and still requesting halt
  result_t result = Unsafe;
  if (navdata->stopped)
    result = Blocked;
  trace("safety controller", pcmd, result);
  return result;
}
