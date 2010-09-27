/*
 *  ROS stage simulation for ART autonomous vehicle
 *
 *  (based on stage/src/stageros.cpp Willow Garage ROS node)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2009, Austin Robot Technology
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  $Id$
 */


/**  \file
 
     ROS node for simulating the ART autonomous vehicle with Stage..

     \todo handle multiple robots (code partially exists from stageros.cpp)

     \todo map multiple lasers to their own frame and topic names

     \todo terminate when stage window terminates
 
     \author Brian Gerkey, Jack O'Quin

 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>


// libstage
#include <stage.hh>

// roscpp
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <roslib/Clock.h>
#include "tf/transform_broadcaster.h"

#include <art/frames.h>                 // ART vehicle frames of reference

#include "vehicle_model.h"

#define USAGE "artstage [-g] [ <worldfile> ]"

// ROS relative topic names
#define FRONT_LASER "front_sick"

// Our node
class StageNode
{
  private:
    // Messages that we'll send or receive
    sensor_msgs::LaserScan *laserMsgs;
    roslib::Clock clockMsg;

    // roscpp-related bookkeeping
    ros::NodeHandle n_;

    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelLaser *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;
    std::vector<ros::Publisher> laser_pubs_;
    ros::Publisher clock_pub_;

    // ART vehicle dynamics simulation
    std::vector<ArtVehicleModel *> vehicleModels_;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(Stg::Model* mod, StageNode* node);

    static bool s_update(Stg::World* world, StageNode* node)
    {
      node->WorldCallback();
      // We return false to indicate that we want to be called again (an
      // odd convention, but that's the way that Stage works).
      return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID);
    const inline char *mapName(const std::string name, size_t robotID)
    {return mapName(name.c_str(), robotID);}

    tf::TransformBroadcaster tf;

    // Current simulation time
    ros::Time sim_time;

  public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    StageNode(int argc, char** argv, bool gui, const char* fname);
    ~StageNode();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();
    
    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // The main simulator object
    Stg::World* world;
};

// since stage_art is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapName(const char *name, size_t robotID)
{
  if (positionmodels.size() > 1)
  {
    static char buf[100];
    snprintf(buf, sizeof(buf), "/robot_%lu/%s", (long unsigned) robotID, name);
    return buf;
  }
  else
    return name;
}

void
StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
  if (dynamic_cast<Stg::ModelLaser *>(mod))
    node->lasermodels.push_back(dynamic_cast<Stg::ModelLaser *>(mod));

#if 0  // newer version
  if (dynamic_cast<Stg::ModelPosition *>(mod))
    node->positionmodels.push_back(dynamic_cast<Stg::ModelPosition *>(mod));
#else  // earlier version
  // only use the first position model
  if (dynamic_cast<Stg::ModelPosition *>(mod)
      && node->positionmodels.size() == 0)
    node->positionmodels.push_back(dynamic_cast<Stg::ModelPosition *>(mod));
#endif
}

StageNode::StageNode(int argc, char** argv, bool gui, const char* fname)
{
  ROS_INFO_STREAM("libstage version " << Stg::Version());

  this->sim_time.fromSec(0.0);

  // We'll check the existence of the world file, because libstage doesn't
  // expose its failure to open it.  Could go further with checks (e.g., is
  // it readable by this user).
  struct stat s;
  if(stat(fname, &s) != 0)
  {
    ROS_FATAL("The world file %s does not exist.", fname);
    ROS_BREAK();
  }

  // initialize libstage
  Stg::Init( &argc, &argv );

  if(gui)
    this->world = new Stg::WorldGui(800, 700, "Stage (ART)");
  else
    this->world = new Stg::World();

  
  // Apparently an Update is needed before the Load to avoid crashes on
  // startup on some systems.
  this->UpdateWorld();
  this->world->Load(fname);

  // We add our callback here, after the Update, so avoid our callback
  // being invoked before we're ready.
  this->world->AddUpdateCallback((Stg::stg_world_callback_t)s_update, this);

  this->world->ForEachDescendant((Stg::stg_model_callback_t)ghfunc, this);

  size_t numRobots = positionmodels.size();
  ROS_INFO_STREAM("found " << numRobots << " position model(s) in the file");

  /// \todo support more than one simulated robot
  if (numRobots != 1)
  {
#if 0
    ROS_FATAL("artstage currently only simulates one position model");
    ROS_BREAK();
#endif
    size_t numRobots = positionmodels.size();
    ROS_INFO("found %u position/laser pair%s in the file", 
             (unsigned int)numRobots, (numRobots==1) ? "" : "s");
  }

  this->laserMsgs = new sensor_msgs::LaserScan[numRobots];

  for (size_t r = 0; r < numRobots; r++)
  {
    // TODO assign a namespace to each robot (from stage??)
    vehicleModels_.push_back(new ArtVehicleModel(positionmodels[r], &tf, ""));
  }
}


// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int
StageNode::SubscribeModels()
{
  n_.setParam("/use_sim_time", true);

  for (size_t r = 0; r < this->positionmodels.size(); r++)
  {
    if(this->lasermodels[r])
    {
      this->lasermodels[r]->Subscribe();
    }
    else
    {
      ROS_ERROR("no laser");
      return(-1);
    }
    if(this->positionmodels[r])
    {
      this->positionmodels[r]->Subscribe();
    }
    else
    {
      ROS_ERROR("no position");
      return(-1);
    }

    // subscribe to ROS topics
    laser_pubs_.push_back(n_.advertise<sensor_msgs::LaserScan>
                          (mapName(FRONT_LASER,r), 10));

    // set up vehicle model ROS topics
    vehicleModels_[r]->setup();
  }
  clock_pub_ = n_.advertise<roslib::Clock>("/clock",10);
  return(0);
}

StageNode::~StageNode()
{
  delete[] laserMsgs;
  for (size_t r = 0; r < vehicleModels_.size(); r++)
    delete vehicleModels_[r];
}

bool
StageNode::UpdateWorld()
{
  return this->world->UpdateAll();
}

void
StageNode::WorldCallback()
{
  boost::mutex::scoped_lock lock(msg_lock);

  this->sim_time.fromSec(world->SimTimeNow() / 1e6);
  // We're not allowed to publish clock==0, because it used as a special
  // value in parts of ROS, #4027.
  if(this->sim_time.sec == 0 && this->sim_time.nsec == 0)
  {
    ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
    return;
  }

  // Get latest laser data
  for (size_t r = 0; r < this->lasermodels.size(); r++)
  {
    std::vector<Stg::ModelLaser::Sample> samples = this->lasermodels[r]->GetSamples();
    if(samples.size())
    {
      // Translate into ROS message format and publish
      Stg::ModelLaser::Config cfg = this->lasermodels[r]->GetConfig();
      this->laserMsgs[r].angle_min = -cfg.fov/2.0;
      this->laserMsgs[r].angle_max = +cfg.fov/2.0;
      this->laserMsgs[r].angle_increment = cfg.fov/(double)(cfg.sample_count-1);
      this->laserMsgs[r].range_min = 0.0;
      this->laserMsgs[r].range_max = cfg.range_bounds.max;
      this->laserMsgs[r].ranges.resize(cfg.sample_count);
      this->laserMsgs[r].intensities.resize(cfg.sample_count);
      for(unsigned int i=0;i<cfg.sample_count;i++)
      {
        this->laserMsgs[r].ranges[i] = samples[i].range;
        this->laserMsgs[r].intensities[i] = (uint8_t)samples[i].reflectance;
      }

      // TODO map each laser to separate frame and topic names
      this->laserMsgs[r].header.frame_id = "/" + ArtFrames::front_sick;
      this->laserMsgs[r].header.stamp = sim_time;
      this->laser_pubs_[r].publish(this->laserMsgs[r]);
    }
  }

  // update latest position data
  for (size_t r = 0; r < this->positionmodels.size(); r++)
  {
    vehicleModels_[r]->update(this->sim_time);
  }

  this->clockMsg.clock = sim_time;
  this->clock_pub_.publish(this->clockMsg);
}

static bool quit = false;
void
sigint_handler(int num)
{
  quit = true;
}

int 
main(int argc, char** argv)
{ 
  ros::init(argc, argv, "artstage");

  ros::NodeHandle private_nh("~");
  bool gui = true;
  std::string world_file;
  if (private_nh.getParam("world_file", world_file))
    {
      // world_file parameter defined
      for (int i=0; i<argc; i++)
        {
          if(!strcmp(argv[i], "-g"))
            gui = false;
        }
    }
  else
    {
      // no world_file parameter
      // TODO: clean up repetitive code sequences
      if (argc < 2)
        {
          puts(USAGE);
          exit(-1);
        }
      for (int i=0; i<(argc-1); i++)
        {
          if(!strcmp(argv[i], "-g"))
            gui = false;
        }
      world_file = argv[argc-1];
    }

  StageNode sn(argc-1, argv, gui, world_file.c_str());

  if(sn.SubscribeModels() != 0)
    exit(-1);

  // spawn a thread to read incoming ROS messages
  boost::thread t = boost::thread::thread(boost::bind(&ros::spin));

  // TODO: get rid of this fixed-duration sleep, using some Stage builtin
  // PauseUntilNextUpdate() functionality.
  ros::WallRate r(10.0);

  // run stage update loop in the main thread
  while(ros::ok() && !sn.world->TestQuit())
  {
    if(gui)
      Fl::wait(r.expectedCycleTime().toSec());
    else
    {
      sn.UpdateWorld();
      r.sleep();
    }
  }
  t.join();

  exit(0);
}
