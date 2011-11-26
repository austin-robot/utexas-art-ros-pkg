/*
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

#include <ros/ros.h>

#include <art_msgs/ArtHertz.h>
#include <art/conversions.h>

#include <art_msgs/Shifter.h>
#include <art_msgs/IOadrCommand.h>
#include <art_msgs/IOadrState.h>

#include "dev8x.h"			// IOADR8x device interface

/**  \file

     @brief NCD IOADR8x multipurpose I/O board driver

This driver provides an interface to a National Control Devices
IOADR8x hybrid relay and A/D controller board.  This hybrid device
provides eight relays and five analog to digital converters.  The
driver publishes their current values and handles commands to set the
relays.

Our vehicle currently has two IOADR8x boards.  Each is controlled by a
separate ROS node.  The node name determines the input and output
messages:

- "ioadr" provides steering sensor voltage and relays to interface with
the DARPA E-stop unit and the vehicle warning devices (flasher and
audible alarm).

- "shifter" reads and sets the transmission control relays.

@par Commands

- PLAYER_AIO_CMD_STATE

Sets the state of a relay.  The "id" is a bit number (0 to 7) for the
relay.  If the "voltage" is close to 0.0, the corresponding relay will
be set off, otherwise it is set on.

@par ROS Parameters

- ~/port (string)
  - tty port name for IOADR8x board
  - default: "/dev/null"

  \author Jack O'Quin

*/


class IOadr
{
public:

  IOadr();

  void Main();
  int Setup(ros::NodeHandle node);
  int Shutdown();

  // pointer to any of the poll_* methods
  typedef int (IOadr::*poll_method_t)(int ch);

  typedef struct
  {
    const char *name;			// parameter name
    poll_method_t function;		// function method to call
    int devnum;				// IOadr8x device number
    int field;                          // analog voltage field index
  } poll_parms_t;

  int poll_Analog_8bit(int ch);
  int poll_Analog_10bit(int ch);
  int poll_Digital(int ch);
  int poll_ShifterInd(int ch);

private:

  void GetSetRelays(void);
  void PollDevice(void);
  void processOutput(const art_msgs::IOadrCommand::ConstPtr &cmd);
  void processShifter(const art_msgs::Shifter::ConstPtr &shifterIn);

  std::vector<poll_parms_t *> poll_list_; // poll list

  // .cfg variables:
  std::string node_name_;               // actual node name assigned
  int reset_relays_;			// initial/final relays setting
  std::string port_;			// IOADR8x tty port name
  bool do_shifter_;                     // handle Shifter messages

  // ROS topic interfaces
  ros::Subscriber ioadr_cmd_;            // ioadr command
  ros::Publisher  ioadr_state_;          // ioadr state
  ros::Subscriber shifter_cmd_;          // shifter command
  ros::Publisher  shifter_state_;        // shifter state
  uint8_t shifter_gear_;                 // current gear number

  // requested relay settings
  uint8_t relay_mask_;
  uint8_t relay_bits_;

  // current device input state
  art_msgs::IOadrState ioMsg_;         // controller state message

  // hardware IOADR8x interface
  dev8x *dev_;
};

static IOadr::poll_parms_t poll_parms_table[] =
{
  //   name          method                      devnum  field
  {"AnalogA",        &IOadr::poll_Analog_8bit,   3,      0},
  {"AnalogA(10bit)", &IOadr::poll_Analog_10bit,  3,      0},
  {"AnalogB",        &IOadr::poll_Analog_8bit,   4,      1},
  {"AnalogB(10bit)", &IOadr::poll_Analog_10bit,  4,      1},
  {"AnalogC",        &IOadr::poll_Analog_8bit,   5,      2},
  {"AnalogC(10bit)", &IOadr::poll_Analog_10bit,  5,      2},
  {"DigitalB",       &IOadr::poll_Digital,       1,     -1},
  {"ShifterInd",     &IOadr::poll_ShifterInd,    1,     -1},
  {"",               NULL,                      -1,     -1},
  {NULL,             NULL,                      -1,     -1},
};

IOadr::poll_parms_t *LookupInput(const char *name)
{
  int i;
  for (i = 0; poll_parms_table[i].name; ++i)
    {
      if (0 == strcmp(poll_parms_table[i].name, name))
	break;
    }
  return &poll_parms_table[i];
}

// Relays bit values for gear indices: reset, park, rev, neut, drive.
// NOTE: these are NOT the same as the digital B input values.
static const uint8_t relay_value_[5] = {0x00, 0x02, 0x04, 0x08, 0x10};

// constructor
IOadr::IOadr()
{
  node_name_ = ros::this_node::getName();
  ROS_INFO_STREAM("initialize node: " << node_name_);

  // use private node handle to get parameters
  ros::NodeHandle mynh("~");

  mynh.param("reset_relays", reset_relays_, 0);
  if (reset_relays_ >= 0)
    ROS_INFO("reset relays to 0x%02x", reset_relays_);

  mynh.param("shifter", do_shifter_, false);
  if (do_shifter_)
    {
      ROS_INFO("providing shifter interface");
      shifter_gear_ = art_msgs::Shifter::Park;
      port_ = "/dev/shifter";
    }
  else
    {
      // default port name for main IOADR8x board
      port_ = "/dev/ioadr8x";
    }
  mynh.getParam("port", port_);
  ROS_INFO_STREAM("IOADR8x port = " << port_);

  if (mynh.hasParam("poll_list"))
    {
      // read list of poll strings
    }
  else
    {
      // set default poll list depending on node name
      if (do_shifter_)
        {
          // default shifter poll list
          if (port_ != "/dev/null")
            poll_list_.push_back(LookupInput("ShifterInd"));
        }
      else
        {
          // default ioadr poll list
          poll_list_.push_back(LookupInput("AnalogA(10bit)"));
          poll_list_.push_back(LookupInput("DigitalB"));
        }
    }
}

// Set up the device.  Return 0 if things go well, and -1 otherwise.
int IOadr::Setup(ros::NodeHandle node)
{   
  static int qDepth = 1;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  if (do_shifter_)
    {
      // initialize shifter command and state topics
      shifter_cmd_ = node.subscribe("shifter/cmd", qDepth,
                                    &IOadr::processShifter, this, noDelay);
      shifter_state_ =
        node.advertise<art_msgs::Shifter>("shifter/state", qDepth);
    }
  else
    {
      // initialize ioadr command and state topics
      ioadr_cmd_ = node.subscribe("ioadr/cmd", qDepth,
                                  &IOadr::processOutput, this, noDelay);
      ioadr_state_ =
        node.advertise<art_msgs::IOadrState>("ioadr/state", qDepth);
    }

  // create device interface class
  dev_ = new dev8x(port_.c_str());
  int rc = dev_->dev8x::setup();
  if (rc != 0)
    return rc;

  ROS_INFO_STREAM(node_name_ << " device opened");

  if (reset_relays_ >= 0)
    {
      dev_->set_relays((uint8_t) reset_relays_);
      ROS_INFO("set relays to 0x%02x", reset_relays_);
    }  

  return 0;
}

// Shutdown the device
int IOadr::Shutdown()
{
  if (reset_relays_ >= 0)
    {
      // make sure IOADR8x board is not busy before setting relays.
      usleep((useconds_t) rint(MIN_RELAY_WAIT*1000000.0));
      dev_->set_relays((uint8_t) reset_relays_);
      ROS_INFO("set relays to 0x%02x", reset_relays_);
    }

  dev_->dev8x::shutdown();
  ROS_INFO_STREAM(node_name_ << " device closed");
  delete dev_;
  return 0;
}

void IOadr::processOutput(const art_msgs::IOadrCommand::ConstPtr &cmd)
{
  relay_mask_ |= (cmd->relays_on | cmd->relays_off);
  relay_bits_ |= cmd->relays_on;
  ROS_DEBUG("relay bits, mask = 0x%02x, 0x%02x", relay_bits_, relay_mask_);
}

// Callback when shifter command arrives.
// (only subscribed when do_shifter_ is true)
void IOadr::processShifter(const art_msgs::Shifter::ConstPtr &shifterIn)
{
  // save requested gear when doing shifter simulation
  if (shifterIn->gear != art_msgs::Shifter::Reset
      && port_ == "/dev/null")
    shifter_gear_ = shifterIn->gear;
  ROS_INFO("Shifter command: gear %u", shifterIn->gear);
  relay_bits_ = relay_value_[shifterIn->gear];
  relay_mask_ = 0xff;                   // set all relay bits
}

int IOadr::poll_Analog_8bit(int ch)
{
  int data;
  int rc = dev_->read_8bit_port(poll_list_[ch]->devnum, &data);
  if (rc == 0)
    {
      // convert A/D input to voltage (8-bit converter with 5-volt range)
      int i = poll_list_[ch]->field;
      ioMsg_.voltages[i] = analog_volts(data, 5.0, 8);
      ROS_DEBUG("%s input = %.3f volts (0x%04x)",
		poll_list_[i]->name, ioMsg_.voltages[ch], data);
    }
  return rc;
}

int IOadr::poll_Analog_10bit(int ch)
{
  int data;
  int rc = dev_->read_10bit_port(poll_list_[ch]->devnum, &data);
  if (rc == 0)
    {
      // convert A/D input to voltage (10-bit converter with 5-volt range)
      int i = poll_list_[ch]->field;
      ioMsg_.voltages[i] = analog_volts(data, 5.0, 10);
      ROS_DEBUG("%s input = %.3f volts (0x%04x)",
		poll_list_[ch]->name, ioMsg_.voltages[i], data);
    }
  return rc;
}

int IOadr::poll_Digital(int ch)
{
  int data;
  int rc = dev_->read_8bit_port(poll_list_[ch]->devnum, &data);
  if (rc == 0)
    {
      ioMsg_.digitalB = data;
    }
  return rc;
}

// translate shifter indicator into shift request ID
// (only used when do_shifter_ true and port is a real device)
int IOadr::poll_ShifterInd(int ch)
{
  // read shifter feedback from Digital port B
  int data;
  int rc = dev_->read_8bit_port(1, &data);
  if (rc == 0)
    {
      // ignore the results unless the I/O was successful
      uint8_t gear;
      uint8_t digitalB_bits = ~data;
      if (digitalB_bits & 0x80)
	gear = art_msgs::Shifter::Park;
      else if (digitalB_bits & 0x40)
	gear = art_msgs::Shifter::Reverse;
      else if (digitalB_bits & 0x20)
	gear = art_msgs::Shifter::Neutral;
      else if (digitalB_bits & 0x10)
	gear = art_msgs::Shifter::Drive;
      else
        // there should only be one bit on at a time, so this should
        // not occur
	gear = art_msgs::Shifter::Reset;

      // save current gear number
      shifter_gear_ = gear;
    }
  return rc;
}

// poll device for pending input
//
// if an I/O fails, the corresponding voltages[i] remains unchanged
//
void IOadr::PollDevice(void)
{
  int rc = 0;

  // First, poll any analog or digital ports that are configured.
  for (unsigned i = 0; i < poll_list_.size(); ++i)
    {
      poll_method_t poll_method = poll_list_[i]->function;
      if (poll_method)
        {
          rc = (this->*poll_method)(i);
          if (rc != 0)
            ROS_ERROR_THROTTLE(100, "poll method returns %d", rc);
        }
    }

  // Get relay values, set new ones if requested.  Note: setting the
  // relays MUST be the last IOADR8x operation of the cycle.  After
  // that, the device seems to stay busy for a while.  It hangs if
  // contacted again too soon.
  GetSetRelays();

  if (do_shifter_)                      // publishing shifter state?
    {
      art_msgs::Shifter shifter_msg;
      shifter_msg.header.stamp = ros::Time::now();
      shifter_msg.gear = shifter_gear_;
      shifter_msg.relays = ioMsg_.relays;
      shifter_state_.publish(shifter_msg);
    }
  else
    {
      // publish ioadr state message
      ioMsg_.header.stamp = ros::Time::now();
      ioadr_state_.publish(ioMsg_);
    }
}

// Get relay values, set new ones if requested.
//
// Updates: relay_mask_, relay_bits_, ioMsg_.relays
//
// If we failed to set the relays this cycle, leave relay_mask_,
// relay_bits_ alone and try again next time.
//
void IOadr::GetSetRelays(void)
{
  // get current relay settings
  int rc = dev_->query_relays(&ioMsg_.relays);
  if (rc != 0)				// device busy or not working?
    return;

  if (relay_mask_)			// new setting requested?
    {
      uint8_t new_relays = (ioMsg_.relays & (~relay_mask_)) | relay_bits_;
      rc = dev_->set_relays(new_relays);
      if (rc == 0)			// successful?
	{
	  // The device requires a wait after setting all relays
	  // before accessing them again.  Since set_relays()
	  // returns immediately, we must leave them alone until
	  // our next cycle, which should be long enough for the
	  // device to finish.  At 10Hz, that should not be a
	  // problem.

	  relay_mask_ = relay_bits_ = 0;
	  ioMsg_.relays = new_relays;
          ROS_DEBUG("set relays to 0x%02x", ioMsg_.relays);
	}
    }
}

void IOadr::Main()
{
  ros::Rate cycle(art_msgs::ArtHertz::IOADR); // set driver cycle rate

  // Main loop; grab messages off our queue and republish them via ROS
  while(ros::ok())
    {
      // wait for next cycle -- this must come first, because Setup()
      // may have initialized the relays.  If we hit the device again
      // too soon, it locks up.  Apparently, this is not just limited
      // to the relay ports.

      cycle.sleep();

      relay_mask_ = relay_bits_ = 0;

      ros::spinOnce();                  // handle incoming commands

      PollDevice();                     // get & publish device status
    }
}
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ioadr");       // default node name
  ros::NodeHandle node;

  IOadr io;

  if (io.Setup(node) != 0)
    return 2;

  io.Main();                            // driver main loop

  io.Shutdown();

  return 0;
}
