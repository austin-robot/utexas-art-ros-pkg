/* -*- mode: C++ -*-
 *
 *  Description:  PID (Proportional, Integral, Derivative) control output.
 *
 *  Copyright (C) 2005 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2008 Austin Robot Technology, Patrick Beeson
 *
 *  License: Modified BSD Software License Agreement
 */

#ifndef _PID_H_
#define _PID_H_

/**  @file
   
     @brief PID (Proportional, Integral, Derivative) control output.
 */

#include <float.h>
#include <ros/ros.h>

/** @brief PID (Proportional, Integral, Derivative) control output. */
class Pid 
{
 public:

  /** @brief Constructor
   *  @param ctlname control output name for log messages
   */
 Pid(const char *ctlname):
  kp(1.0), ki(0.0), kd(0.0), omax(FLT_MAX), omin(-FLT_MAX), C(0.0), starting(1)
    {
      this->name = ctlname;
      Clear();
    };

  Pid(const char *ctlname, float kp, float ki, float kd,
      float omax=FLT_MAX, float omin=-FLT_MAX, float C=0.0): starting(1)
    {
      this->name = ctlname;
      this->Configure(kp, ki, kd, omax, omin, C);
      Clear();
    };
  
  virtual ~Pid() {};
  
  /** @brief Configure PID gains */
  virtual void Configure(float kp, float ki, float kd)
  {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
  };
  
  /** @brief Configure all PID parameters */
  virtual void Configure(float kp, float ki, float kd,
                         float omax, float omin, float C)
  {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      this->omax = omax;
      this->omin = omin;
      this->C = C;
  };
  
  /** @brief Configure PID parameters */
  virtual void Configure(const ros::NodeHandle &node)
  {
    // configure PID constants
    CfgParam(node, "kp", &this->kp);
    CfgParam(node, "ki", &this->ki);
    CfgParam(node, "kd", &this->kd);
    ROS_DEBUG("%s gains (%.3f, %.3f, %.3f)",
              this->name.c_str(), this->kp, this->ki, this->kd);
    CfgParam(node, "omax", &this->omax);
    CfgParam(node, "omin", &this->omin);
    CfgParam(node, "C", &this->C);
    ROS_DEBUG("%s output range [%.1f, %.1f]",
              this->name.c_str(), this->omin, this->omax);
  };
  
  /** @brief Update PID control output.
   *  @param error current output error
   *  @param output current output level
   *  @returns recommended change in output
   */
  float Update(float error, float output)
  {
    if (starting)
      {
	this->istate=0;
	this->dstate=output;
	starting=false;
      }
    
    // Proportional term
    float p = this->kp * error;
    
    // Derivative term
    float d = this->kd * (output - this->dstate);
    this->dstate = output;

    float i = this->ki * this->istate;

    float PID_control = (p + i - d);

    ROS_DEBUG("%s PID: %.3f = %.3f + %.3f - %.3f",
	      this->name.c_str(), PID_control, p, i, d);

    float PID_out=PID_control;

    if (PID_control > omax)
      PID_out=omax;
    if (PID_control < omin)
      PID_out=omin;

    // Integral term -- In reading other code, I is calculated after
    // the output.
    // The C term reduces the integral when the controller is already
    // pushing as hard as it can.
    this->istate = this->istate + error;
    float tracking = C*(PID_out-PID_control);
    if((istate > 0 && -tracking > istate) || (istate < 0 && -tracking < istate))
      istate = 0;
    else
      this->istate = this->istate + tracking;

    if (isnan(istate) || isinf(istate))
      istate=0;

    ROS_DEBUG("%s istate = %.3f, PID_out: %.3f, C*(PID_out-PID_control):%.3f",
              this->name.c_str(), istate, PID_out, C*(PID_out-PID_control));

    return PID_out;
  }

  /** @brief Clears the integral term if the setpoint has been reached **/
  void Clear()
  {
    starting=true;
  }

  /** @brief Copy the error history from another PID
   *
   *  @param pid The PID controller that has the history to copy
   */
  void CopyHistory(const Pid* pid)
  {
    // These values do not depend on the constants of the other PID,
    // so they're safe to copy
    this->dstate = pid->dstate;
    this->istate = pid->istate;
    // Check if we were called on an unused PID for whatever reason
    this->starting = pid->starting;
  }
  
 protected:

  std::string name;                     /**< control output name */

  // PID control parameters
  float dstate;				/**< previous output level */
  float istate;				/**< integrator state */
  float kp;				/**< proportional gain */
  float ki;				/**< integral gain */
  float kd;				/**< derivative gain */
  float omax;				/**< output maximum limit */
  float omin;				/**< output minimum limit */
  float C;                              /**< tracker to adapt integral */
  
  bool starting;


  /** @brief Configure one PID parameter
   *  @param node node handle for parameter server
   *  @param pname base name for this parameter
   *  @param fvalue place to store parameter value, if defined
   *               (already set to default value).
   */
  void CfgParam(const ros::NodeHandle &node,
                const char *pname, float *fvalue)
  {
    double dvalue;                      // (ROS parameter type is double)
    std::string optname = this->name + '_' + pname;
    if (node.getParamCached(optname, dvalue))
      {
        float param_value = dvalue;     // convert double to float
        if (*fvalue != param_value)     // new value?
          {
            ROS_INFO("%s changed to %.3f", optname.c_str(), param_value);
            *fvalue = param_value;
          }
      }
  }


};

#endif // _PID_H_ //
