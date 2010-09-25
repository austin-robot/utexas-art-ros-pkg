#
#  Description:  PID (Proportional, Integral, Derivative) control output.
#
#  Copyright (C) 2005 Austin Robot Technology, Jack O'Quin
#  Copyright (C) 2008 Austin Robot Technology, Patrick Beeson
#
#  License: Modified BSD Software License Agreement
#


#/**  @file
#   
#  @brief PID (Proportional, Integral, Derivative) control output.
# */

PKG_NAME = 'art_common'
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy
import math

FLT_MAX = 1e100                         # really large floating point value

#/** @brief PID (Proportional, Integral, Derivative) control output. */
class Pid(object):

    def __init__(self, ctlname, kp=1.0, ki=0.0, kd=0.0,
              omax=FLT_MAX, omin=-FLT_MAX, C=0.0):
        """ @brief Constructor
            @param ctlname control output name for log messages
        """

        # PID control parameters
        self.name = ctlname             # control name for logging
        self.kp = kp                    # proportional gain
        self.ki = ki                    # integral gain
        self.kd = kd                    # derivative gain
        self.omax = omax                # output maximum limit
        self.omin = omin                # output minimum limit
        self.C = C                      # tracker to adapt integral
        self.Clear()
  
    def Configure(self):
        "@brief Configure PID parameters"

        # configure PID constants

        # CfgParam was changed to return the new value.
        # I've made a slight modification so that this function
        # catches the return value.
        self.kp = self.CfgParam("kp", self.kp)
        self.ki = self.CfgParam("ki", self.ki)
        self.kd = self.CfgParam("kd", self.kd)
        rospy.logdebug("%s gains (%.3f, %.3f, %.3f)",
                       self.name, self.kp, self.ki, self.kd)
        self.omax = self.CfgParam("omax", self.omax)
        self.omin = self.CfgParam("omin", self.omin)
        self.C = self.CfgParam("C", self.C);
        rospy.logdebug("%s output range [%.1f, %.1f]",
                       self.name, self.omin, self.omax)
  
    def Update(self, error, output):
        """ @brief Update PID control output.
        
            @param error current output error
            @param output current output level
            @returns recommended change in output
        """

        if (self.starting):
            self.istate=0
            self.dstate=output
            self.starting=False
    
        # Proportional term
        p = self.kp * error
    
        # Derivative term
        d = self.kd * (output - self.dstate)
        self.dstate = output            # previous output level

        # Integral term
        i = self.ki * self.istate       # integrator state
        PID_control = (p + i - d)

        rospy.logdebug("%s PID: %.3f = %.3f + %.3f - %.3f",
                       self.name, PID_control, p, i, d)

        PID_out=PID_control

        if (PID_control > self.omax):
            PID_out=self.omax
        if (PID_control < self.omin):
            PID_out=self.omin

        # Integral term -- In reading other code, I is calculated
        # after the output.
        #
        # The C term reduces the integral when the controller is
        # already pushing as hard as it can.
        self.istate = self.istate + error
        tracking = self.C*(PID_out-PID_control)
        if ((self.istate > 0 and -tracking > self.istate)
            or (self.istate < 0 and -tracking < self.istate)):
            self.istate = 0
        else:
            self.istate = self.istate + tracking

        if (math.isnan(self.istate) or math.isinf(self.istate)):
            self.istate=0

        rospy.logdebug("%s istate = %.3f, PID_out: %.3f, "
                       + "C*(PID_out-PID_control):%.3f",
                       self.name, self.istate, PID_out,
                       self.C*(PID_out-PID_control))

        return PID_out

    def Clear(self):
        """#/** @brief Clears the integral term if the setpoint
                       has been reached */
        """
        self.starting=True

    def CopyHistory(self, pid):
        """
        #/** @brief Copy the error history from another PID
        # *  @param pid The PID controller that has the history to copy
        # */
        """

        # These values do not depend on the constants of the other
        # PID, so they're safe to copy
        self.dstate = pid.dstate
        self.istate = pid.istate
        # Check if we were called on an unused PID for whatever reason
        self.starting = pid.starting
  
    def CfgParam(self, pname, fvalue):
        """
        /** @brief Configure one PID parameter
         *  @param node node handle for parameter server -- #NOTE: Removed by David
         *  @param pname base name for this parameter
         *  @returns parameter value, if defined
         *           (already set to default value).
         */
        """
        # Original program logic
        """
        optname = self.name + '_' + pname
        if (node.getParamCached(optname, fvalue)):

            param_value = dvalue         # convert double to float
            if (fvalue != param_value): # new value?

                rospy.info("%s changed to %.3f", optname, param_value)
                fvalue = param_value
        return fvalue
        """
        
        # The original program logic used something called a NodeHandle,
        # supported by roscpp. However, rospy appears to have no such thing. 
        # Also, when this function is called, the variables in the function
        # call are passed by value, so this function has to return the new
        # value and the calling function has to catch the return value.
        # So I've changed both functions a bit.        
        optname = self.name + '_' + pname
        if (rospy.has_param(optname)) :
          param_value = rospy.get_param(optname)
          if (fvalue != param_value) :
            rospy.loginfo("%s changed to %.3f", optname, param_value)
            fvalue = param_value
        return fvalue
