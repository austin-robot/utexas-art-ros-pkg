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
class Pid:

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
  
    def Configure(self, node):
        "@brief Configure PID parameters"

        # configure PID constants
        self.CfgParam(node, "kp", self.kp)
        self.CfgParam(node, "ki", self.ki)
        self.CfgParam(node, "kd", self.kd)
        rospy.debug("%s gains (%.3f, %.3f, %.3f)",
                    self.name.c_str(), self.kp, self.ki, self.kd)
        self.CfgParam(node, "omax", self.omax)
        self.CfgParam(node, "omin", self.omin)
        self.CfgParam(node, "C", self.C)
        rospy.debug("%s output range [%.1f, %.1f]",
                    self.name.c_str(), self.omin, self.omax)
  
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

        rospy.debug("%s PID: %.3f = %.3f + %.3f - %.3f",
                    self.name.c_str(), PID_control, p, i, d)

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
        tracking = C*(PID_out-PID_control)
        if ((self.istate > 0 and -tracking > self.istate)
            or (self.istate < 0 and -tracking < self.istate)):
            self.istate = 0
        else:
            self.istate = self.istate + tracking

        if (math.isnan(self.istate) or math.isinf(self.istate)):
            self.istate=0

        rospy.debug("%s istate = %.3f, PID_out: %.3f, "
                    + "C*(PID_out-PID_control):%.3f",
                    self.name, self.istate, PID_out, C*(PID_out-PID_control))

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
  
    def CfgParam(self, node, pname, fvalue):
        """
        /** @brief Configure one PID parameter
         *  @param node node handle for parameter server
         *  @param pname base name for this parameter
         *  @returns parameter value, if defined
         *           (already set to default value).
         */
        """
        optname = self.name + '_' + pname
        if (node.getParamCached(optname, fvalue)):

            param_value = dvalue         # convert double to float
            if (fvalue != param_value): # new value?

                rospy.info("%s changed to %.3f", optname, param_value)
                fvalue = param_value
        return fvalue
