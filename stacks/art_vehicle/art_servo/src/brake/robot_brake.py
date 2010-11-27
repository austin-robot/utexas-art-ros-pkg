#!/usr/bin/python
# $Id: 
"""
brake script for Marvin -- Function to send the controller a command,
then read its response  with a 1-second timeout.

This program can be run as a shell script or imported as a module.
Help is provided in each case:

  as a script:
    --help describes the arguments and options

  as a module:
    import roslib
    roslib.load_manifest('art_servo')
    import brake.robot_brake
    help(brake.robot_brake.commands)

"""

import serial
import time
import types
import sys
import getopt
import os
from os.path import basename, splitext

# globals
TTY="/dev/brake"

class commands:
    """ Class to connect and send commands to brake.
    """

    def __init__(self, device=TTY):
        """ Constructor method.
            Takes serial port as input.
        """
        self.ser=serial.Serial(device, 38400, timeout=1)
        self.ser.open()

    def baud(self):   
        """ Set baud rate
        """
        print("setting brake baud rate")
        self.ser.setBaudrate(9600)
        self.ser.write("BAUD38400\r")
        time.sleep(3)
        self.ser.close()
        self.ser.setBaudrate(38400)
        self.ser.open()
        self.st()

    def servo_cmd(self,cmd):
        """ Send command and wait for response
        """

        print(cmd)
        cmd+=" RW\r"
    
        self.ser.write(cmd)
        status=self.ser.readline()
        if status == '':
            print("Device not responding")
        else:
            lstatus=status.split('\r')
            for x in lstatus:
                if x:
                    print("Device returns (0x%x)\n" % (int(x)))
        return status

    def setup(self):
        """ Run brake setup routine
        """

        print("initializing brake on " + TTY)
        self.servo_cmd("ZS")		# reset all status bits
        self.servo_cmd("UCI")        # disable right (pos, on) limit
        self.servo_cmd("UDI")        # disable left (neg, off) limit
        self.servo_cmd("F=0")        # disable servo on stop
    
        self.servo_cmd("MP D=0 G")	# ensure position mode
        self.servo_cmd("UAI")        # set Port A as an input
        self.servo_cmd("O=0")        # set temporary origin
        self.servo_cmd("A=8*96")	# set acceleration
        self.servo_cmd("V=32212*48")	# set velocity
        
        self.servo_cmd("MT T=-125")	# push firmly in neg direction
        time.sleep(4)
        self.servo_cmd("RP")		# report encoder position
        self.servo_cmd("MT T=5")	# push gently in pos direction
        time.sleep(1)
        self.servo_cmd("MP D=0 G")	# back to position mode
        self.servo_cmd("RP")		# report encoder position
        self.servo_cmd("O=0")        # set the origin here
        
        self.servo_cmd("LIMD")	# enable directional limits
        self.servo_cmd("LIML")	# set limits active low (default)
    
        # UCP: Restore pin C to right (plus, brake-off) limit function
        self.servo_cmd("UCP")

        # UDM: Restore pin D to left (minus, brake-full) limit function
        self.servo_cmd("UDM")

        # F=1: stop and then servo in place after limit fault
        self.servo_cmd("F=1")

    def off(self):
        """ Release brake
        """

        print("disengaging brake")
        self.servo_cmd("Zr")
        self.servo_cmd("Zl")
        self.servo_cmd("MP P=0 G")


    def full(self):
        """ Engage brake fully
        """
        print("setting brake fully on")
        self.servo_cmd("MT T=200")
        self.servo_cmd("Zr")
        self.servo_cmd("Zl")
        time.sleep(1)
        self.servo_cmd("MP D=0 G")
        self.servo_cmd("RP")
        self.servo_cmd("c=UEA Rc")

    def pos(self):
        """ Return brake position info
        """
        print("motor position:")
        self.servo_cmd("RP")
        print("potentiometer value:")
        self.servo_cmd("c=UEA Rc")
        print("brake pressure:")
        self.servo_cmd("c=UAA Rc")
        print("position error limit:")
        self.servo_cmd("c=E Rc")


    def st(self):
        """ Print status of motor
        """
        print("current status")
        self.servo_cmd("")

def usage(progname):
    """ print usage message
    """
    print("\n" + str(progname) + """[-h] command [ <serial-port> ]

 -h, --help\tprint this message

 command options:
    \tbaud\tset baud rate to 38400
    \tfull\tfully engage brake
    \toff\tfully release brake
    \tpos\treport current motor position and potentiometer reading
    \tsetup\tinitialize brake servo
    \tst\tprint current status

 default serial-port: /dev/brake
""")

# main program
def main(argv=None):
    """ Main program. For either script or interactive use.
    """
    global TTY

    if argv is None:
        argv = sys.argv

    # extract base name of command ('', when imported)
    progname = basename(argv[0])
    if progname is "":
        progname = "robot-brake.py"

    # process parameters
    opts = {}
    try:
        o, params = getopt.gnu_getopt(argv[1:], 'h', ('help'))

    except getopt.error, msg:
        print(msg)
        print("for help use --help")
        return 8

    for k,v in o:
        opts[k] = v
    if '-h' in opts or '--help' in opts:
        usage(progname)
        return 9
    if len(params) < 1:
        usage(progname)
        print("no command provided")
        return 8

    if len(params) == 2:
        TTY = params[1]

    try:
        c = commands(TTY)

        CMD=params[0]
        if CMD=="baud":
            c.baud()
        elif CMD=="pos":
            c.pos()
        elif CMD=="full":
            c.full()
        elif CMD=="off":
            c.off()
        elif CMD=="setup":
            c.setup()
        elif CMD=="st":
            c.st()
        else:
            usage(progname)
            print("invalid command provided")
            return 7

    except serial.serialutil.SerialException, ioerror:
        print(ioerror)
        return 1


if __name__ == "__main__":
    # run main function and exit
    sys.exit(main())
