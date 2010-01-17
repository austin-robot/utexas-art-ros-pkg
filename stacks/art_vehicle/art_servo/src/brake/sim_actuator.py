#!/usr/bin/env python
#
# Description:  Simulate ART brake driver actuator
#
#   Copyright (C) 2009 Austin Robot Technology
#
#   License: Modified BSD Software License Agreement
#
# $Id$
#

import sys
import getopt
import os

import math
import numpy

#import matplotlib.pyplot as pyplot
#import pylab

import plot_brake

# global actuator constants
ticksPerInch = 24000                    # encoder ticks per inch of travel
aMax = 8 * ticksPerInch                 # max acceleration (ticks/s^2)
vMax = 4 * ticksPerInch                 # max velocity (ticks/s)

# driver cycle interval
cycleInterval = 1.0/20.0                # 20Hz cycle interval

epsilon = 0.000001                      # small value, nearly zero

def getAccel(direction):
    "Get acceleration constant from direction of movement (-, 0, +)."
    if direction == 0.0:
        a = 0.0
    elif direction > 0.0:
        a = aMax
    else:
        a = -aMax
    return a

class Range:
    "Sensor range conversion"

    def __init__(self, off, full):
        "Constructor."
        self.off = off                  # value with brake off
        self.full = full                # value with brake fully on
        self.range = full - off         # corresponding range

    def __getitem__(self, pos):
        "Range indexing operation."
        return (pos * self.range) + self.off

    def pos(self, key):
        "Get corresponding position."
        return (key - self.off) / self.range

class Actuator:
    "Simulated ART brake actuator."

    def __init__(self, verbose=False):
        "Constructor."
        self.verbose = verbose

        # initialize sensor ranges
        self.encRg = Range(0.0, 50000.0)
        self.potRg = Range(4.9, 0.49)
        self.prRg = Range(0.85, 4.5)

        self.reset()

    def __str__(self):
        "Convert actuator state to printable string."
        return ("actuator state: [t: %.3f" % self.t
                + ", x: " + str(self.x)
                + ", v: " + str(self.v)
                + ", a: " + str(self.a) + "]\n"
                + "                [pos: " + str(self.pos)
                + ", pot: " + str(self.pot)
                + ", pr: " + str(self.pr) + "]")

    def move(self, dt):
        """Move actuator for dt seconds at constant acceleration.
        """
        if dt <= epsilon:
            raise RuntimeError('invalid move interval: ' + str(dt))
        v0 = self.v                     # previous velocity
        self.v += dt * self.a           # velocity change
        self.x += dt * (self.v+v0)/2.0  # advance by average velocity
        self.t += dt                    # advance simulated time
        self.update()                   # update sensor values
        if self.verbose and abs(self.v) > vMax:
            print 'actuator velocity too high:', self.v, '(time: %.3f)' % self.t
        #if abs(self.v) > vMax:
        #    raise RuntimeError('actuator velocity too high: ' + str(self.v))

    def reset(self):
        "Reset actuator state."
        self.t = 0                      # time (in seconds)
        self.x = 0                      # position (in ticks)
        self.v = 0                      # velocity (ticks/s)
        self.a = 0                      # acceleration (ticks/s^2)
        self.update()                   # update sensor values

    def setAccel(self, direction):
        "Set new acceleration constant from direction of movement (-, 0, +)."
        self.a = getAccel(direction)

    def update(self):
        "Update sensor values."
        self.pos = self.encRg.pos(self.x) # position [0.0, 1.0]
        self.pot = self.potRg[self.pos]   # potentiometer voltage
        self.pr = self.prRg[self.pos]     # pressure sensor voltage

# make an actuator instance
a = Actuator()

class Step:
    "One step in an actuator movement plan."

    def __init__(self, t=0.0, a=0.0):
        "Constructor."
        self.t = t                      # end of step time
        self.a = getAccel(a)            # constant acceleration

    def __str__(self):
        "Convert plan step to printable string."
        return ("step: [t: %.3f" % self.t + ", a: " + str(self.a) + "]")

class Plan:
    "An actuator movement plan."

    def __init__(self):
        "Constructor."
        self.reset()

    def __getitem__(self, i):
        "Get i-th Step of Plan."
        return self.steps[i]

    def __iter__(self):
        "Plan iterator visits each Step."
        return self.steps.__iter__()

    def __str__(self):
        "Convert next step of plan to printable string (empty if none)."
        retval = ""
        if len(self.steps) > 0:
            retval = 'plan ' + str(self.steps[0])
        return retval

    def advance(self, now):
        "Advance to next Plan Step, if it is now time."
        for s in self.steps:
            if s.t > now:
                return
            self.steps = self.steps[1:] # remove the first Step

    def append(self, step):
        "Add a Step to the Plan."
        self.steps.append(step)

    def getAccel(self):
        "Get current acceleration value."
        if len(self.steps) > 0:
            return self.steps[0].a
        else:
            return 0.0

    def interval(self, now, finish):
        """Return interval (in seconds) until end of:
                (1) current plan step,
                (2) current 20Hz driver cycle, or
                (3) finish time.
        """
        dt = finish - now               # (3) interval to finish
        if len(self.steps) > 0:
            dt = self.steps[0].t - now  # (1) interval in step

        # (2) interval remaining in current driver cycle
        dcycle = math.fmod(now, cycleInterval)
        if dcycle <= epsilon:           # now is start of a cycle
            dcycle = cycleInterval
        if dcycle < dt:
            dt = dcycle
        if dt <= epsilon:
            raise RuntimeError('invalid time interval: ' + str(dt))
        return dt

    def reset(self):
        "Reset the plan."
        self.steps = []

class Test:
    "Actuator simulation test."

    def __init__(self, verbose=False):
        "Constructor."
        self.verbose = verbose
        self.dev = Actuator(verbose)
        self.plan = Plan()
        if verbose:
            print self.dev
        self.plt = plot_brake.brakeData('sim-actuator')
        self.plt.set_state(self.dev.t, self.dev.pos, self.dev.pot,
                           self.dev.x, self.dev.pr)

    def check(self, value, expected, label=''):
        "Check test result against expected value."
        if (value - expected) > 0.00001:
            format = 'test error (sim time %.3f) %s %.6f (expecting %.6f)'
            print format % (self.dev.t, label, value, expected)
            print self.dev
            print self.plan

    def cmd(self, dx):
        """Command actuator to do relative move.

        @param dx = encoder ticks to move.
        @returns time (in seconds) required to get there.
        
        This calculation assumes a triangular velocity profile with
        constant acceleration |a|, applied first in one direction,
        then in the other.

        Erase any previous plan, then move the actuator in two steps:

          (1) accelerate in direction requested;
          (2) decelerate to zero velocity
        
        The real device supports a trapezoidal velocity profile with a
        maximum velocity (vMax).  Since it takes a long time to reach
        vMax, this model ignores that detail, which does not seem to
        matter much in practice.
        """
        t0 = self.dev.t                 # starting time
        x0 = self.dev.x                 # current encoder position
        v0 = self.dev.v                 # current actuator velocity
        if abs(dx) >= 0.5:              # nonzero move?
            a = getAccel(dx)            # accelerate in new direction
        elif abs(v0) > epsilon:         # current velocity nonzero?
            a = getAccel(-v0)           # cancel current velocity
        else:                           # holding in place
            self.plan.reset()           # delete any existing plan
            return 0.0

        dt_h = -v0 / a                  # time to halt current movement
        dx_h = dx - 0.5 * v0 * dt_h     # dx remaining after halt
        dt2 = math.sqrt(dx_h/a)         # deceleration time
        dt1 = dt2 + dt_h                # time to move towards goal
        dt = dt1 + dt2                  # total move duration

        # make a new plan
        self.plan.reset()
        if dt1 > epsilon:
            self.plan.append(Step(t0+dt1, a))
        if dt2 > epsilon:
            self.plan.append(Step(t0+dt, -a))

        if self.verbose:
            print str(self.dev)
            print str(self.plan)
            print 'cmd(%.f): %.3f' % (dx, t0), dt1, dt2, dt
        if dt < 0.0:
            raise RuntimeError('invalid move duration: ' + str(dt))

        # record command info for later plotting
        self.plt.set_cmd(t0, self.dev.encRg.pos(x0+dx))
        return dt

    def plot(self, save=False):
        "Plot some test results."
        self.plt.plot_position(save)
        #self.plt.plot_encoder(save)

    def run(self, duration):
        "Run the test for some duration."
        t = self.dev.t                  # start time
        finish = t + duration           # finish time
        if self.verbose:
            print 'run time: %.3f, duration %.3f' % (t, duration)
        while t < finish:
            self.plan.advance(t)
            if self.verbose:
                print 'run time: %.3f;' % t, str(self.plan)
            self.dev.setAccel(self.plan.getAccel())
            self.dev.move(self.plan.interval(t, finish))
            t = self.dev.t
            self.plt.set_state(t, self.dev.pos, self.dev.pot,
                               self.dev.x, self.dev.pr)
    

def usage(progname):
    "Print usage message."
    print "\n", progname, """[-h] [ <commands.txt> ]

 -h, --help         print this message
 -i, --interactive  display plots to terminal (default: save as files)
 -n, --no-plot      do not plot results
 -s, --save         save plot image files
 -v, --verbose      print verbose debug messages

 <commands.txt>\tname of command file to simulate

If a command file is specified, read brake commands from it, saving
plots of the data.  Otherwise, read brake commands interactively.
 """

# main program -- for either script or interactive use
def main(argv=None):
    "Main program, called as a script or interactively."

    if argv is None:
        argv = sys.argv                 # use command args

    # extract base name of command, will be '' when imported
    progname = os.path.basename(argv[0])
    if progname is "":
        progname = "sim_actuator.py"

    # process parameters
    try:
        opts, files = getopt.gnu_getopt(argv[1:], 'hinsv',
                                        ('help', 'interactive', 'no-plot'
                                         'save', 'verbose'))
    except getopt.error, msg:
        print msg
        print "for help use --help"
        return 9

    save = False
    plot = True
    verbose = False

    for k,v in opts:
        if k in ("-h", "--help"):
            usage(progname)
            return 2
        if k in ("-i", "--interactive"):
            save = False
        if k in ("-n", "--no-plot"):
            plot = False
        if k in ("-s", "--save"):
            save = True
        if k in ("-v", "--verbose"):
            verbose = True

    test = Test(verbose)

    if len(files) < 1:
        print "running pre-defined test sequence"
        test.run(0.1)

        # apply full brake
        test.run(test.cmd(50000.0))
        test.check(test.dev.x, 50000.0, 'encoder')
        test.check(test.dev.v, 0.0, 'velocity')

        # request brake off
        test.run(test.cmd(-50000.0))
        test.check(test.dev.x, 0.0, 'encoder')
        test.check(test.dev.v, 0.0, 'velocity')
        test.run(0.2)

        # apply full brake, run until one quarter finished
        dt = test.cmd(50000.0)
        test.run(dt/4.0)

        # stop the brake and hold it at the current position
        test.run(test.cmd(0.0))
        test.run(0.2)
        
        # release the brake slightly, then back to full
        test.run(test.cmd(-5000.0))
        test.run(0.2)
        test.run(test.cmd(50000.0 - test.dev.x))
        test.run(0.2)
        test.check(test.dev.x, 50000.0, 'encoder')
        test.check(test.dev.v, 0.0, 'velocity')

        # request brake off, stop part way and hold, then release fully
        dt = test.cmd(-50000.0)
        test.run(0.3 * dt)
        test.run(test.cmd(0.0))
        test.run(0.2)
        test.run(test.cmd(0.0 - test.dev.x))
        test.check(test.dev.x, 0.0, 'encoder')
        test.check(test.dev.v, 0.0, 'velocity')

        test.run(test.cmd(25000.0))
        test.check(test.dev.x, 25000.0, 'encoder')
        test.check(test.dev.v, 0.0, 'velocity')

        # request brake off
        test.run(test.cmd(-25000.0))
        test.check(test.dev.x, 0.0, 'encoder')
        test.check(test.dev.v, 0.0, 'velocity')

        test.run(test.cmd(0.0))
        test.check(test.dev.x, 0.0, 'encoder')
        test.check(test.dev.v, 0.0, 'velocity')

        dt = test.cmd(4320.0)
        test.check(dt, 0.3, 'dt')
        test.run(dt)
        test.check(test.dev.v, 0.0, 'velocity')

        test.run(0.199)
        test.check(test.dev.v, 0.0, 'velocity')

        dt = test.cmd(-4320.0)
        test.check(dt, 0.3, 'dt')
        test.run(dt)
        test.check(test.dev.v, 0.0, 'velocity')

        test.run(0.199)
        test.check(test.dev.v, 0.0, 'velocity')

        if plot:
            test.plot(save)

    elif len(files) == 1:
        print "Simulating " + files[0]
        #b.get_bag(files[0])
        #b.plot(save)

    else:
        print "only one command file can be processed at a time"
        usage(progname)
        return 9

    return 0

# when called as a script or via python-send-buffer
if __name__ == "__main__":

    # run main function and exit
    rc = 0
    try:
        rc = main()
    except KeyboardInterrupt:
        pass
        
    sys.exit(rc)
