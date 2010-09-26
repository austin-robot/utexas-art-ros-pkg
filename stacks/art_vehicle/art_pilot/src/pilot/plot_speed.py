#!/usr/bin/env python
#
# Description: ART speed data analysis
#
#   Copyright (C) 2009 Austin Robot Technology
#
#   License: Modified BSD Software License Agreement
#
# $Id$

import sys
import getopt
import os

import numpy
import matplotlib.pyplot as pyplot
import pylab

class speedData:
    "ART speed data analysis."

    def __init__(self, name=None):
        "Constructor method."

        # set default name prefix for plots
        if name:
            self.name = name
        else:
            self.name = 'speed'

        # initialize some class variables
        self.clear()

        # default pressure polynomial
        self.pr_poly = numpy.array([0.44371437, -3.84781813,  9.19754362])
        # this value from octave (when pressure > 1.001v)
        self.oct_poly = numpy.array([0.40480, -3.56259, 8.69659])

    def cd(self, path):
        "Change directory to path."
        os.chdir(os.path.expanduser(path))

    def clear(self):
        "Clear data arrays."
        self.bc_t = numpy.array([])
        self.bc_pos = numpy.array([])
        self.bs_t = numpy.array([])
        self.bs_pos = numpy.array([])
        self.od_t = numpy.array([])
        self.od_v = numpy.array([])
        self.pc_t = numpy.array([])
        self.pc_v = numpy.array([])
        self.tc_t = numpy.array([])
        self.tc_pos = numpy.array([])
        self.ts_t = numpy.array([])
        self.ts_pos = numpy.array([])

    #def fit_pressure(self, order):
    #    "Fit a polynomial function of potentiometer to pressure voltages. "
    #    (self.pr_poly, resid, rank, singularvalues,
    #     rcond) = numpy.polyfit(self.bs_pot, self.bs_pr, order, full=True)
    #    print "polynomial coefficients: ", self.pr_poly
    #    print "residual: ", resid

    def plot(self, save=False):
        """ Plot some interesting data from the speed test run.

        When save=True write the figures to PNG files, otherwise show
        them interactively.
        """
        self.plot_summary(save)
        self.plot_speed(save)
        self.plot_servo(save)
        #self.plot_brake(save)
        #self.plot_throttle(save)

    def plot_brake(self, save=False):
        """ Plot brake requests and results from the speed test run.

        When save=True write the figure to a PNG file, otherwise show
        it interactively.
        """
        pyplot.hold(False)
        pyplot.plot(self.bs_t, self.bs_pos, 'r-', label="brake position")
        pyplot.hold(True)
        pyplot.plot(self.bc_t, self.bc_pos, 'bd', label="brake request")
        pyplot.xlabel("time (s)")
        pyplot.title(self.name + " brake")
        pyplot.legend(loc="best")
        pyplot.axis()
        pyplot.grid(True)
        self.render_plot(save, "-brake")

    def plot_servo(self, save=False):
        """ Plot servo controller requests and results from the speed test.

        When save=True write the figure to a PNG file, otherwise show
        it interactively.
        """
        pyplot.hold(False)
        pyplot.title(self.name + " servo")
        pyplot.plot(self.bs_t, -self.bs_pos, 'r-', label="-brake")
        pyplot.hold(True)
        pyplot.plot(self.bc_t, -self.bc_pos, 'rd', label="-request")
        pyplot.plot(self.ts_t, self.ts_pos, 'g-', label="throttle")
        pyplot.plot(self.tc_t, self.tc_pos, 'gd', label="request")
        pyplot.xlabel("time (s)")
        pyplot.ylabel("servo position")
        pyplot.legend(loc="best")
        pyplot.axis()
        pyplot.grid(True)
        self.render_plot(save, "-servo")

    def plot_speed(self, save=False):
        """ Plot speed requests and results from the speed test run.

        When save=True write the figure to a PNG file, otherwise show
        it interactively.
        """
        pyplot.hold(False)
        pyplot.title(self.name + " speed")
        pyplot.plot(self.od_t, self.od_v, 'b-', label="velocity")
        pyplot.hold(True)
        pyplot.plot(self.pc_t, self.pc_v, 'rd', label="request")
        pyplot.xlabel("time (s)")
        pyplot.ylabel("velocity (m/s)")
        pyplot.legend(loc="best")
        pyplot.axis()
        pyplot.grid(True)
        self.render_plot(save, "-speed")

    def plot_summary(self, save=False):
        """ Plot summary of speed test requests and results.

        When save=True write the figure to a PNG file, otherwise show
        it interactively.
        """
        pyplot.hold(False)
        pyplot.axis("off")              # no axis for overall figure
        pyplot.title(self.name + " speed summary")

        # 2 rows, 1 column: first (upper) subplot
        pyplot.subplot(211)
        pyplot.plot(self.od_t, self.od_v, 'b-', label="velocity")
        pyplot.hold(True)
        pyplot.plot(self.pc_t, self.pc_v, 'rd', label="request")
        pyplot.xlabel("time (s)")
        pyplot.ylabel("velocity (m/s)")
        pyplot.legend(loc="best")
        #pyplot.axis()
        pyplot.grid(True)

        # 2 rows, 1 column: second (lower) subplot
        pyplot.subplot(212)
        pyplot.hold(False)
        pyplot.plot(self.bs_t, -self.bs_pos, 'r-', label="-brake")
        pyplot.hold(True)
        pyplot.plot(self.bc_t, -self.bc_pos, 'rd', label="-request")
        pyplot.plot(self.ts_t, self.ts_pos, 'g-', label="throttle")
        pyplot.plot(self.tc_t, self.tc_pos, 'gd', label="request")
        pyplot.xlabel("time (s)")
        pyplot.ylabel("servo position")
        pyplot.legend(loc="best")
        #pyplot.axis()
        pyplot.grid(True)
        self.render_plot(save, "-summary")

    def plot_throttle(self, save=False):
        """ Plot throttle requests and results from the speed test run.

        When save=True write the figure to a PNG file, otherwise show
        it interactively.
        """
        pyplot.hold(False)
        pyplot.plot(self.ts_t, self.ts_pos, 'g-', label="throttle position")
        pyplot.hold(True)
        pyplot.plot(self.tc_t, self.tc_pos, 'gd', label="throttle request")
        pyplot.xlabel("time (s)")
        pyplot.title(self.name + " throttle")
        pyplot.legend(loc="best")
        pyplot.axis()
        pyplot.grid(True)
        self.render_plot(save, "-throttle")

    def pwd(self):
        "Print current working directory."
        return os.getcwd()

    def render_plot(self, save=False, suffix=''):
        """Common plot rendering commands.

        When save=True write the figure to a PNG file appending suffix
        to the file name.  Otherwise show the plot interactively.

        """
        if save:
            pylab.savefig(self.name + suffix + ".png")
        else:
            pyplot.show()

    def set_brake_cmd(self, t, pos):
        "set brake command data fields."
        self.bc_t =   numpy.append(self.bc_t, t)
        self.bc_pos = numpy.append(self.bc_pos, pos)

    def set_brake_state(self, t, pos):
        "set brake state data fields."
        self.bs_t =   numpy.append(self.bs_t, t)
        self.bs_pos = numpy.append(self.bs_pos, pos)

    def set_odometry(self, t, v):
        "set odometry data fields."
        self.od_t = numpy.append(self.od_t, t)
        self.od_v = numpy.append(self.od_v, v)

    def set_pilot_cmd(self, t, v):
        "set pilot command data fields."
        self.pc_t = numpy.append(self.pc_t, t)
        self.pc_v = numpy.append(self.pc_v, v)

    def set_throttle_cmd(self, t, pos):
        "set throttle command data fields."
        self.tc_t =   numpy.append(self.tc_t, t)
        self.tc_pos = numpy.append(self.tc_pos, pos)

    def set_throttle_state(self, t, pos):
        "set throttle state data fields."
        self.ts_t =   numpy.append(self.ts_t, t)
        self.ts_pos = numpy.append(self.ts_pos, pos)
        

def usage(progname):
    "Print usage message."
    print "\n", progname, """[-h] [ <file.bag> ]

 -h, --help        print this message
 -i, --interactive display plots to terminal (default: save as files)

Run some unit tests of the ART speed data analysis package.
 """

# main program -- for either script or interactive use
def main(argv=None):
    "Main program, called as a script or interactively."

    if argv is None:
        argv = sys.argv                 # use command args

    # extract base name of command, will be '' when imported
    progname = os.path.basename(argv[0])
    if progname is "":
        progname = "plot_speed.py"

    # process parameters
    try:
        opts, files = getopt.gnu_getopt(argv[1:], 'hi',
                                        ('help', 'interactive'))
    except getopt.error, msg:
        print msg
        print "for help use --help"
        return 9

    save = True

    for k,v in opts:
        if k in ("-h", "--help"):
            usage(progname)
            return 2
        if k in ("-i", "--interactive"):
            save = False

    # @todo run some unit tests

    return 0

# when called as a script or via python-send-buffer
if __name__ == "__main__":
    # run main function and exit
    sys.exit(main())
