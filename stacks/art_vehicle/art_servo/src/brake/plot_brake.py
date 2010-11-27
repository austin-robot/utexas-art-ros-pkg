#!/usr/bin/env python
#
# Description: ART brake data analysis
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

class brakeData:
    "ART brake data analysis."

    def __init__(self, name=None):
        "Constructor method."

        # set default name prefix for plots
        if name:
            self.name = name
        else:
            self.name = 'brake'

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
        self.c_t = numpy.array([])
        self.c_pos = numpy.array([])
        self.s_t = numpy.array([])
        self.s_pos = numpy.array([])
        self.s_pot = numpy.array([])
        self.s_enc = numpy.array([])
        self.s_pr = numpy.array([])

    def fit_pressure(self, order):
        "Fit a polynomial function of potentiometer to pressure voltages. "
        (self.pr_poly, resid, rank, singularvalues,
         rcond) = numpy.polyfit(self.s_pot, self.s_pr, order, full=True)
        print("polynomial coefficients: " + str(self.pr_poly))
        print("residual: " + str(resid))

    def plot(self, save=False):
        """ Plot some interesting data from the brake test run.

        When save=True write the figures to PNG files, otherwise show
        them interactively.
        """
        self.plot_encoder(save)
        self.plot_position(save)
        self.plot_pressure(save)
        self.plot_sensors(save)
        self.plot_fit(save)

    def plot_encoder(self, save=False):
        """ Plot encoder data from the brake test run.

        When save=True write the figure to a PNG file, otherwise show
        it interactively.
        """
        pyplot.hold(False)
        pyplot.plot(self.s_t, self.s_enc, 'b-', label="encoder ticks")
        pyplot.xlabel("time (s)")
        pyplot.title(self.name + " encoder values")
        self.render_plot(save, "-encoder")

    def plot_fit(self, save=False):
        """ Plot pressure sensor voltages and corresponding curve fit from
        potentiometer values for the brake test run.

        When save=True write the figure to a PNG file, otherwise show
        it interactively.
        """
        pyplot.hold(False)
        pyplot.plot(self.s_t, self.s_pr, 'b', label="pressure (v)")
        pyplot.hold(True)
        pyplot.plot(self.s_t, numpy.polyval(self.pr_poly, self.s_pot),
                    'r', label="pot -> pressure")
        pyplot.xlabel("time (s)")
        pyplot.title(self.name + " pressure curve fit")
        self.render_plot(save, "-fit")

    def plot_position(self, save=False):
        """ Plot position requests and results from the brake test run.

        When save=True write the figure to a PNG file, otherwise show
        it interactively.
        """
        pyplot.hold(False)
        pyplot.plot(self.s_t, self.s_pos, 'b-', label="position reported")
        pyplot.hold(True)
        pyplot.plot(self.c_t, self.c_pos, 'rd', label="position request")
        pyplot.xlabel("time (s)")
        pyplot.title(self.name + " position")
        self.render_plot(save, "-position")

    def plot_pressure(self, save=False):
        """ Plot brake potentiometer and pressure sensor voltages.

        When save=True write the figure to a PNG file, otherwise show
        it interactively.
        """
        pyplot.hold(False)
        pyplot.plot(self.s_pot, self.s_pr, 'b+', label="pressure (v)")
        pyplot.hold(True)
        pyplot.plot(self.s_pot, numpy.polyval(self.pr_poly, self.s_pot),
                    'rd', label="polynomial fit")
        #pyplot.plot(self.s_pot, numpy.polyval(self.oct_poly, self.s_pot),
        #            'go', label="octave curve fit")
        pyplot.xlabel("potentiometer (v)")
        pyplot.title(self.name + " pressure")
        self.render_plot(save, "-pressure")

    def plot_sensors(self, save=False):
        """ Plot pressure and potentiometer sensor voltages from the brake
        test run.

        When save=True write the figure to a PNG file, otherwise show
        it interactively.
        """
        pyplot.hold(False)
        pyplot.plot(self.s_t, self.s_pot, 'b', label="potentiometer (v)")
        pyplot.hold(True)
        pyplot.plot(self.s_t, self.s_pr, 'r', label="pressure (v)")
        pyplot.xlabel("time (s)")
        pyplot.title(self.name + " sensor voltages")
        self.render_plot(save, "-sensor")

    def pwd(self):
        "Print current working directory."
        return os.getcwd()

    def render_plot(self, save=False, suffix=''):
        """Common plot rendering commands.

        When save=True write the figure to a PNG file appending suffix
        to the file name.  Otherwise show the plot interactively.

        """
        pyplot.legend(loc="best")
        pyplot.axis("auto")
        pyplot.grid()
        if save:
            pylab.savefig(self.name + suffix + ".png")
        else:
            pyplot.show()

    def set_cmd(self, t, pos):
        "set command data fields."
        self.c_t =   numpy.append(self.c_t, t)
        self.c_pos = numpy.append(self.c_pos, pos)

    def set_state(self, t, pos, pot, enc, pr):
        "set state data fields."
        self.s_t =   numpy.append(self.s_t, t)
        self.s_pos = numpy.append(self.s_pos, pos)
        self.s_pot = numpy.append(self.s_pot, pot)
        self.s_enc = numpy.append(self.s_enc, enc)
        self.s_pr =  numpy.append(self.s_pr, pr)
        

b = brakeData()

def usage(progname):
    "Print usage message."
    print("\n" + str(progname) + """[-h] [ <file.bag> ]

 -h, --help        print this message
 -i, --interactive display plots to terminal (default: save as files)

Run some unit tests of the ART brake data analysis package.
 """)

# main program -- for either script or interactive use
def main(argv=None):
    "Main program, called as a script or interactively."

    if argv is None:
        argv = sys.argv                 # use command args

    # extract base name of command, will be '' when imported
    progname = os.path.basename(argv[0])
    if progname is "":
        progname = "plot_brake.py"

    # process parameters
    try:
        opts, files = getopt.gnu_getopt(argv[1:], 'hi',
                                        ('help', 'interactive'))
    except getopt.error, msg:
        print(msg)
        print("for help use --help")
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
