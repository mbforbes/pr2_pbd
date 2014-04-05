'''
Action Log Analysis -- Anaylze user action logs (time spent fixing tasks).
'''

__author__ = "max"


#
# IMPORTS
#
# builtins
import code
from collections import Counter
import time

# 3rd party
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter, AutoMinorLocator, FixedLocator
import brewer2mpl as b2

# mine
from beautyplot import beautify

#
# CONSTANTS
#

PLOT_SC = 'scenario'
PLOT_FX = 'fix'
# Set PLOT to decide what to plot:
# PLOT_SC = plot scenario: plot time / scenario
# PLOT_FX = plot fix: plot time / fix (scenario's time / no. unreachable poses)
PLOT = PLOT_FX

#
# CLASSES
#


#
# FUNCTIONS
#

        
#
# MAIN
#

def get_time_arr():
    '''Returns numpy array of the following shape:

        n_users, n_scenarios

    where each entry is the time that a user spent in a given scenario on
    either:

        (a) each fix, on average (if PLOT == PLOT_FX)

        (b) the entire scenario (if PLOT == PLOT_SC)
    '''
    # settings
    low_u = 4 # low user num (inclusive)
    high_u = 34 # high user num (inclusive)
    nuns = [0, 2, 1, 3, 1, 2, 4, 2, 3, 5, 1, 3, 1, 2, 5, 4] # dummy in 0 place
    n_scenarios = len(nuns) - 1 # dummy in 0 place; subtract 1 for len
    xs = range(1, n_scenarios + 1)
    basedir = '/home/mbforbes/rosbuild_ws/pr2_pbd/pr2_pbd_interaction/data/'
    linecolor = '#0095D9'

    # no. users by no. scenarios
    resarr = np.zeros((high_u - low_u + 1, n_scenarios))
    for i, u in enumerate(range(low_u, high_u + 1)):
        # Counter gives us 0 by default so we can always add AND access missing
        # ones correctly.
        secs = Counter()
        lines = open(basedir + 'experiment' + str(u) + '/actionLog.txt', 'r').\
            readlines()
        prevtime, prevact, lastdiff = None, None, None
        for line in lines:
            pieces = line.strip().split(',')
            curtime = time.mktime(time.strptime(pieces[0]))
            curact = int(pieces[1])
            # on on the first line, prevtime & prevact are none; don't do this there
            if prevtime is not None:
                lastdiff = curtime - prevtime
                secs[prevact] += lastdiff
            # always update
            prevtime = curtime
            prevact = curact
        # we have to estimate the last timestamp (todo maybe turn this off)
        # we do this by assuming the user spent the same amount on the last action
        # as they did on the second-to-last action
        secs[curact] += lastdiff

        # organize by arranging keys & values of counter
        # don't want to use sorted(secs.viewkeys()) in case they missed one
        if PLOT == PLOT_SC:
            # seconds / scenario
            ys = [secs[key] for key in xs]
        elif PLOT == PLOT_FX:
            # seconds / n_unreachable fix
            ys = [secs[key] / nuns[key] for key in xs]
        else:
            exit(1)

        # save in overall array
        resarr[i] = ys
    return resarr

def plot_times(resarr):
    '''Plots the times as returned by get_time_arr().'''
    # plot all the things
    nuns = [0, 2, 1, 3, 1, 2, 4, 2, 3, 5, 1, 3, 1, 2, 5, 4] # dummy in 0 place
    xs = range(1, resarr.shape[1] + 1)
    ys = np.average(resarr, axis=0)
    stds = np.std(resarr, axis=0)

    if PLOT == PLOT_SC:
        title = 'Seconds Spent Working on Each Scenario'
        ylabel = 'Seconds / Scenario'
    elif PLOT == PLOT_FX:
        title = 'Seconds Spent Working on Each Unreachable Marker'
        ylabel = 'Seconds / Unreachable Marker (Avg)'
    else:
        exit(1)

    fig = plt.figure()
    plt.title(title)
    plt.xlabel('Scenario')
    plt.ylabel(ylabel)

    # NOTE(mbforbes): Testing whether this is reasonable...
    colors = b2.get_map('Set3', 'qualitative', 12).mpl_colors
    more_grey = '#929292'
    # Plot 30--miss a user
    hardlines = [3, 9, 14]
    newtasks = [1,6,11]
    print len(resarr)
    for i in range(27,28):
        #ax =  plt.subplot(5, 6, i + 1)
        ax = plt.axes()
        plt.plot(xs, resarr[i], color=colors[i % (len(colors) - 1)], linewidth=2.5)

        # Add vertical lines for the hard tasks
        for vloc in hardlines:
            plt.axvline(x=vloc, ymin=0, ymax=700, color=more_grey, ls=':', lw=2)
        for vloc in newtasks:
            plt.axvline(x=vloc, ymin=0, ymax=700, color=more_grey, ls='--',
                lw=2)

        # TODO(max): Split into different functions / graphs. Command line
        # options probably better than constants... maybe... but don't want
        # to have to specify every time...


        #plt.errorbar(xs, ys, yerr=stds, color=linecolor, linewidth=2.5,
        #    elinewidth=1)

        #Set up minor ticks (locations and labels) for y axis
        #fig = plt.figure(1)
        #ax = plt.axes()
        minorLocator = AutoMinorLocator(2)
        minorFormatter = FormatStrFormatter('%d')
        ax.yaxis.set_minor_formatter(minorFormatter)
        ax.yaxis.set_minor_locator(minorLocator)

        # Change x axis ticks appropriately (majors on 1, 6, 11)
        majorLocator = FixedLocator([1,6,11])
        minorLocator = FixedLocator(range(2,6) + range(7,11) + range(12,16))
        minorFormatter = FormatStrFormatter('%d')
        ax.xaxis.set_major_locator(majorLocator)
        ax.xaxis.set_minor_formatter(minorFormatter)
        ax.xaxis.set_minor_locator(minorLocator)

        # Set title, etc.
        plt.title('User ' + str(i + 1))

        # apply beautiful settings
        beautify(ax)

    #plt.tight_layout()
    plt.show()

def plot_fit_lines(resarr):
    pass

if __name__ == '__main__':
    resarr = get_time_arr()
    plot_times(resarr)
    plot_fit_lines(resarr)