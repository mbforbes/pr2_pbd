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
import sys
import os

# 3rd party
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter, AutoMinorLocator, FixedLocator
import brewer2mpl as b2

#
# CONSTANTS
#

PLOT_SC = 'scenario'
PLOT_FX = 'fix'
# Set PLOT to decide what to plot:
# PLOT_SC = plot scenario: plot time / scenario
# PLOT_FX = plot fix: plot time / fix (scenario's time / no. unreachable poses)
PLOT = PLOT_FX

COLORS = b2.get_map('YlOrRd', 'Sequential', 9).mpl_colors

#
# CLASSES
#


#
# FUNCTIONS
#

def beautify(ax=None):
    '''Modified beautify :-/
    '''
    # Settings
    almost_black = '#262626'
    more_grey = '#929292'
    text_font = 'serif'
    number_font = 'serif'

    # Get the figure and axes.
    if ax is None:
        fig = plt.figure(1)
        ax = plt.axes()

    # Remove 'spines' (axis lines)
    spines_to_remove = ['top', 'right']
    for spine in spines_to_remove:
        ax.spines[spine].set_visible(False)

    # Make ticks only on the left and bottom (not on the spines that we removed)
    ax.yaxis.tick_left()
    ax.xaxis.tick_bottom()

    # To remove the ticks all-together (like in prettyplotlib), do the following
    # instead of tick_left() and tick_bottom()
    #ax.xaxis.set_ticks_position('none')
    #ax.yaxis.set_ticks_position('none')

    # Now make them go 'out' rather than 'in'
    for axis in ['x', 'y']:
        ax.tick_params(axis=axis, which='both', direction='out', pad=7)
        ax.tick_params(axis=axis, which='major', color=almost_black, length=6)
        ax.tick_params(axis=axis, which='minor', color=more_grey, length=4)

    # Make thinner and off-black
    spines_to_keep = ['bottom', 'left']
    for spine in spines_to_keep:
        ax.spines[spine].set_linewidth(0.5)
        ax.spines[spine].set_color(almost_black)

    # Change the labels & title to the off-black and change their font
    for label in [ax.yaxis.label, ax.xaxis.label, ax.title]:
        label.set_color(almost_black)
        label.set_family(text_font)
        label.set_fontsize(15)

    # Change the tick labels' color and font and padding
    for axis in [ax.yaxis, ax.xaxis]:
        # padding
        #axis.labelpad = 20
        # major ticks
        for major_tick in axis.get_major_ticks():
            label = major_tick.label
            label.set_color(almost_black)
            label.set_family(number_font)
        # minor ticks
        for minor_tick in axis.get_minor_ticks():
            label = minor_tick.label
            label.set_color(more_grey)
            label.set_family(number_font) 

    # Turn on grid lines for y-only
    #plt.grid(axis='y', color=more_grey)

        
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

def plot_times_old(resarr):
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

def plot_times(resarr, save_filename=None):
    '''Plots the times as returned by get_time_arr().'''
    # plot all the things
    nuns = [0, 2, 1, 3, 1, 2, 4, 2, 3, 5, 1, 3, 1, 2, 5, 4] # dummy in 0 place
    xs = range(1, resarr.shape[1] + 1)
    avgs = np.average(resarr, axis=0)
    stds = np.std(resarr, axis=0)
    more_grey = '#929292'
    color=COLORS[4]
    hardlines = [3, 9, 14]
    newtasks = [1,6,11]

    # first set are downward trend, second set are spikes
    users = [[1, 18, 22], [3, 4, 28]]


    # assuming plot == plot_fx
    ylabel = 'Avg. Marker\nFix Time (s)'
    fig = plt.figure(figsize=(8,5))
    for row_idx, user_group in enumerate(users):
        for col_idx, user in enumerate(user_group):
            ax = plt.subplot(2, # nrows
                3, # ncols
                (row_idx * 3) + col_idx + 1) # plot_number
            # plot y labels on left
            if col_idx == 0:
                plt.ylabel(ylabel)
            # Plot x labels in middle
            if col_idx == 1:
                plt.xlabel('Scenario')
            ax.plot(xs, resarr[user - 1], color=color, linewidth=2.5)

            # for jumpy ones, plot v lines
            if row_idx == 1:
                # Add vertical lines for the hard tasks
                for vloc in hardlines:
                    ax.axvline(x=vloc, ymin=0, ymax=700, color=more_grey, ls=':', lw=2)
                for vloc in newtasks:
                    ax.axvline(x=vloc, ymin=0, ymax=700, color=more_grey, ls='--',
                        lw=2)

            # TODO(max): Split into different functions / graphs. Command line
            # options probably better than constants... maybe... but don't want
            # to have to specify every time...


            #plt.errorbar(xs, avgs, yerr=stds, color=linecolor, linewidth=2.5,
            #    elinewidth=1)

            #Set up minor ticks (locations and labels) for y axis
            #fig = plt.figure(1)
            #ax = plt.axes()
            # minorLocator = AutoMinorLocator(2)
            # minorFormatter = FormatStrFormatter('%d')
            # ax.yaxis.set_minor_formatter(minorFormatter)
            # ax.yaxis.set_minor_locator(minorLocator)

            # Change x axis ticks appropriately (majors on 1, 6, 11)
            majorLocator = FixedLocator([1,6,11])
            minorLocator = FixedLocator(range(2,6) + range(7,11) + range(12,16))
            minorFormatter = FormatStrFormatter('%d')
            ax.xaxis.set_major_locator(majorLocator)
            # ax.xaxis.set_minor_formatter(minorFormatter)
            # ax.xaxis.set_minor_locator(minorLocator)

            # Set title, etc.
            plt.title('User ' + str(user))

            # apply beautiful settings
            beautify(ax)

    plt.tight_layout()
    if save_filename is not None:
        uid = 'ala'
        save_fig(save_filename, uid)
    else:
        plt.show()

def save_fig(save_filename, uid):
    '''Save figure to all formats in formats (in function) and close.
    save_filename should be root directory from which format subdirectories
    are used to house figures.'''
    formats = ['png', 'pdf']    
    if save_filename[-1] != '/':
        save_filename += '/'
    for fmt in formats:
        savedir = save_filename + fmt + '/'
        if not os.path.exists(savedir):
            os.makedirs(savedir)
        savepath = savedir + uid + '.' + fmt
        print 'Saving to', savepath
        plt.savefig(savepath, bbox_inches='tight')
    plt.close()

def print_times(resarr):
    ys = np.average(resarr, axis=0)
    stds = np.std(resarr, axis=0)
    variances = np.square(stds)
    t_mus = np.array([np.sum(ys[:5]), np.sum(ys[5:10]),
        np.sum(ys[10:15])])
    sigs = np.array([np.sum(variances[:5]), np.sum(variances[5:10]),
        np.sum(variances[10:15])])
    t_stds = np.sqrt(sigs)
    code.interact(local=locals())

if __name__ == '__main__':
    save_filename = None
    if len(sys.argv) > 1:
        save_filename = sys.argv[1]
    resarr = get_time_arr()
    # NOTE: set to PLOT_FX before running the following!
    plot_times(resarr, save_filename)
    # NOTE: set to PLOT_SC before running the following!
    #print_times(resarr)
