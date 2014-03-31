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

def main():
    '''Do some crowd action log analysis!'''
    # settings
    low_u = 4 # low user num (inclusive)
    high_u = 4 # high user num (inclusive)
    nuns = [0, 2, 1, 3, 1, 2, 4, 2, 3, 5, 1, 3, 1, 2, 5, 4] # dummy in 0 place
    n_scenarios = len(nuns) - 1 # dummy in 0 place; subtract 1 for len
    xs = range(1, n_scenarios + 1)
    basedir = '/home/mbforbes/rosbuild_ws/pr2_pbd/pr2_pbd_interaction/data/'

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

    # plot all the things
    #code.interact(local=locals())
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

    plt.figure()
    plt.title(title)
    plt.xlabel('Scenario')
    plt.ylabel(ylabel)
    plt.errorbar(xs, ys, yerr=stds)
    plt.show()

if __name__ == '__main__':
    main()