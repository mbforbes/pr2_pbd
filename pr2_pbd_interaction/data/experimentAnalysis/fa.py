'''
Feasibility anaylsis

Newer data type. Here's what it looks like (ignoring the first line of the file,
which also describes this):

  - [0] task
  - [1] no. unreachable before fixing
  - [2] test dir no.
  - [3] test action no. (test_no)
  - [4] user no.
  - [5] user action no. (scenario no.)
  - [6] no. unreachable result (n_unreachable)
  - [7] user orig. no. unreachable (for user's action)
  - [8] user's confidence in their fix for their action
'''

__author__ = 'mbforbes'



### IMPORTS

# builtins
import sys
import code
import os

# 3rd party
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FixedLocator, LinearLocator
import brewer2mpl as b2

### CONSTANTS
COLORS = b2.get_map('YlOrRd', 'Sequential', 9).mpl_colors
ALMOST_BLACK = '#262626'

### FUNCTIONS
def computeLines(logfile):
    '''Do analysis.'''
    # load data
    data = np.genfromtxt(logfile, delimiter=',', dtype='int8',
        skip_header=1)

    # settings
    n_runs = 100
    n_splits = 1 # ~10 for graph 1, 1 for graph 2

    # compute from settings...
    split_frac = 1.0 / float(n_splits)

    # data shape
    col_task      = 0 # task number
    col_nun_start = 1 # number unreachable before fixing (this test action)
    col_testdir   = 2 # test directory
    col_testact   = 3 # test action (i.e. the "test")
    col_userdir   = 4 # user directory / user number
    col_useract   = 5 # user action (which they were fixing)
    col_nun_res   = 6 # the resulting n. unreachable from user fix -> test
    col_nun_user  = 7 # original n. unreachable that user's action started with
    col_userconf  = 8 # user's confidence in their fix for their action

    # NOTE(max): Test directory and n_unreachable almost fully-specifies a
    # particular test instance... the only time the test action is necessary
    # is in the first task, as there are multiple actions with 1 and 2
    # unreachable start poses.

    # graph2 1, 2
    # ===========

    # data to hang on to (set below during code execution)
    data_amt = 0

    # loop tasks (1-3)
    tasks = np.unique(data[:,col_task]) 
    overall_res = []
    for task in tasks:
        task_data = data[np.where(data[:,col_task] == task)]
        nun_starts = np.unique(task_data[:,col_nun_start])
        task_res = []
        # loop n unreachable (1-3 for task 1, 1-5 for tasks 2-3)
        for nun_start in nun_starts:
            nun_data = task_data[np.where(
                task_data[:,col_nun_start] == nun_start)]
            test_dirs = np.unique(nun_data[:, col_testdir])
            nun_start_res = [] # Not np array. :-(
            for test_dir in test_dirs:
                test_dir_data = nun_data[np.where(
                    nun_data[:,col_testdir] == test_dir)]

                # TODO (curspot): need to make array to hold the results for
                # each test action; probably need to redo the naming scheme for
                # the inner result arrays to be consistent...

                # test_acts will have multiple elements only for task 1, with 1
                # and 2 start unreachable.
                test_acts = np.unique(nun_data[:, col_testact])

                for idx, test_act in enumerate(test_acts):
                    test_data = test_dir_data[np.where(
                        test_dir_data[:,col_testact] == test_act)]
                    # Now we have just data from
                    # - one task
                    # - one n unreachable start
                    # - one test directory
                    # - one test action
                    # and we can randomly pick user fixes
                    # Split the data in inreasing amounts of split_frac
                    data_amt = len(test_data)
                    test_act_res = np.zeros(shape=(n_splits, n_runs))
                    for split in range(1, n_splits + 1):
                        split_portion = split * split_frac
                        split_amt = int(split_portion * len(test_data))
                        # Do multiple runs for error bars.
                        split_res = np.zeros(n_runs)
                        for run in range(n_runs):
                            # Split the data for this run
                            run_data = test_data[np.random.choice(len(
                                test_data), size=split_amt, replace=False)]

                            # Pick what kind of test we're doing!
                            # --------------------------------------------------

                            # graphs 1, 2
                            #
                            # (a); are there any fixes that get 0 unreachable
                            # poses as a result on this test?
                            #run_res = 1 if \
                            #   sum(run_data[:,col_nun_res] == 0) >= 1 else 0

                            # (b) what's the average number of unreachable poses
                            # gotten as a result? sanity check: should be
                            # roughly constant vs amt. of data.
                            #run_res = np.average(run_data[:,col_nun_res])

                            # (c) how many users' fixes got the test to 0
                            # unreachable?
                            #run_res = sum(run_data[:,col_nun_res] == 0)

                            # (d) what potion of users' fixes got the test to 0
                            # unreachable? note: should also be constant vs
                            # amt. of data
                            run_res = sum(run_data[:,col_nun_res] == 0) \
                                / float(split_amt)

                            split_res[run] = run_res
                        test_act_res[split - 1] = split_res
                    nun_start_res.append(test_act_res)
                # Not separating directories because this doesn't matter.
            # At this point, we've run over all test directories, and have
            # a result array of the shape
            #
            # n_test_dirs * n_test_actions, n_splits, n_runs.
            #
            # This is for one task, one no. unreachable start 
            nun_start_res = np.array(nun_start_res)

            # shape: n_splits, n_runs
            nun_start_avg_across_tests = np.average(nun_start_res, axis=0)
            # shape: n_splits
            nun_start_avg_across_runs = np.average(nun_start_avg_across_tests,
                axis=1)
            # shape: n_splits
            nun_start_std_across_runs = np.std(nun_start_avg_across_tests,
                axis=1)
            task_res.append({'avgs': nun_start_avg_across_runs,
                'stds': nun_start_std_across_runs})
        overall_res.append(task_res)

    overall_res = np.array(overall_res)
    # save
    if len(sys.argv) >= 4:
        # allow for providing desired archive name
        save_filename = sys.argv[3]
    else:
        # default. np automatically appends .npz if it doesn't exist, but being
        # explicit here for the sake of clarity
        save_filename = logfile.split('.txt')[0] + '.npz'
    np.savez(save_filename, overall_res=overall_res, data_amt=data_amt)
    print 'Saved in ' + save_filename

def computeConfheat(logfile, save_filename):
    '''Heat map crowd rankings vs n_unreachable result.'''
    data = np.genfromtxt(logfile, delimiter=',', dtype='int8',
        skip_header=1)
    col_task      = 0 # task number
    col_nun_start = 1 # number unreachable before fixing (this test action)
    col_testdir   = 2 # test directory
    col_testact   = 3 # test action (i.e. the "test")
    col_userdir   = 4 # user directory / user number
    col_useract   = 5 # user action (which they were fixing)
    col_nun_res   = 6 # the resulting n. unreachable from user fix -> test
    col_nun_user  = 7 # original n. unreachable that user's action started with
    col_userconf  = 8 # user's confidence in their fix for their action

    # Change 100 ratings to 99 to make them go into the 90-100 bin
    # There's gotta be a better way to do this in vector-notation, but I don't
    # see anything here (http://docs.scipy.org/doc/numpy/reference/
    #     routines.array-manipulation.html) and seems like most other methods
    # return a new copy. So we're looping.
    for i in range(len(data)):
        if data[i][col_userconf] == 100:
            data[i][col_userconf] = 99
    confs = data[:,col_userconf]
    nun_res = data[:,col_nun_res]

    # create plot stuff
    xedges = np.arange(0, 110, 10)
    yedges = np.arange(1, 7, 1)
    plt.figure()
    # Pick colors here:
    # color http://wiki.scipy.org/Cookbook/Matplotlib/Show_colormaps
    plt.hist2d(confs, nun_res, bins=[xedges,yedges], cmap='OrRd')

    # Make and configure colorbar
    cbar = plt.colorbar(orientation='horizontal', ticks=range(0,25000,5000))
    cbar.ax.tick_params(axis='x', which='major', color=ALMOST_BLACK, length=4)
    #cbar.set_ticks([])
    for label in cbar.ax.get_xticklabels():
        label.set_color(ALMOST_BLACK)
    cbar.outline.set_color(ALMOST_BLACK)
    cbar.outline.set_linewidth(1)

    # Configure normal ticks
    ax = plt.gca()
    ax.set_yticks([1.5,2.5,3.5,4.5,5.5])
    ax.set_yticklabels([str(i) for i in [1,2,3,4,5]])

    # labels
    plt.title('Crowd confidence vs feasibility of fixes', size=20)
    plt.ylabel('Result no. unreachable poses', size=15)
    plt.xlabel('Confidence from 0 - 100', size=15)
    beautify_heat_plot(ax)
    if save_filename is not None:
        uid = 'fa_confheat'
        save_fig(save_filename, uid)
    else:
        plt.show()
 
def computeExplore(logfile):
    '''Check out different trends in data.

    Currently seeing how users' start no. unreachable affects their performance
    in getting any (x)-no unreachable to (y)-no unreachable. This is to see if
    certain start configurations result in certain end configurations (are there
    patterns in the input data). Here are the axes:

    - x: start no. unreachable
    - y: end no. unreachable
    - different lines: user's start no. unreachable
    '''
    # settings
    colors = [
        '#66c2a5',
        '#fc8d62',
        '#8da0cb',
        '#e78ac3',
        '#a6d854',
    ]

    data = np.genfromtxt(logfile, delimiter=',', dtype='int8',
        skip_header=1)
    col_task      = 0 # task number
    col_nun_start = 1 # number unreachable before fixing (this test action)
    col_testdir   = 2 # test directory
    col_testact   = 3 # test action (i.e. the "test")
    col_userdir   = 4 # user directory / user number
    col_useract   = 5 # user action (which they were fixing)
    col_nun_res   = 6 # the resulting n. unreachable from user fix -> test
    col_nun_user  = 7 # original n. unreachable that user's action started with
    col_userconf  = 8 # user's confidence in their fix for their action

    # loop tasks (1-3)
    tasks = np.unique(data[:,col_task])
    for task in tasks:
        fig = plt.figure(figsize=(8,8))
        ax = plt.subplot(111)
        task_data = data[np.where(data[:,col_task] == task)]
        nun_users = np.unique(task_data[:,col_nun_user])
        # loop through nun_user (users' start no. unreachable)
        for nun_user in nun_users:
            nun_user_data = task_data[np.where(task_data[:,col_nun_user] ==
                nun_user)]
            points = []
            stds = []
            start_nuns = np.unique(nun_user_data[:,col_nun_start])
            # loop through start no. unreachable (tests cases)
            for start_nun in start_nuns:
                start_nun_data = nun_user_data[np.where(
                    nun_user_data[:,col_nun_start] == start_nun)]
                # the data we have now is for:
                # - one task
                # - one user start no. unreachable
                # - one task start no. unreachable
                # and we want to compute the average result
                avg = np.average(start_nun_data[:, col_nun_res])
                std = np.std(start_nun_data[:, col_nun_res])
                points += [avg]
                stds += [std]
            plt.errorbar(start_nuns, points, label=str(nun_user),
                color=colors[nun_user - 1], linewidth=2)#, yerr=stds)
        plt.title('Task ' + str(task))
        plt.xlabel('test start no. unreachable')
        plt.ylabel('test end no. unreachable')
        plt.axis([0, len(start_nuns) + 1, 0, 6])

        # Legend manip. from 
        # http://stackoverflow.com/questions/4700614/how-to-put-the-legend-out-of-the-plot
        
        # Shink current axis's height by 10% on the bottom
        box = ax.get_position()
        ax.set_position([box.x0, box.y0 + box.height * 0.1,
            box.width, box.height * 0.9])

        # Put a legend below current axis
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.10),
            fancybox=True, shadow=True, ncol=5,
            title='Users\' starting no. unreachable')
    plt.show()


def plot1(logfile, plot_let, save_filename=None):
    '''Plot line graph 1, which is is, per task,
    - x-axis: number of fixes (amt. of data)
    - y-axis: portion of tests|fixes feasible.
    - one color per n. unreachable test (3 or 5 lines)
    ''' 
    # debug
    print os.getcwd()

    # settings
    styles = ['-', '--', '-.', ':', '--']
    markers = ['*', '^', 'D', 's', '8']

    # load data
    filedata = np.load(logfile)
    overall_res = filedata['overall_res']
    data_amt = filedata['data_amt']
    print 'Loaded from ' + logfile

    n_splits = len(overall_res[0][0]['avgs'])
    step = 1.0 / float(n_splits)
    xs = np.arange(0.0, 1.0 + step, step) * data_amt

    fig = plt.figure(figsize=(12,8))
    bigax = plt.gca()
    for i, task in enumerate(overall_res):
        ax = plt.subplot(1, # nrows
            3, # ncols
            i + 1) # plot_number
        plt.title('Task ' + str(i + 1))
        # Plot x axis only in middle
        if i == 1:
            plt.xlabel('User fixes')

        # Because graphs a bit more cramped, only plt y axis once (left)
        if i == 0:
            if plot_let == 'a':
                # a
                plt.ylabel('Portion of tests made feasible')
            elif plot_let == 'd':
                # d
                plt.ylabel('Portion of fixes feasible on test set')
            else:
                # bad
                print 'Unsupported plot letter:', plot_let
                exit(1)

        # Collect lines (only using them from last plot) to make legend.
        lines = []
        for idx, nun_start in enumerate(task):
            l = ax.errorbar(x=xs,
                y=np.insert(nun_start['avgs'], 0, 0),
                color=COLORS[idx + 4],
                lw=2,
                ls=styles[idx],
                marker=markers[idx],
                elinewidth=1,
                yerr=np.insert(nun_start['stds'], 0, 0),
                label=str(idx + 1))
            lines += [l]
        plt.axis([0, data_amt + 10, 0, 1.05])
        # shrink x axis ticks
        majorLocator = FixedLocator(range(0, 200, 40))
        ax.xaxis.set_major_locator(majorLocator)

        # Do beautification
        beautify_line_plots(ax, i, 1)

        # Shink current axis's height by 10% on the bottom
        box = ax.get_position()
        ax.set_position([box.x0, box.y0 + box.height * 0.1, box.width,
            box.height * 0.9])

    # Put a legend below current axis
    legend = fig.legend(lines,
        [str(n) for n in range(1,6)],
        loc='lower center',
        frameon=False,
        #bbox_to_anchor=(0.5, -0.10),
        fancybox=True,
        shadow=True,
        ncol=5,
        title='Starting no. unreachable poses',
        prop={'size':15, 'family':'serif'})
    # And no font color property, so now we extract...
    for text in (legend.get_texts() + [legend.get_title()]):
        plt.setp(text, color=ALMOST_BLACK, family='serif')
    if save_filename is not None:
        uid = 'fa_1' + plot_let
        save_fig(save_filename, uid)
    else:
        plt.show()

def plot2(logfile, plot_let, save_filename=None):
    '''Plot line graph 2, which is is, per task,
    - x-axis: n_unreachable
    - y-axis: portion of tests|fixes feasible.
    - one color (single line)
    '''
    # load data
    filedata = np.load(logfile)
    overall_res = filedata['overall_res']
    data_amt = filedata['data_amt']
    print 'Loaded from ' + logfile
    fig = plt.figure(figsize=(8,8))
    for i, task in enumerate(overall_res):
        nun_opts = len(task)
        xs = range(1, nun_opts + 1)
        ax = plt.subplot(1, # nrows
            3, # ncols
            i + 1) # plot_number        
        plt.title('Task ' + str(i + 1))
        # only xlabel for middle
        if i == 1:
            plt.xlabel('Start no. unreachable')
        # only ylabel for left (first)
        if i == 0:
            if plot_let == 'a':            
                # a
                plt.ylabel('Portion of tests made feasibile')
            elif plot_let == 'd':
                # d
                plt.ylabel('Portion of fixes feasibile on test set')
            else:
                # bad
                print 'Unsupported plot letter:', plot_let
                exit(1)

        ys, stds = [], []
        for nun_start in task:
            ys += [nun_start['avgs'][0]]
            stds += [nun_start['stds'][0]]
        ax.plot(xs,
            ys,
            color=COLORS[4], # use mid orange
            linewidth=2,
            marker='o',
            markeredgecolor=ALMOST_BLACK)
        if plot_let == 'd':
            plt.axis([0.5, nun_opts + 0.5, 0, 0.5])
        elif plot_let == 'a':
            plt.axis([0.5, nun_opts + 0.5, 0.2, 1.05])
        # need to change major locator for first plot
        if i == 0:            
            majorLocator = FixedLocator(range(1,4))
            ax.xaxis.set_major_locator(majorLocator)
        beautify_line_plots(ax, i, 2)
    if save_filename is not None:
        uid = 'fa_2' + plot_let
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

def beautify_line_plots(ax, pnum, gnum):
    '''Modified beautify :-/
    '''
    # Settings
    more_grey = '#929292'
    text_font = 'serif'
    number_font = 'serif'

    # Get the figure and axes.
    if ax is None:
        fig = plt.figure(1)
        ax = plt.axes()

    # Remove 'spines' (axis lines)
    spines_to_remove = ['top', 'right']
    if pnum > 0:
        spines_to_remove += ['left']
    for spine in spines_to_remove:
        ax.spines[spine].set_visible(False)

    # Tick only on leftmost left graph
    if pnum > 0:
        ax.yaxis.set_ticks_position('none')
        ax.set_yticklabels([])
    else:
        ax.yaxis.tick_left()

    # change ticks so on both in middle graph, only on left in right graph
    # if pnum == 2:
    #     ax.yaxis.tick_left()
    ax.xaxis.tick_bottom()

    # To remove the ticks all-together (like in prettyplotlib), do the following
    # instead of tick_left() and tick_bottom()
    #ax.xaxis.set_ticks_position('none')
    #ax.yaxis.set_ticks_position('none')

    # Now make them go 'out' rather than 'in'
    for axis in ['x', 'y']:
        ax.tick_params(axis=axis, which='both', direction='out', pad=7)
        ax.tick_params(axis=axis, which='major', color=ALMOST_BLACK, length=6)
        ax.tick_params(axis=axis, which='minor', color=more_grey, length=4)

    # Make thinner and off-black
    spines_to_keep = ['bottom', 'left']
    for spine in spines_to_keep:
        ax.spines[spine].set_linewidth(0.5)
        ax.spines[spine].set_color(ALMOST_BLACK)

    # Change the labels & title to the off-black and change their font
    for label in [ax.yaxis.label, ax.xaxis.label, ax.title]:
        label.set_color(ALMOST_BLACK)
        label.set_family(text_font)
        label.set_fontsize(20)

    # Change the tick labels' color and font and padding
    for axis in [ax.yaxis, ax.xaxis]:
        # padding
        #axis.labelpad = 20
        # major ticks
        for major_tick in axis.get_major_ticks():
            label = major_tick.label
            label.set_color(ALMOST_BLACK)
            label.set_family(number_font)
        # minor ticks
        for minor_tick in axis.get_minor_ticks():
            label = minor_tick.label
            label.set_color(more_grey)
            label.set_family(number_font)

    # Turn on grid lines for y always
    plt.grid(axis='y', color=more_grey)
    # And for first graph, do x lines as well
    if gnum == 1:
        plt.grid(axis='x', color=more_grey)

def beautify_heat_plot(ax):
    '''Modified beautify :-/
    '''
    # Settings
    more_grey = '#929292'
    text_font = 'serif'
    number_font = 'serif'

    # Get the figure and axes.
    if ax is None:
        fig = plt.figure(1)
        ax = plt.axes()

    # To remove the ticks all-together (like in prettyplotlib), do the following
    # instead of tick_left() and tick_bottom()
    ax.xaxis.set_ticks_position('none')
    ax.yaxis.set_ticks_position('none')

    # Now make them go 'out' rather than 'in'
    # for axis in ['x', 'y']:
    #     ax.tick_params(axis=axis, which='both', direction='out', pad=7)
    #     ax.tick_params(axis=axis, which='major', color=ALMOST_BLACK, length=6)
    #     ax.tick_params(axis=axis, which='minor', color=more_grey, length=4)

    # Make thinner and off-black
    spines_to_keep = ['bottom', 'left', 'top', 'right']
    for spine in spines_to_keep:
        ax.spines[spine].set_linewidth(0.5)
        ax.spines[spine].set_color(ALMOST_BLACK)

    # Change the labels & title to the off-black and change their font
    for label in [ax.yaxis.label, ax.xaxis.label, ax.title]:
        label.set_color(ALMOST_BLACK)
        label.set_family(text_font)
        label.set_fontsize(20)

    # Change the tick labels' color and font and padding
    for axis in [ax.yaxis, ax.xaxis]:
        # padding
        #axis.labelpad = 20
        # major ticks
        for major_tick in axis.get_major_ticks():
            label = major_tick.label
            label.set_color(ALMOST_BLACK)
            label.set_family(number_font)
        # minor ticks
        for minor_tick in axis.get_minor_ticks():
            label = minor_tick.label
            label.set_color(more_grey)
            label.set_family(number_font)

def usage():
    '''Tell 'em what to do.'''
    print 'Usage: python fa.py <log_x.<txt|npz>> [comp_opt|plot_opt] [save_path]'
    print 'comp_opt: lines|explore|confheat'
    print 'plot_opt: 1[a|d]|2[a|d]'
    print 'save_path: for plots, will save in pdf/ and png/ subdirs under arg'
    exit(1)

# Program enters here
if __name__ == '__main__':
    if len(sys.argv) >= 2:
        logfile = sys.argv[1]
        if logfile.endswith('.txt'):
            # Crunch lines / plot
            if len(sys.argv) >= 3:
                comp_opt = sys.argv[2]
                save_filename = None
                if len(sys.argv) >= 4:
                    save_filename = sys.argv[3]
                if comp_opt == 'lines':
                    computeLines(logfile)
                elif comp_opt == 'explore':
                    computeExplore(logfile)
                elif comp_opt == 'confheat':
                    computeConfheat(logfile, save_filename)
                else:
                    usage()
            else:
                usage()
        elif logfile.endswith('.npz'):
            save_filename = None
            if len(sys.argv) >= 4:
                save_filename = sys.argv[3]
            # Plot lines
            if len(sys.argv) >= 3:
                plot_num, plot_let = sys.argv[2]
                if plot_num == '1':
                    plot1(logfile, plot_let, save_filename)
                elif plot_num == '2':
                    plot2(logfile, plot_let, save_filename)
                else:
                    usage()
            else:
                usage()
        else:
            usage()
    else:
        usage()


