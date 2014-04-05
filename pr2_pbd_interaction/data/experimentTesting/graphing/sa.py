'''
Success Analysis -- Anaylze success-testing
'''

__author__ = "max"


#
# IMPORTS
#

# builtins
import code
import os
import sys

# 3rd party
import numpy as np
import matplotlib.pyplot as plt
import brewer2mpl as b2

#
# CONSTANTS
#
N_TASKS = 3
N_TESTS = 10
N_SCORES = 3

COLORS = b2.get_map('YlOrRd', 'Sequential', 9).mpl_colors
ALMOST_BLACK = '#262626'

#
# CLASSES
#


#
# FUNCTIONS
#

        
#
# MAIN
#

def main(save_filename):
    # So we can load files
    os.chdir(os.path.dirname(os.path.realpath(__file__)))    
    tasks = []
    for i in range(1, N_TASKS + 1):
        data = np.genfromtxt('task' + str(i) + '.txt', delimiter=',',
            dtype='int8')
        tasks += [data]

    lows = range(0, N_TESTS * N_SCORES, N_TESTS)
    highs = range(N_TESTS, N_TESTS * (N_SCORES + 1), N_TESTS)

    color ='#43a2ca'
    color2 = '#a8ddb5'

    colors = [COLORS[4], COLORS[6], COLORS[8]]

    # make top-1 data
    fig = plt.figure(figsize=(8,8))
    more_grey = '#929292'    

    for task_no, task in enumerate(tasks):
        ax = plt.subplot(1, #nrows
            3, #ncols
            task_no + 1) # plot_number
        xs = []
        ys = []
        t5xs = []
        t5ys = []
        for s in range(len(lows)):
            low = lows[s]
            high = highs[s]
            s_data = task[low:high,:]
            # For task 1, do top 5
            if task_no == 0:
                # See if any in top 5 good
                top5 = sum(np.sum(s_data[:,1:] == 1, axis=1) > 0)
                #plt.bar(s + 0.5, top5, color=color2)
                t5xs += [s + 0.5]
                t5ys += [top5]
            # For all tasks, do top 1
            n_good = sum(s_data[:,1] == 1)
            #plt.bar(s + 0.5, n_good, color=color)
            xs += [s + 0.5]
            ys += [n_good]
        plt.grid(axis='y', color=more_grey)                
        # do plotting all at once (old: do for top 5 now)
        if len(t5xs) > 0:
            plt.bar(t5xs, t5ys, color='white', edgecolor=ALMOST_BLACK,
                ls='dotted', label='Top 5')
        # plot each score function in different color
        # Collect lines from last
        lines = []
        for i in range(len(xs)):
            l = plt.bar(xs[i], ys[i], color=colors[i])
            lines += [l]
        plt.title('Task ' + str(task_no + 1), size=20)
        # Only plot x label in middle
        if task_no == 1:
            plt.xlabel('Score function', size=15)
        # Only plot y label on left
        if task_no == 0:
            plt.ylabel('Number successful out of 10 tests', size=15)
        plt.axis([0.25, N_SCORES + 0.5, 0, 10])
        ax = plt.gca()
        ax.set_xticks(np.arange(1, N_SCORES + 1) - 0.125)
        ax.set_xticklabels(['s1', 's2', 's3'])
        #ax.set_xticklabels(['s' + str(i) for i in range(1, N_SCORES + 1)])

        beautify_bar_plot(ax, task_no)

        # # reverse the order of labels in legend so top 1 comes before top 5
        # # (just grabbing here; reverse at legend call)
        # handles, labels = ax.get_legend_handles_labels()

        # # Legend manip from
        # # http://stackoverflow.com/questions/4700614/how-to-put-the-legend-out-of-the-plot

        # Shink current axis's height by 10% on the bottom
        box = ax.get_position()
        ax.set_position([box.x0, box.y0 + box.height * 0.1,
            box.width, box.height * 0.9])
        
        # # Put a legend below current axis
        # ax.legend(handles[::-1], labels[::-1], loc='upper center',
        #     bbox_to_anchor=(0.5, -0.10), fancybox=True, shadow=True, ncol=5)

    # Put a legend below current axis
    legend = fig.legend(lines,
        ['Confidence', 'Seed distance', 'Compactness'],
        loc='lower center',
        frameon=False,
        #bbox_to_anchor=(0.5, -0.10),
        fancybox=True,
        shadow=True,
        ncol=3,
        #title='Score functions',
        prop={'size':15, 'family':'serif'})
    # And no font color property, so now we extract...
    for text in (legend.get_texts() + [legend.get_title()]):
        plt.setp(text, color=ALMOST_BLACK, family='serif')
    if save_filename is not None:
        uid = 'sa_top1-5'
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

def beautify_bar_plot(ax, pnum):
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

if __name__ == '__main__':
    save_filename = None
    if len(sys.argv) > 1:
        save_filename = sys.argv[1]
    main(save_filename)