'''
Confidences Analysis -- Anaylze user-provided confidence measures
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
from matplotlib.colors import LogNorm

# custom
import prettyplotlib as ppl
from prettyplotlib import brewer2mpl

#
# CONSTANTS
#

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

def heat(save_filename=None):
    '''Do some crowd confidence analysis!'''
    nuns = [2, 1, 3, 1, 2, 4, 2, 3, 5, 1, 3, 1, 2, 5, 4]
    # So we can load the file
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    data = np.genfromtxt('confidences.txt', delimiter=',', dtype='int8')

    # Copy nuns len(data) times into big flat array (fnuns).
    fnuns = np.zeros(len(data) * len(nuns))
    for i in range(len(data)):
        fnuns[i*len(nuns):(i+1)*len(nuns)] = nuns

    # Old way that I think was buggy.
    #fnuns = np.repeat(nuns, len(data))
    fdata = data.flatten()

    xedges = np.arange(1,7,1)
    yedges = np.arange(0,110,10)

    for i in range(len(fdata)):
        # Change 100s to 99s for easier plotting (doesn't change rep.)
        if fdata[i] == 100:
            fdata[i] = 99
    # Pick colors here:
    # http://wiki.scipy.org/Cookbook/Matplotlib/Show_colormaps
    plt.figure(figsize=(8,6))
    plt.hist2d(fnuns, fdata, bins=[xedges,yedges], cmap='OrRd')

    # Make and configure colorbar
    cbar = plt.colorbar(orientation='horizontal')
    cbar.ax.tick_params(axis='x', which='major', color=ALMOST_BLACK, length=4)
    #cbar.set_ticks([])
    for label in cbar.ax.get_xticklabels():
        label.set_color(ALMOST_BLACK)
    cbar.outline.set_color(ALMOST_BLACK)
    cbar.outline.set_linewidth(1)

    ax = plt.gca()
    ax.set_xticks([1.5,2.5,3.5,4.5,5.5])
    ax.set_xticklabels([str(i) for i in [1,2,3,4,5]])

    # labels
    #plt.title('Crowd confidence', size=20)
    plt.xlabel('Start no. unreachable poses', size=15)
    plt.ylabel('Confidence from 0 - 100', size=15)

    # Prettify
    beautify_heat_plot(ax)

    if save_filename is not None:
        uid = 'ca_startheat'
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

def line():
    lw = 2
    color = '#43a2ca'
    xlab = 'Start no. unreachable poses'
    ylab = 'Confidence from 0 - 100'

    nuns = [2, 1, 3, 1, 2, 4, 2, 3, 5, 1, 3, 1, 2, 5, 4]
    data = np.genfromtxt('confidences.txt', delimiter=',', dtype='int8')
    xs = []
    avgs = []
    stds = []
    # task 1
    xs = [1,2,3]
    avgs += [np.average(data[:,[1,3]])]
    stds += [np.std(data[:,[1,3]])]
    avgs += [np.average(data[:,[0,4]])]    
    stds += [np.std(data[:,[0,4]])]
    avgs += [np.average(data[:,2])]
    stds += [np.std(data[:,2])]
    plt.figure()
    plt.errorbar(xs, avgs, yerr=stds, color=color, linewidth=lw)
    plt.title('Crowd confidence for Task 1', size=20)
    plt.xlabel(xlab, size=15)
    plt.ylabel(ylab, size=15)
    ax = plt.gca()
    ax.set_xticks([1,2,3])
    plt.axis([0, 4, 0, 100])

    # tasks 2 and 3
    for t in range(2,4):
        plt.figure()
        xs = {}
        avgs = {}
        stds = {}
        for i in range((t - 1) * 5,t * 5):
            nun = nuns[i]
            xs[nun] = nun
            avgs[nun] = np.average(data[:,i])
            stds[nun] = np.std(data[:,i])
        fxs = []
        favgs = []
        fstds = []
        for i in range(1, 6):
            fxs += [i]
            favgs += [avgs[i]]
            fstds += [stds[i]]
        plt.errorbar(fxs, favgs, yerr=fstds, color=color, linewidth=lw)
        plt.title('Crowd confidence for Task ' + str(t), size=20)
        plt.xlabel(xlab, size=15)
        ax = plt.gca()
        ax.set_xticks([1,2,3,4,5])
        plt.ylabel(ylab, size=15)
        plt.axis([0, 6, 0, 110])

    # show all
    plt.show()

if __name__ == '__main__':
    save_filename = None
    if len(sys.argv) > 1:
        save_filename = sys.argv[1]
    heat(save_filename)
    #line()