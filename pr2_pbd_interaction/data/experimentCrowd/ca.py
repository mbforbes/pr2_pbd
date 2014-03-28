'''
Confidences Analysis -- Anaylze user-provided confidence measures
'''

__author__ = "max"


#
# IMPORTS
#
# builtins
import code

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


#
# CLASSES
#


#
# FUNCTIONS
#

        
#
# MAIN
#

def heat():
    '''Do some crowd confidence analysis!'''
    nuns = [2, 1, 3, 1, 2, 4, 2, 3, 5, 1, 3, 1, 2, 5, 4]
    data = np.genfromtxt('confidences.txt', delimiter=',', dtype='int8')

    fnuns = np.repeat(nuns, len(data))
    fdata = data.flatten()

    xedges = np.arange(1,7,1)
    yedges = np.arange(0,110,10)

    for i in range(len(fdata)):
        # Change 100s to 99s for easier plotting (doesn't change rep.)
        if fdata[i] == 100:
            fdata[i] = 99
    # Pick colors here:
    # http://wiki.scipy.org/Cookbook/Matplotlib/Show_colormaps
    plt.figure()
    plt.hist2d(fnuns, fdata, bins=[xedges,yedges], cmap='OrRd')
    plt.colorbar()
    ax = plt.gca()
    ax.set_xticks([1.5,2.5,3.5,4.5,5.5])
    ax.set_xticklabels([str(i) for i in [1,2,3,4,5]])

    # labels
    plt.title('Crowd confidence', size=20)
    plt.xlabel('Start no. unreachable poses', size=15)
    plt.ylabel('Confidence from 0 - 100', size=15)

    plt.show()

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
    heat()
    line()