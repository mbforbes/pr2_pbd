'''
Success Analysis -- Anaylze success-testing
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

#
# CONSTANTS
#
N_TASKS = 3
N_TESTS = 10
N_SCORES = 3


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

    tasks = []
    for i in range(1, N_TASKS + 1):
        data = np.genfromtxt('task' + str(i) + '.txt', delimiter=',',
            dtype='int8')
        tasks += [data]

    lows = range(0, N_TESTS * N_SCORES, N_TESTS)
    highs = range(N_TESTS, N_TESTS * (N_SCORES + 1), N_TESTS)
    color ='#43a2ca'
    color2 = '#a8ddb5'
    # make top-1 data
    for task_no, task in enumerate(tasks):
        plt.figure(figsize=(8,8))
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
        # do plotting all at once
        if len(t5xs) > 0:
            plt.bar(t5xs, t5ys, color=color, label='Top 5')
        plt.bar(xs, ys, color=color2, label='Top 1')
        ax = plt.gca()
        plt.title('Task ' + str(task_no + 1), size=20)
        plt.xlabel('Score function', size=15)
        plt.ylabel('Number successful out of 10 tests', size=15)
        plt.axis([0.25, N_SCORES + 0.5, 0, 11])
        ax = plt.gca()
        ax.set_xticks(np.arange(1, N_SCORES + 1) - 0.125)
        ax.set_xticklabels(['Confidence', 'Seed distance', 'Compactness'])
        #ax.set_xticklabels(['s' + str(i) for i in range(1, N_SCORES + 1)])

        # reverse the order of labels in legend so top 1 comes before top 5
        # (just grabbing here; reverse at legend call)
        handles, labels = ax.get_legend_handles_labels()

        # Legend manip from
        # http://stackoverflow.com/questions/4700614/how-to-put-the-legend-out-of-the-plot

        # Shink current axis's height by 10% on the bottom
        box = ax.get_position()
        ax.set_position([box.x0, box.y0 + box.height * 0.1,
                         box.width, box.height * 0.9])
        
        # Put a legend below current axis
        ax.legend(handles[::-1], labels[::-1], loc='upper center',
            bbox_to_anchor=(0.5, -0.10), fancybox=True, shadow=True, ncol=5)

    plt.show()

if __name__ == '__main__':
    main()