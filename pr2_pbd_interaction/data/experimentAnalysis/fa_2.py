'''
Feasability anaylsis

2: newer data type. Here's what it looks like (skipping first line,
which also describes this):

  - [0] task
  - [1] no. unreachable before fixing
  - [2] test dir no.
  - [3] test action no. (test_no)
  - [4] user no.
  - [5] user action no. (scenario no.)
  - [6] no. unreachable result (n_unreachable)
  - [7] user orig. no. unreachable (for user's action)


'''

__author__ = 'max'

### imports

# builtins
import sys
import code

# 3rd party
import numpy as np

# settings
N_TESTS = 15

### main
def main(logfile):
	'''Do analysis.'''
	# load data
	data = np.genfromtxt('log_2.txt', delimiter=',', dtype='int32',
		skip_header=1)

	# settings
	n_runs = 2
	n_splits = 10

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

	# NOTE(max): Test directory and n_unreachable almost fully-specifies a
	# particular test instance... the only time the test action is necessary
	# is in the first task, as there are multiple actions with 1 and 2
	# unreachable start poses.

	# graph 1
	# =======

	# loop tasks (1-3)
	tasks = np.unique(data[:,col_task])	
	for task in tasks:
		task_data = data[np.where(data[:,col_task] == task)]
		nun_starts = np.unique(task_data[:,col_nun_start])
		# loop n unreachable (1-3 for task 1, 1-5 for tasks 2-3)
		for nun_start in nun_starts:
			nun_data = task_data[np.where(
				task_data[:,col_nun_start] == nun_start)]
			test_dirs = np.unique(nun_data[:, col_testdir])
			for test_dir in test_dirs:
				test_dir_data = nun_data[np.where(
					nun_data[:,col_testdir] == test_dir)]

				# TODO (curspot): need to make array to hold the results for
				# each test action; probably need to redo the naming scheme for
				# the inner result arrays to be consistent...

				# test_acts will have multiple elements only for task 1, with 1
				# and 2 start unreachable.
				test_acts = np.unique(nun_data[:, col_testact])
				for test_act in test_acts:
					test_data = test_dir_data[np.where(
						test_dir_data[:,col_testact] == test_act)]
					# Now we have just data from
					# - one task
					# - one n unreachable start
					# - one test directory
					# - one test action
					# and we can randomly pick user fixes
					# Split the data in inreasing amounts of split_frac
					test_act_res = np.zeros(shape=(n_splits, n_runs), dtype='int32')
					for split in range(1, n_splits + 1):
						split_portion = split * split_frac
						split_amt = int(split_portion * len(test_data))
						# Do multiple runs for error bars.
						run_res = np.zeros(n_runs, dtype='int32')
						for run in range(n_runs):
							split_data = test_data[np.random.choice(len(test_data),
								size=split_amt, replace=False)]
							# doing option (a); are there any fixes that get
							# 0 unreachable poses as a result on this test?
							split_res = sum(split_data[:,col_nun_res] == 0)
							run_res[run] = split_res
						test_act_res[split - 1,:] = run_res
						code.interact(local=locals())

if __name__ == '__main__':
	if len(sys.argv) == 2:
		main(sys.argv[1])
	else:
		print 'Usage: python feasability_anaylsis.py <log_x.txt>'
		exit(1)