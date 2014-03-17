'''
Feasibility anaylsis

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
  - [8] user's confidence in their fix for their action


'''

__author__ = 'max'

### imports

# builtins
import sys
import code

# 3rd party
import numpy as np
import matplotlib.pyplot as plt

### stuff
def computeLines(logfile):
	'''Do analysis.'''
	# load data
	data = np.genfromtxt(logfile, delimiter=',', dtype='int8',
		skip_header=1)

	# settings
	n_runs = 100
	n_splits = 10 # ~10 for graph 1, 1 for graph 2

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

	# data to hang on to (saved in code)
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
							#	sum(run_data[:,col_nun_res] == 0) >= 1 else 0

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

def computeExplore(logfile):
	'''Check out different trends in data.'''
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
				points += [avg]
			plt.plot(start_nuns, points, label=str(nun_user),
				color=colors[nun_user - 1], linewidth=2)
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

def plot2(logfile):
	# load data
	filedata = np.load(logfile)
	overall_res = filedata['overall_res']
	data_amt = filedata['data_amt']
	print 'Loaded from ' + logfile

	for i, task in enumerate(overall_res):
		nun_opts = len(task)
		xs = range(1, nun_opts + 1)
		fig = plt.figure(figsize=(4,8))
		ax = plt.subplot(111)
		plt.title('Task ' + str(i + 1))
		plt.xlabel('Start no. unreachable')
		# a
		#plt.ylabel('Portion of tests feasibile')
		# d
		plt.ylabel('Portion of fixes feasibile')
		ys, stds = [], []
		for nun_start in task:
			ys += [nun_start['avgs'][0]]
			stds += [nun_start['stds'][0]]
		ax.errorbar(x=xs, y=ys, color='#222222', linewidth=2, yerr=stds)
		plt.axis([0, nun_opts + 1, 0, 1])
		plt.tight_layout()
	plt.show()

def plot1(logfile):
	'''Do plotting'''
	# settings
	colors = [
		'#66c2a5',
		'#fc8d62',
		'#8da0cb',
		'#e78ac3',
		'#a6d854',
	]

	# another option
	# (check this out for more)
	# http://colorbrewer2.org/
	#colors = [
	#	'#ccebc5',
	#	'#a8ddb5',
	#	'#7bccc4',
	#	'#43a2ca',
	#	'#0868ac',
	#]


	# load data
	filedata = np.load(logfile)
	overall_res = filedata['overall_res']
	data_amt = filedata['data_amt']
	print 'Loaded from ' + logfile

	n_splits = len(overall_res[0][0]['avgs'])
	step = 1.0 / float(n_splits)
	xs = np.arange(0.0, 1.0 + step, step) * data_amt

	for i, task in enumerate(overall_res):
		fig = plt.figure(figsize=(8,8))
		ax = plt.subplot(111)
		plt.title('Task ' + str(i + 1))
		plt.xlabel('User fixes')
		# a
		plt.ylabel('Portion of tests feasible')
		# d
		#plt.ylabel('Portion of fixes feasible')
		for idx, nun_start in enumerate(task):
			ax.errorbar(x=xs,
				y=np.insert(nun_start['avgs'], 0, 0),
				color=colors[idx],
				linewidth=2,
				yerr=np.insert(nun_start['stds'], 0, 0),
				label=str(idx + 1))
			plt.axis([0, data_amt, 0, 1])

		# Legend manip. from 
		# http://stackoverflow.com/questions/4700614/how-to-put-the-legend-out-of-the-plot
		
		# Shink current axis's height by 10% on the bottom
		box = ax.get_position()
		ax.set_position([box.x0, box.y0 + box.height * 0.1,
			box.width, box.height * 0.9])

		# Put a legend below current axis
		ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.10),
			fancybox=True, shadow=True, ncol=5, title='Starting no. unreachable')
	plt.show()

def usage():
	'''Tell 'em what to do.'''
	print 'Usage: python pa_2.py <log_x.<txt|npz>> [comp_opt|plot_opt] [save_fielname]'
	print 'comp_opt: lines|explore'
	print 'plot_opt: 1|2'
	exit(1)

# Program enters here
if __name__ == '__main__':
	if len(sys.argv) >= 2:
		logfile = sys.argv[1]
		if logfile.endswith('.txt'):
			# Crunch
			if len(sys.argv) >= 3:
				comp_opt = sys.argv[2]
				if comp_opt == 'lines':
					computeLines(logfile)
				elif comp_opt == 'explore':
					computeExplore(logfile)
			else:
				# Compute lines if nothing specified
				computeLines(logfile)
		elif logfile.endswith('.npz'):
			# Plot
			if len(sys.argv) >= 3:
				plot_opt = int(sys.argv[2])
				if plot_opt == 1:
					plot1(logfile)
				elif plot_opt == 2:
					plot2(logfile)
				else:
					usage()
			else:
				# Just do plot 1 if nothing specified.
				plot1(logfile)
		else:
			usage()
	else:
		usage()


