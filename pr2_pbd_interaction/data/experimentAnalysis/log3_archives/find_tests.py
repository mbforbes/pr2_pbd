import numpy as np

col_task      = 0 # task number
col_nun_start = 1 # number unreachable before fixing (this test action)
col_testdir   = 2 # test directory
col_testact   = 3 # test action (i.e. the "test")
col_userdir   = 4 # user directory / user number
col_useract   = 5 # user action (which they were fixing)
col_nun_res   = 6 # the resulting n. unreachable from user fix -> test
col_nun_user  = 7 # original n. unreachable that user's act. started w/
col_userconf  = 8 # user's confidence in their fix for their action   

log = np.genfromtxt('log_3.txt', delimiter=',', dtype='int8', skip_header=1)

# Look @ task 3
task_data = log[np.where(log[:, col_task] == 3)]
# Want only feasible
feasibile_data = task_data[np.where(task_data[:, col_nun_res] == 0)]
# Find for no. orig. unreachable 1, 3, 4, and 5
for orig_unreach in [1, 3, 4, 5]:
	print 'orig unreach', orig_unreach
	print '--------------'

	un_data = feasibile_data[np.where(feasibile_data[:, col_nun_start]
		== orig_unreach)]
	un_data = un_data[:, [col_testdir, col_testact]]
	prev_d = un_data[0]
	count = 1
	for d in un_data[1:,:]:
		if d[0] == prev_d[0] and d[1] == prev_d[1]:
			count += 1
		else:
			print prev_d, count
			prev_d = d
			count= 1
	print d, count


