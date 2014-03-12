'''
Analyze feasability log data.
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
	# setup vars
	baselines = {}
	tests = {}
	for i in range(1, N_TESTS + 1):
		tests[i] = []

	# read through file
	fh = open(logfile, 'r')		
	for line in fh:
		line = line.strip()
		if line.startswith('test_no') or line.startswith('TASK'):
			# just file annotations
			continue

		# Either seed or test data
		pieces = line.split(',')
		if pieces[1] == 'seed':
			baselines[int(pieces[0])] = int(pieces[2])
		else:
			tests[int(pieces[0])].append(int(pieces[3]))
	fh.close()

	# Basic text reporting:
	for i in range(1, N_TESTS + 1):
		print 'Test ' + str(i) + ':'
		print '\tbaseline: ' + str(baselines[i])
		print '\tuser min: ' + str(np.min(tests[i]))
		print '\tuser avg: ' + str(np.mean(tests[i]))
		print '\tuser max: ' + str(np.max(tests[i]))


if __name__ == '__main__':
	if len(sys.argv) == 2:
		main(sys.argv[1])
	else:
		print 'Usage: python feasability_anaylsis.py <log_x.txt>'
		exit(1)