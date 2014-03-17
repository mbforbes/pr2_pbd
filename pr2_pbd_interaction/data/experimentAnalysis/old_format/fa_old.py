'''
Feasability anaylsis

1 - old data type. For newer data type, see fa_2
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
	tests = {}
	for i in range(1, N_TESTS + 1):
		tests[i] = {}
		tests[i]['res'] = []

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
			tests[int(pieces[0])]['baseline'] = int(pieces[2])
		else:
			tests[int(pieces[0])]['res'].append(int(pieces[3]))
	fh.close()

	# Processing (was basic text reporting).
	for i in range(1, N_TESTS + 1):
		arr = np.array(tests[i]['res'])

		#print 'Test ' + str(i) + ':'
		#print '\tbaseline: ' + str(tests[i]['baseline'])
		tests[i]['min'] = np.min(arr)
		#print '\tuser min: ' + str(arr)
		n_zero = sum(arr == 0)
		total = len(arr)
		p_zero = (float(n_zero) / total) * 100
		tests[i]['n_zero'] = n_zero
		tests[i]['p_zero'] = p_zero
		p_zero_str = '%0.2f' % (p_zero)
		tests[i]['p_zero_str'] = p_zero_str
		#print '\tnum zero: ' + str(n_zero) + ' (' + p_zero_str + '%)'
		tests[i]['avg'] = np.mean(arr)
		#print '\tuser avg: ' + str(np.mean(arr)
		tests[i]['max'] = np.max(arr)
		#print '\tuser max: ' + str(np.max(arr))

	# More grouped reporting
	n_fixes = len(tests[1]['res'])
	print 'Number of fixes (out of ' + str(n_fixes) + ') that reached feasibility'
	print '(going from x to 0 unreachable) by task:'
	for i in range(1, 4):
		print 'Task ' + str(i)
		valid_testnos = range((i - 1) * 5 + 1, i * 5 + 1)
		for b in range(1, 6):
			ts = [t for idx,t in tests.iteritems() if t['baseline'] == b and idx in valid_testnos]
			if len(ts) > 0:
				sys.stdout.write('\t' + str(b) + ' unreachable: ' )
				for idx, t in enumerate(ts):
					sys.stdout.write(str(t['n_zero']) + ' (' + t['p_zero_str'] + '%)') 
					if idx == len(ts) - 1:
						# last
						sys.stdout.write('\n')
					else:
						# more to come
						sys.stdout.write(', ')


if __name__ == '__main__':
	if len(sys.argv) == 2:
		main(sys.argv[1])
	else:
		print 'Usage: python feasability_anaylsis.py <log_x.txt>'
		exit(1)