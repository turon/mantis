#!/usr/bin/python

import os, sys

pids = []

for i in range(0,25):
	print 'Launching: prog %s /dev/ttyUSB%d' % (sys.argv[1], i)
	pid = os.spawnlp(os.P_NOWAIT, 'prog', 'prog', sys.argv[1], '/dev/ttyUSB%d'%i)
	pids.append(pid)

for i in range(0,25):
	pid, ret = os.waitpid(pids[i], 0)
	if ret:
		print 'prog %s /dev/ttyUSB%d returned %d' % (sys.argv[1], i, ret)

