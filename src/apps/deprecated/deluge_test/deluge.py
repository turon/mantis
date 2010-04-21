#!/usr/bin/python

import sys, cPickle, os, re

# structure of data:
#
# stuff.startTime		- time experiment started
# stuff.finishTime		- time experiment ended
# stuff.stats[id]		- map nodes to packet tables
# stuff.stats[id][id]		- map nodes to node of counts
# stuff.stats[id][id].summary	- summaries send by node
# stuff.stats[id][id].profile	- etc
# stuff.stats[id][id].request
# stuff.stats[id][id].data
# stuff.stats[id][id].total
# stuff.neighbors[id][]		- list of (id, dtb) tuples

protos = ['','Aqueduct', 'Aqueduct-no-symmetry', 'Deluge']

class empty:
	pass

def doPacketTotals(nodes, totals):
	for id, node in nodes.iteritems():
		s = node.stats[id].summary
		p = node.stats[id].profile
		r = node.stats[id].request
		d = node.stats[id].data

		totals[0] += s
		totals[1] += p
		totals[2] += r
		totals[3] += d
		totals[4] += s + p + r + d

def doCacheTotals(nodes, totals):
	for id, node in nodes.iteritems():
		h = node.hits
		m = node.misses
		hf = node.hitForward
		hpf = node.hitPastForward

		totals[0] += h
		totals[1] += m
		totals[2] += hf
		totals[3] += hpf

def printRequestHistogram(nodes):
	hist = {}
	for id, record in nodes.iteritems():
		nr = record[id].request
		if nr not in hist:
			hist[nr] = 0
		hist[nr] += 1
	for n in hist.iteritems():
		print '%d: %d' % n
	print

def printPacketCounts(stuff, fname, grandTotals):
	size = int(fname[-10:-8])
	time = stuff.finishTime - stuff.startTime
	totals = [0,0,0,0,0]
	doPacketTotals(stuff.nodes, totals)
	#print protos[proto]
	print ' %4d   %5d %7d %8d %8d %6d   %6d' % (size, time, totals[0], totals[1], totals[2], totals[3], totals[4])
	
	if fname[1]=='1':
		if fname[3]=='1':
			grandTotals[0] += 18 * totals[0]
		if fname[3]=='2':
			grandTotals[0] += 8 * totals[0]
		if fname[3]=='3':
			grandTotals[0] += 7 * totals[0]
		grandTotals[1] += 13 * totals[1]
		grandTotals[2] += 16 * totals[2]
		grandTotals[3] += 64 * totals[3]
		grandTotals[4] += 56 * totals[3]

def printCacheCounts(stuff, fname):
	size = int(fname[-10:-8])
	time = stuff.finishTime - stuff.startTime
	totals = [0,0,0,0,0]
	doCacheTotals(stuff.nodes, totals)
	print ' %4d       %6d     %6d        %6d                 %6d' % (size,totals[0],totals[1],totals[2],totals[3])

def advertRates(nodes, completionTime, size):
	for id, node in nodes.iteritems():
		#print float(node.stats[id].summary) / float(completionTime)
		#print node.stats[id].summary
		#print node.stats[id].request
		#print node.stats[id].data
		total = float(0)
		for id2, record in node.stats.iteritems():
			if id == id2: continue
			total += record.data
		print total / float(48 * size)
	
def printCacheHeadings():
	print '#     ||          |          |Request hit  |Request hit old e'
	print '#Size || Cache hit|Cache miss|FORWARD state|partially downloaded page'
	print '#---- || ---------|----------|-------------|-------------------------'

def printPacketHeadings():
	print '#Size | Time  Adverts Profiles Requests  Data  | Total'
	print '#---- | ----- ------- -------- -------- ------ | ------'

e1match = re.compile('^e1_\d_\d\d_\d\d.pdat$')
e4match = re.compile('^e4_\d\d_\d\d.pdat$')

def main():
	stuffs = []
	fname = sys.argv[1]
	if fname == '-a':
		files = [f for f in os.listdir('.') if e1match.search(f) or e4match.search(f)]
		files.sort()
		for fname in files:
			#print 'Opening', fname
			stream = file(fname)
			stuffs.append((fname, cPickle.load(stream)))
			stream.close()
	else:
		stream = file(fname)
		stuffs.append((fname, cPickle.load(stream)))
		stream.close()
	
	grandTotals = [0, 0, 0, 0, 0]
	printPacketHeadings()
	for fname, stuff in stuffs:
		printPacketCounts(stuff, fname, grandTotals)
	print
	print
	printCacheHeadings()
	for fname, stuff in stuffs:
		if fname[1]=='4':
			printCacheCounts(stuff, fname)
	if fname[1]=='1':
		total = grandTotals[0]+grandTotals[1]+grandTotals[2]+grandTotals[3]
		print
		print
		print '#Summary | Profile | Request |  Data  | Payload |  Total  | Overhead'
		print '#--------|---------|---------|--------|---------|---------|---------'
		print ' %8d %9d %9d %8d %9d %9d %5.2f%%' % (grandTotals[0], grandTotals[1], grandTotals[2], grandTotals[3], grandTotals[4], total, float((total-grandTotals[4])*100) / float(total))
	
		

main()

