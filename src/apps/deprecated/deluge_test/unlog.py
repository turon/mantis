#!/usr/bin/python

import sys, cPickle, struct

# structure of data:
#
# stuff.startTime
# stuff.finishTime
# stuff.nodes[id]
# stuff.nodes[id].stats[id]
# stuff.nodes[id].stats[id].summary
# stuff.nodes[id].stats[id].profile
# stuff.nodes[id].stats[id].request
# stuff.nodes[id].stats[id].data
# stuff.nodes[id].stats[id].total
# stuff.nodes[id].hits
# stuff.nodes[id].misses
# stuff.nodes[id].hitForward
# stuff.nodes[id].hitPastForward
# stuff.neighbors[id][]

lineno = 0

class empty:
	pass

def readEntry(node, line):
	id = int(line[0:5])
	if line[0:5] != '   30':
		node.stats[id] = empty()
		node.stats[id].summary = int(line[6:13])
		node.stats[id].profile = int(line[14:21])
		node.stats[id].request = int(line[22:29])
		node.stats[id].data = int(line[30:34])
		node.stats[id].total = int(line[35:40])
	else:
		node.hits = int(line[6:13])
		node.misses = int(line[14:21])
		node.hitForward = int(line[22:29])
		node.hitPastForward = int(line[30:34])

def readStats(node, stream):
	global lineno
	stream.readline()	# discard header
	lineno += 1
	stream.readline()	# discard border
	lineno += 1
	
	line = stream.readline()
	lineno += 1
	# loop until we see the bottom border
	while line[0:5] != '-----':
		readEntry(node, line)
		line = stream.readline()
		lineno += 1
	
	stream.readline()	# discard totals
	lineno += 1

def readNeighborEntry(record, line):
	entry = (int(line[0:5]), int(line[6:9]))
	record.append(entry)

def readNeighbors(record, stream):
	global lineno
	stream.readline()	# discard header
	lineno += 1
	stream.readline()	# discard border
	lineno += 1
	
	line = stream.readline()
	lineno += 1
	# loop until we see the bottom border
	while line[0:5] != '-----':
		readNeighborEntry(record, line)
		line = stream.readline()
		lineno += 1

def readStream(stuff, stream):
	global lineno
	line = stream.readline()
	lineno += 1
	while line:
		if line[0:6] == 'Stats ':
			id = int(line[6:])
			if id not in stuff.nodes:
				stuff.nodes[id] = empty()
			stuff.nodes[id].stats = {}
			readStats(stuff.nodes[id], stream)
		elif line[0:9] == 'Finished ':
			id = int(line[9:])
			line = stream.readline()
			lineno += 1
			stuff.finishTime = long(line[6:16])
		elif line[0:20] == 'Version acknowledge ':
			id = int(line[20:])
			line = stream.readline()
			lineno += 1
			stuff.startTime = long(line[6:16])
		elif line[0:10] == 'Neighbors ':
			id = int(line[10:])
			stuff.neighbors[id] = []
			readNeighbors(stuff.neighbors[id], stream)
		line = stream.readline()
		lineno += 1

def dump(stuff):
	print 'Start Time', stuff.startTime
	print 'Finish Time', stuff.finishTime
	for id, node in stuff.nodes.iteritems():
		print 'Stats', id
		print ' Node Summary Profile Request Data Total'
		print '----- ------- ------- ------- ---- -----'
		for nid, nd in node.stats.iteritems():
			print '%5d %7d %7d %7d %4d %5d' % (nid,nd.summary,nd.profile,nd.request,nd.data,nd.total)
		print '----- ------- ------- ------- ---- -----'
		print 'Cache', id
		print 'Hits:', node.hits, 'Misses:', node.misses, 'Hit Forward State:', node.hitForward, 'Hit Past Forward State:', node.hitPastForward
		
	for id, record in stuff.neighbors.iteritems():
		print 'Neighbors', id
		print ' Node DTB'
		print '----- ---'
		for entry in record:
			print '%5d %3d' % entry
		print '----- ---'
		print
	
def dumpBinary(stuff, stream):
	stream.write(struct.pack('!qqi', stuff.startTime, stuff.finishTime, len(stuff.nodes)))
        for id, node in stuff.nodes.iteritems():
		stream.write(struct.pack('!ii', id, len(node.stats)))
                for nid, nd in node.stats.iteritems():
			stream.write(struct.pack('!iiiii', nid, nd.summary, nd.profile, nd.request, nd.data))
	stream.write(struct.pack('!i', len(stuff.neighbors)))
        for id, record in stuff.neighbors.iteritems():
		stream.write(struct.pack('!ii', id, len(record)))
                for entry in record:
			stream.write(struct.pack('!ii', entry[0], entry[1]))

def main():
	stuff = empty()
	stuff.nodes = {}
	stuff.neighbors = {}

	fname = sys.argv[1]
	stream = file(sys.argv[1])

	pname = fname[:-4] + '.pdat'
	out = file(pname, 'w')
	
	readStream(stuff, stream)
	stream.close()
	#dump(stuff)
	cPickle.dump(stuff, out)
	out.flush()
	out.close()
	
	jname = fname[:-4] + '.jdat'
	out = file(jname, 'w')
	dumpBinary(stuff, out)
	out.flush()
	out.close()

try:
	main()
except:
	print "Error occurred on line %d of input" % lineno
	raise


