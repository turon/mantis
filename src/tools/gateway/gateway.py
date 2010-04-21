#!/usr/bin/python

import socket, signal, sys, atexit, os, threading, thread, termios, struct, time
import random

import xml.sax.handler
import getprops

MOS_GATEWAY_PORT = 4200
SERIAL_DEVICE = "/dev/ttyS0"
#SERIAL_SPEED = termios.B38400
SERIAL_SPEED = termios.B57600
# preamble byte from com.h
PREAMBLE = 0x53
# preamble size from com.h
PREAMBLE_SIZE = 2
use_fake_data = 0

def graceful_quit (one, two):
    for addr,soc in connected_sockets.iteritems ():
        #socket.shutdown (2)
        soc.close () 

    #server_sock.shutdown (2)
    server_sock.close ()
    print ("Exiting..")
    sys.exit (0)

def serial_thread (one,two):
    print ("In serial thread, using device " + SERIAL_DEVICE)

    try:
        serial_file = file (SERIAL_DEVICE, "r")
    except:
        print ("Problem open serial file")

    fd = serial_file.fileno ()
    old = termios.tcgetattr (fd)
    new = termios.tcgetattr (fd)

    new[0] = new[1] = new[2] = new[3] = new[4] = new[5] = 0
    for item in new[6]:
        item = 0
    
    # iflag
    new[0] = new[0] & ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                       termios.ISTRIP | termios.INLCR  | termios.IGNCR  |
                       termios.ICRNL  | termios.IXON   | termios.INPCK  |
                       termios.IXOFF  | termios.IXON)
    # oflag
    new[1] = new[1] & ~(termios.OPOST)
    # cflag
    new[2] = new[2] & ~(termios.CSIZE  | termios.PARENB)
    new[2] = new[2] | (termios.CLOCAL  | termios.CREAD | termios.CS8)
    new[2] = new[2] & ~(termios.HUPCL | termios.CSTOPB | termios.CRTSCTS)
    # lflag
    new[3] = 0

    # cc
    new[3] = 0
    new[6][termios.VTIME] = 0
    new[6][termios.VMIN] = 1
    # ispeed
    new[4] = SERIAL_SPEED
    # ospeed
    new[5] = SERIAL_SPEED

    termios.tcflush (fd, termios.TCIFLUSH)
    termios.tcsetattr (fd, termios.TCSANOW, new)

    preamble_count = 0

    while (1):
        ret = serial_file.read (1)
        ret = struct.unpack ("B", ret)
        if ret[0] == PREAMBLE:
            preamble_count += 1
            if preamble_count == PREAMBLE_SIZE:
                ret = serial_file.read (1)
                ret = struct.unpack ("B", ret)
                data = serial_file.read (ret[0])
                handle_event (data)          
                preamble_count = 0
            else:
                continue
        else:
            print ("Got an unexpected byte: " + str (ret[0]) + ", searching for " +
                   "another preamble byte")
            preamble_count = 0
    
    return

def handle_event (data):
    try:
        # unpack the header of FROM, TO, TYPE from the node
        event_header = struct.unpack("HHH", data[0:6])
        data = data[6:len(data)]

        node = event_header[0]
        id = event_header[2]


        # check for event in event list
        if not events.eventIsRegistered(id):
            print "Unknown event type: ", str(id)
            return

        # retrieve the list of names
        eventnames = events.getNames(id)
        packstr = events.getPackString(id)

        # unpack the data
        try:
            data = struct.unpack (packstr, data)
        except:
            print "Problem unpacking data."
            return

        # display the data
        print "Node: ", str(node),

        i=0
        for value_name in eventnames:
            print value_name + ":" + str(data[i]),
            i=i+1

        print ""

        # start re-packing data in network byte order, appending
        # the timeval 
        size = struct.calcsize(packstr)
        tv = time.time ()
        tv = int (tv)

        finaldata = struct.pack("!iili", size, id, tv, node)


        i = 0
        # repack the payload in network byte order
        for name in eventnames:
            eventtype = events.getDataType(id, name)
            newdata = struct.pack("!" + eventtype, data[i])
            finaldata = finaldata + newdata
            i = i + 1
            

    except:
        print ("Invalid event packet.");
        return
    
    send_to_clients (finaldata)


def send_to_clients (data):
   
    list_lock.acquire ()
    for addr,soc in connected_sockets.iteritems ():
        soc.send (data)
    list_lock.release ()

def to_node_byte_order (data):
    tmp_pk_string = struct.pack("!H", data)
    newdata = struct.unpack("H", tmp_pk_string)
    return newdata[0]

def make_fake_data(node, id):
    eventnames = events.getNames(id)
    packstr = events.getPackString(id)

    size = struct.calcsize(packstr)
    tv = time.time ()
    tv = int(tv)

    finaldata = struct.pack("!illi", size, id, tv, node)

    for evname in eventnames:
        type = events.getDataType(id, evname)
        try:
            max = int(str(events.getDataProperty(id, evname, "max")))
            min = int(str(events.getDataProperty(id, evname, "min")))
        except:
            if type == "H":
                max = 65535
                min = 0
            elif type == "B":
                max = 255
                min = 0
            else:
                print "unknown type, can't generate random data"
                max = 100
                min = 0

        if min < max:
            newval = random.randint(min,max)
        else:
            newval = random.randint(max,min)

        if type == "H":
            newval = to_node_byte_order(newval)

        newdata = struct.pack("!" + type, newval)
        finaldata = finaldata + newdata

    return finaldata


def make_fake_mst_data(node, nodedict):
    eventnames = events.getNames(9)
    packstr = events.getPackString(9)

    size = struct.calcsize(packstr)
    tv = time.time ()
    tv = int(tv)

    thisdict = {}

    finaldata = struct.pack("!illi", size, 9, tv, node)

    try:
        lastdict = nodedict[node]
    except:
        lastdict = { "seq":0 }

    for evname in eventnames:
        type = events.getDataType(9, evname)

        newval = 5
        
        if evname == "seq":
            newval = lastdict[evname] + 1
            drop_this_packet = random.randint(0,100);
            if(drop_this_packet < 4):
                newval = newval + 1
            if(drop_this_packet < 20):
                newval = newval + 1
        
        thisdict[evname] = newval
        if type == "H":
            newval = to_node_byte_order(newval)

        newdata = struct.pack("!" + type, newval)
        finaldata = finaldata + newdata

    nodedict[node] = thisdict
    return finaldata                

# send fake sensor data
def fake_thread (one,two):
    nodedict = {}
    while 1:
        nodenum = random.randint(5,20)
        type = random.randint(0,10)
#        if type < 9:
#            data = make_fake_data(nodenum, 8)
#        elif type >= 9:
#            data = make_fake_data(nodenum, 7)
#        data = make_fake_data(nodenum, 9);
        data = make_fake_mst_data(nodenum, nodedict);
        send_to_clients (data)
        time.sleep(.01)

def client_thread (client, fulladdr):
    print ("In client thread for " + fulladdr)
    try:
        serial_file = file (SERIAL_DEVICE, "w")
    except:
        print ("Couldn't open serial device")

    while 1:
        data = client.recv (1024)
        if len (data) == 0:
            print ("Removing client at " + fulladdr)
            list_lock.acquire ()
            client.close ()
            del (connected_sockets[fulladdr])
            connected_clients.remove (fulladdr)
            list_lock.release ()
            thread.exit ()
        else:
            # has a command arg
            if len (data) == 24:
                to_send = struct.unpack ("!iiliii", data)
                print (to_send[3], to_send[4], to_send[5])
                try:
                    to_send = struct.pack ("!BBBHHBBB", PREAMBLE, PREAMBLE, 7, 0,
                                           to_send[3], to_send[4], 0, to_send[5])
                except:
                    print ("Problem packing data1")
            # no command arg
            elif len (data) == 20:
                to_send = struct.unpack ("!iilii", data)
                try:
                    to_send = struct.pack ("!BBBHBBBB", PREAMBLE, PREAMBLE, 6, 0,
                                           to_send[3], 0, to_send[4], 0)
                except:
                    print ("Problem packing data2")
            else:
                print ("Unhandled length " + str (len (data)))
                continue
            serial_file.write (to_send)
            serial_file.flush ()

#signal.signal (signal.SIGKILL, graceful_quit)
#signal.signal (signal.SIGSTOP, graceful_quit)
##signal.signal (signal.SIGQUIT, graceful_quit)
#signal.signal (signal.SIGTERM, graceful_quit)
#signal.signal (signal.SIGINT, graceful_quit)

global connected_clients
connected_clients = []
global connected_sockets
connected_sockets = {}
global connected_threads
connected_threads = {}

atexit.register (graceful_quit, 1, 2)

server_sock = socket.socket (socket.AF_INET, socket.SOCK_STREAM)
server_sock.setsockopt (socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_sock.bind (("", MOS_GATEWAY_PORT))
server_sock.listen (20)

global events
events = getprops.EventListHandler()
events.readTypes()

global list_lock
list_lock = threading.Lock ()
global serial_lock
serial_lock = threading.Lock ()
global serial_send_data
serial_send_data = ""
serial_thread = thread.start_new_thread (serial_thread, (1, 2))

if use_fake_data == 1:
    print("Generating fake data...")
    fake_thread = thread.start_new_thread (fake_thread, (1, 2))

print ("---MOS Gateway---")

while 1:
    client,addr = server_sock.accept ()
    fulladdr = addr[0] + ":" + str(addr[1])
    print ("Got a connection from " + fulladdr)
    list_lock.acquire ()
    old_client = None
    for old_addr in connected_clients:
        if old_addr == fulladdr:
            print ("Already have a connection for " + fulladdr)
            old_client = fulladdr
            break

    if old_client == None:
        connected_clients.append (fulladdr)
        connected_sockets[fulladdr] = client
        connected_threads[fulladdr] = thread.start_new_thread (client_thread,
                                                              (client, fulladdr))
    list_lock.release ()
