#!/usr/bin/python

import struct
import time
from time import gmtime, strftime
import getprops
import netevents

class client:
    def handle_event(self, data, eventid, tv):

        # unpack the node id
        tmp = struct.unpack("!i", data[0:4])
        node = tmp[0]
        data = data[4:len(data)]

        # check for type in list
        if not events.eventIsRegistered(eventid):
            print 'unknown type'
            return

        # retrieve list of names and pack string
        pkstr = events.getPackString(eventid)
        names = events.getNames(eventid)

        # unpack the sent data
        data = struct.unpack(pkstr, data)

        #print eventid,
        timestr =  str(time.time()%1)
        timestr = timestr[2:len(timestr)]
        if len(timestr) > 6:
            timestr = timestr[0:6]
        print str(strftime("%H:%M:%S.")) + timestr,
        print node,
        # store the values in a dictionary
        for i in range(len(names)):
            print data[i],

        print " "
        #print tv

    # listen for incoming data on connected socket
    def client_thread (self):
        while 1:
            data, type, tv = nethandler.listen()
            self.handle_event(data, type, tv)

    def __init__ (self):
        global events
        events = getprops.EventListHandler()
        events.readTypes()

        global nethandler
        nethandler = netevents.netevents()


        print ("---MOS Net to stdout Client---")
        self.client_thread()

myclient = client()

