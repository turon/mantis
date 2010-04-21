#!/usr/bin/python

import socket, signal, sys, atexit, os, threading, thread, termios, struct, time

import xml.sax.handler
import getprops
import makepage
import netevents
import mst_handler

class client:

    def handle_event(self, data, eventid, tv):

        # unpack the node id
        tmp = struct.unpack("!i", data[0:4])
        node = tmp[0]

        # index past node id
        data = data[4:len(data)]

        # check for existing node information
        if nodelist.has_key(node):
            # check for packet by this id
            nodeidlist = nodelist[node]

            if nodeidlist.has_key(str(eventid)):
                nodedict = nodeidlist[str(eventid)]
            else:
                nodedict = {}
        else:
            nodeidlist = {}
            nodedict = {}

        # check for type in list
        if not events.eventIsRegistered(eventid):
            print 'unknown type'
            return


        # retrieve list of names and pack string
        pkstr = events.getPackString(eventid)
        names = events.getNames(eventid)

        # unpack the sent data
        data = struct.unpack("!" + pkstr, data)

        new_node_dict = {}
        # store the values in a dictionary
        for i in range(len(names)):
            new_node_dict[names[i]] = data[i]

        if eventid == 9:
            mst_handler.mst_handler(new_node_dict, nodedict)

        # store the time val in the dictionary
        nodedict['tv'] = tv;

        # store the values in a dictionary
        for i in range(len(names)):
            nodedict[names[i]] = data[i]

        # associate the dictionary with the node
        nodeidlist[str(eventid)] = nodedict
        nodelist[node] = nodeidlist;

        mst_handler.mst_tally(nodelist)

    # write the webpage
    def show_data(self):
        mypage = makepage.MakePage()
        mypage.WritePage(nodelist, events)

    # listen for incoming data on connected socket
    def client_thread (self):
        global nodelist
        nodelist = {}
        while 1:

            data, type, tv = nethandler.listen()

            # handle the retrieved data
            self.handle_event(data, type, tv)

            # display the data
            self.show_data()

    def __init__ (self):
        global events
        events = getprops.EventListHandler()
        events.readTypes()

        global nethandler
        nethandler = netevents.netevents()


        print ("---MOS Webtable Client---")
        print ("Waiting for data.")
        self.client_thread()


myclient = client()

