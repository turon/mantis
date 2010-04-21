#!/usr/bin/python
# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import os, sys, re, socket, struct, threading, time, select, string

sys.path.append('modules')
import events

LISTENING_PORT = 1521
MAX_EVENT_SIZE = 256

def load_modules(server, args):
    "Load all the server modules"
    files = os.listdir('modules')
    
    # Use a regular expression to get all python files in the modules directory
    pyfile_reg = re.compile( r'(?P<modname>.*)\.py$' )
    for pyfile in files:
        match = pyfile_reg.match(pyfile)
        if (match):
            if match.group('modname') != 'events':
                try:
                    # Import each module and initialize it.
                    sys.stdout.write('Loading %s --  ' % (match.group('modname')))
                    
                    mod = __import__(match.group('modname'))
                    mod.init_module(server, args)
                except Exception, ie:
                    print 'Error importing module %s: %s' % (match.group( 'modname' ), ie)

class generic:
    "Empty class for holding event fields."
    # TODO can we use object() instead?
    pass
        
class sim_server:
    "The simulation server that does networking and event stuff."

    def __init__(self):
        "Setup the socket server"

        self.handler_table = {}
        self.node_dict = {}
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('',LISTENING_PORT))
        self.ssock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def start(self):
        "Listen for events on a datagram socket."
        while 1:
            buf, client = self.sock.recvfrom(MAX_EVENT_SIZE)

            hdr = struct.unpack("!2i", buf[:8])
            buf = buf[8:]
            event = generic()
            event.size = hdr[0]
            event.id = hdr[1]

            pstr = str(event.size) + "s"
            data = struct.unpack(pstr, buf)
            event.data = data[0]
            
            #print "Event:", event.id, "client:", client
            #print string.join(['%02x' % ord(c) for c in event.data])
            
            self.handle_event(event)
        
    def register_event_handler(self, event_id, handler):
        "Register a function to handle a type of event."
        if self.handler_table.has_key(event_id):
            list = self.handler_table[event_id]
            list.append(handler)
        else:
            list = [handler]
            self.handler_table[event_id] = list

    def handle_event(self, event):
        "Forward an event to all interested handlers."
        if self.handler_table.has_key(event.id):
            list = self.handler_table[event.id]
            for handler in list:
                handler(event)

    def send_event(self, event, recv_list):
        "Send an event datagram to the list of clients."
        addr = '' # localhost
        pstr = "!2i" + str(event.size) + "s"
        pkt = struct.pack(pstr, event.size, event.id, event.data)
        for nid in recv_list:
            port = nid
            self.ssock.sendto(pkt, 0, (addr, port))


# "Main entry point", start the server
server = sim_server()
load_modules(server, sys.argv[1:])
server.start()

