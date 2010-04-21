# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import sys, os, struct, signal

import events

class auto_loader:
    def __init__(self, server):
        self.server = server
        self.ps_dict = {}
        self.all_nodes = []
    
    def al_launch_node(self, event):
        unpackstr = "!3i" + str(event.size - 12) + "s"
        list = struct.unpack(unpackstr, event.data);
        id = list[0]
        x = list[1]
        y = list[2]
        path = list[3]
        
        args = [path, '-viz', '-id', str(id), '-x', str(x), '-y', str(y)]

        ret = os.spawnv(os.P_NOWAIT, path, args)
        self.ps_dict[id] = ret
        
    def al_new_node(self, event):
        id, = struct.unpack('!i', event.data[:4])
        self.all_nodes.append(id)
        
    def al_remove_node(self, event):
        list = struct.unpack("!i", event.data)
        id = list[0]

        #if self.ps_dict.has_key(id):
        #    os.kill(self.ps_dict[id], signal.SIGTERM)
        if id in self.ps_dict: del self.ps_dict[id]
        if id in self.all_nodes: self.all_nodes.remove(id)
        self.kill_nodes([id])

    def kill_nodes(self, nodes):
        #for id, ps in self.ps_dict.iteritems():
        #    os.kill(ps, signal.SIGTERM)

        # NOTE: We might want to keep the simServer up?
        #sys.exit(0)
        class empty:
            pass
        event = empty()
        event.id = events.KILL_NODE_EVENT
        event.size = 0
        event.data = ''
        self.server.send_event(event, nodes)

    def al_kill_all(self, event):
        print "Killing all virtual nodes!"
        #self.kill_nodes(None, None)
        self.kill_nodes(self.all_nodes)
        self.ps_dict = {}
        self.all_nodes = []
        
        
def init_module(server, args):
    loader = auto_loader(server)
    signal.signal(signal.SIGINT, loader.kill_nodes)
    
    server.register_event_handler(events.LAUNCH_NODE_EVENT, loader.al_launch_node)
    server.register_event_handler(events.NEW_NODE_EVENT, loader.al_new_node)
    server.register_event_handler(events.REMOVE_NODE_EVENT, loader.al_remove_node)
    server.register_event_handler(events.REMOVE_ALL_EVENT, loader.al_kill_all)
    print 'Success'
