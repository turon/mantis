# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import struct, math, random, string
import events

RADIO_RANGE = 60

class generic:
    pass

class radio_model:
    def __init__(self, server):
        self.server = server
        self.node_dict = {}     # Store node ids with positions
        self.neighbor_dict = {} # Store neighbor list for each node
        self.biterrorrate = 0
        self.packeterrorrate = 0

    def rm_new_node(self, event):
        list = struct.unpack("!iii", event.data[:12])
        id = list[0]
        x = list[1]
        y = list[2]

        # Compute the neighbor list for this node and update other lists
        neighbors = []
        for nid, list in self.node_dict.iteritems():    
            distance = math.sqrt(pow(abs(list[0] - x),2) + pow(abs(list[1] - y), 2))
            if distance <= RADIO_RANGE:
                neighbors.append(nid)
                self.neighbor_dict[nid].append(id)
                self.rm_send_link_event(id, nid, 1)
            
        # Now throw the node into our dict and add store the neighbor list
        self.node_dict[id] = x, y
        self.neighbor_dict[id] = neighbors
    
        #print 'New node', id, 'at:', x, ',', y, 'with neighbors: ', neighbors

    def rm_move_node(self, event):
        list = struct.unpack("!iii", event.data[:12])
        id = list[0]
        x = list[1]
        y = list[2]

        # Compute the neighbor list for this node and update other lists
        neighbors = self.neighbor_dict[id]
        for nid, list in self.node_dict.iteritems():
            if id == nid:
                continue
            distance = math.sqrt(pow(abs(list[0] - x),2) + pow(abs(list[1] - y), 2))
            if distance <= RADIO_RANGE:
                if nid not in neighbors:
                    neighbors.append(nid)
                if id not in self.neighbor_dict[nid]:
                    self.neighbor_dict[nid].append(id)
                self.rm_send_link_event(id, nid, 1)
                self.rm_send_link_event(nid, id, 1)
            if distance > RADIO_RANGE:
                if nid in neighbors:
                    neighbors.remove(nid)
                if id in self.neighbor_dict[nid]:
                    self.neighbor_dict[nid].remove(id)
                self.rm_send_link_event(id, nid, 0)
                self.rm_send_link_event(nid, id, 0)
            
        # Now throw the node into our dict and add store the neighbor list
        self.node_dict[id] = x, y
        #print 'Move node', id, 'to:', x, ',', y, 'with neighbors: ', neighbors

    def rm_remove_node(self, event):
        list = struct.unpack("!i", event.data)
        id = list[0]
        neighbors = self.neighbor_dict[id]

        # Remove this node from the neighbor lists of its neighbors
        for nid in neighbors:
            nlist = self.neighbor_dict[nid]
            del nlist[nlist.index(id)]
            self.rm_send_link_event(id, nid, 0)
            
        # Now remove the entry from the dicts
        del self.node_dict[id]
        del self.neighbor_dict[id]

    def rm_remove_all(self, event):
        # TODO should we send events for the deleted radio links?
        self.node_dict.clear()
        self.neighbor_dict.clear()

    def rm_forward_pkts(self, event):
        list = struct.unpack("!i", event.data[:4])
        id = list[0]

        if self.neighbor_dict.has_key(id):
            nlist = self.neighbor_dict[id]
            if self.biterrorrate:
                packed_id = event.data[:4]
                bytes = [ord(x) for x in event.data[4:]]
                for nid in nlist:
                    event.data = packed_id + string.join([chr(x) for x in [self.corrupt(y) for y in bytes]],'')
                    self.server.send_event(event, [nid])
            elif self.packeterrorrate:
                for nid in nlist:
                    if random.random() > self.packeterrorrate:
                        self.server.send_event(event, [nid])
            else:
                self.server.send_event(event, nlist)
            
            event = generic()
            event.id = events.RADIO_MODEL_FORWARD
            event.size = 8 + 4*len(nlist)
            event.data = struct.pack('!ii', id, len(nlist))
            event.data += struct.pack('!%ii'%len(nlist), *nlist)
            self.server.handle_event(event)
            
    def rm_send_link_event(self, n1, n2, linked):
        event = generic()
        event.id = events.RADIO_MODEL_LINK
        event.size = 12
        event.data = struct.pack("!iii", n1, n2, linked)
        self.server.handle_event(event)
        
    def corrupt(self, byte):
        mask = sum([(random.random()<self.biterrorrate) << x for x in range(8)])
        return byte ^ mask

        
def init_module(server, args):
    "Setup the event handling for this module"
    model = radio_model(server)
    
    # We can't use getopt because it will blow up on some other guy's option.
    if '--biterrorrate' in args:
        er = args[args.index('--biterrorrate')+1]
        model.biterrorrate = float(er)
        print 'Bit Error Rate: % 2.2f%% --' % (100*model.biterrorrate),
    elif '--packeterrorrate' in args:
        er = args[args.index('--packeterrorrate')+1]
        model.packeterrorrate = float(er)
        print 'Packet Error Rate: % 2.2f%% --' % (100*model.packeterrorrate),
    random.seed()

    server.register_event_handler(events.NEW_NODE_EVENT, model.rm_new_node);
    server.register_event_handler(events.MOVE_NODE_EVENT, model.rm_move_node);
    server.register_event_handler(events.REMOVE_NODE_EVENT, model.rm_remove_node);
    server.register_event_handler(events.REMOVE_ALL_EVENT, model.rm_remove_all)
    server.register_event_handler(events.RADIO_PACKET_EVENT, model.rm_forward_pkts);
    print 'Success'

