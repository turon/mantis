# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import struct
import events

class net_event:
    def __init__(self, server):
        self.server = server
        self.sub_table = {}
        
    def ne_subscribe(self, event):
        list = struct.unpack("!ii", event.data)
        id = list[0]
        event_id = list[1]

        # Add to an event that is already subscribed to
        if self.sub_table.has_key(event_id):
            id_list = self.sub_table[event_id]
            if id_list.count(id) == 0:
                id_list.append(id)

        # Add a new list of interested nodes
        else:
            id_list = []
            id_list.append(id)
            self.sub_table[event_id] = id_list
            
        # Now actually subscribe to the event
        self.server.register_event_handler(event_id, self.ne_event_handler)

    def ne_unsubscribe(self, event):
        list = struct.unpack("!ii", event.data)
        id = list[0]
        event_id = list[1]

        # Pull the id out of the subscription list
        if self.sub_table.has_key(event_id):
            id_list = self.sub_table[event_id]
            if id_list.count(id) != 0:
                del id_list[id_list.index(id)]

    def ne_event_handler(self, event):
        # Get the list of subscribers and forward the event
        if self.sub_table.has_key(event.id):
            id_list = self.sub_table[event.id]
            self.server.send_event(event, id_list)

            #print "Sent net event:", event.id

def init_module(server, args):
    net = net_event(server)

    server.register_event_handler(events.NET_SUBSCRIBE_EVENT, net.ne_subscribe)
    server.register_event_handler(events.NET_UNSUBSCRIBE_EVENT, net.ne_unsubscribe)
    print "Success"
