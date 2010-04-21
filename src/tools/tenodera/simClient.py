# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import socket, sys, thread, struct
import net_model

sys.path.append('modules')
import events
sys.path.pop()

GUI_PORT = 1522
SIM_PORT = 1521
HOST     = '' # empty string means localhost 
MAX_EVENT_SIZE = 256

class generic:
    "Empty class for storing event fields."
    # TODO: can we used object() instead?
    pass


class simClient:
    "Simulation client that keeps the visualizer's local network model in sync with the server."
    
    def __init__(self, model):
        "Launch the client and link it to a network model."
        self.model = model
        
        model.Bind(net_model.LAUNCH_NODE, self.OnNodeLaunch)
        model.Bind(net_model.MOVE_NODE, self.OnNodeMove)
        model.Bind(net_model.REMOVE_NODE, self.OnNodeRemove)
        model.Bind(net_model.NET_CHANGED, self.OnNetChanged)
        
        args = model,
        thread.start_new_thread(net_event_handler, args)
        self.net_event_subscribe(events.NEW_NODE_EVENT)
        self.net_event_subscribe(events.LED_EVENT)
        self.net_event_subscribe(events.RADIO_MODEL_LINK)
        self.net_event_subscribe(events.RADIO_MODEL_FORWARD)

    def OnNodeLaunch(self, arg, id, binary, x, y):
        "Launch an XMOS process from the simulator when a node is added to the network model."
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
       
        packstr = "!5i" + str(len(binary)) + "s"
        data_size = 12 + len(binary)
        pkt = struct.pack(packstr, data_size, events.LAUNCH_NODE_EVENT, id, x, y, str(binary))
        s.sendto(pkt, (HOST, SIM_PORT))

    def OnNodeMove(self, arg, node):
        "Notify the simulator that a node has been moved."
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
       
        pkt = struct.pack("!iiiii", 12, events.MOVE_NODE_EVENT, node.id, node.pos[0], node.pos[1])
        s.sendto(pkt, (HOST, SIM_PORT))

    def OnNodeRemove(self, arg, node):
        "Notify the simulator that a node has been deleted."
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        pkt = struct.pack("!iii", 4, events.REMOVE_NODE_EVENT, node.id)
        s.sendto(pkt, (HOST, SIM_PORT))
        
    def OnNetChanged(self, arg, node):
        "Notify the simulator that the network model has changed drastically."
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        pkt = struct.pack("!ii", 0, events.REMOVE_ALL_EVENT)
        s.sendto(pkt, (HOST, SIM_PORT))
        
        for node in self.model.IterNodes():
            packstr = "!5i" + str(len(node.binary)) + "s"
            data_size = 12 + len(node.binary)
            pkt = struct.pack(packstr, data_size, events.LAUNCH_NODE_EVENT, node.id, node.pos[0], node.pos[1], node.binary)
            s.sendto(pkt, (HOST, SIM_PORT))
        
    def net_event_subscribe(self, event_id):
        "Tell the simulator that this client is interested in an event type."
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
       
        pkt = struct.pack("!iiii", 8, events.NET_SUBSCRIBE_EVENT, GUI_PORT, event_id)
        s.sendto(pkt, (HOST, SIM_PORT))


def net_event_handler(model):
    "Listen to the simulator and forward interesting events to the network model."
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('',GUI_PORT))

    while True:
        buf, client = sock.recvfrom(MAX_EVENT_SIZE)
        
        hdr = struct.unpack("!2i", buf[:8])
        buf = buf[8:]
        event = generic()
        event.size = hdr[0]
        event.id = hdr[1]
        
        pstr = str(event.size) + "s"
        data = struct.unpack(pstr, buf)
        event.data = data[0]

        if event.id == events.LED_EVENT:
            id, state = struct.unpack("!ii", event.data)
            node = model.GetNode(id)
            if node:
                node.SetLedState(state)
                
        elif event.id == events.RADIO_MODEL_LINK:
            n1, n2, linked = struct.unpack("!iii", event.data)
            if linked:
                model.AddLink(n1, n2)
            else:
                model.DeleteLink(n1, n2)
                
        elif event.id == events.RADIO_MODEL_FORWARD:
            src, ndst = struct.unpack("!ii", event.data[:8])
            dst = struct.unpack('!%ii'%ndst, event.data[8:])
            model.ForwardPacket(src, dst)
                
        elif event.id == events.NEW_NODE_EVENT:
            id, x, y = struct.unpack('!iii', event.data[:12])
            binary = event.data[12:]
            model.AddNode(id, x, y, binary)
            
