# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import cPickle

# Event types for registering callbacks to the simulation client and the GUI.
# These are not the same as XMOS simulator events!
ADD_NODE = 1
MOVE_NODE = 2
REMOVE_NODE = 3
ADD_LINK = 4
REMOVE_LINK = 5
NET_CHANGED = 6
LED_CHANGED = 7
FORWARD_PACKET = 8
LAUNCH_NODE = 9

class net_model:
    "Stores the local network state."
    
    def __init__(self):
        self.node_dict = {}           # Maps node id's to nodes
        self.selected = None          # TODO not used
        # First node id is 20000
        self.next_id = int(20000)     # ID of next new node
        self.link_list = []           # Radio links
        self.filename = None          # Filename this model was loaded from
        self.callbacks = {}           # Event callback functions
        
    def __getstate__(self):
        """Return a tuple of the state we want to pickle.  Don't pickle the callbacks."""
        # These fields are the parts of the model we want to persist.
        return (self.node_dict, self.selected, self.next_id)
        
    def __setstate__(self, state):
        """Restore our pickled state."""
        self.node_dict = state[0]
        self.selected = state[1]
        self.next_id = state[2]
        self.link_list = []
        self.filename = None
        self.callbacks = {}
        
    def LaunchNode(self, binary, x, y):
        id = self.next_id
        self.next_id += 1
        self.SendEvent(LAUNCH_NODE, id, binary, x, y)
    
    def AddNode(self, id, x, y, binary):
        if id >= self.next_id:
            self.next_id = id + 1
        node = node_model(id, x, y, binary)
        self.node_dict[id] = node
        self.SendEvent(ADD_NODE, node)
    
    def MoveNode(self, id, x, y):
        if self.node_dict.has_key(id):
            node = self.node_dict[id]
            node.pos = [x, y]
            self.SendEvent(MOVE_NODE, node)
    
    def DeleteNode(self, id):
        if self.node_dict.has_key(id):
            node = self.node_dict[id]
            del self.node_dict[id]
            self.SendEvent(REMOVE_NODE, node)
    
    def AddLink(self, src, dst):
        if self.node_dict.has_key(src) and self.node_dict.has_key(dst):
            n1 = self.node_dict[src]
            n2 = self.node_dict[dst]
            link = link_model(n1, n2)
            n1.outgoing[n2.id] = link
            n2.incoming[n1.id] = link
            self.link_list.append(link)
            self.SendEvent(ADD_LINK, link)
    
    def DeleteLink(self, src, dst):
        if self.node_dict.has_key(src) and self.node_dict.has_key(dst):
            n1 = self.node_dict[src]
            n2 = self.node_dict[dst]
            if n1.outgoing.has_key(n2.id):
                link = n1.outgoing[n2.id]
                del n1.outgoing[n2.id]
                if n1.id in n2.incoming:
                    del n2.incoming[n1.id]
                self.link_list.remove(link)
                self.SendEvent(REMOVE_LINK, link)
                
    def ForwardPacket(self, src, dst_tup):
        if self.node_dict.has_key(src):
            n1 = self.node_dict[src]
            for dst in dst_tup:
                if dst in self.node_dict:
                    if dst not in n1.outgoing:
                        # Radio model might send out link events before we even see a new node event,
                        # which would have prevented us from adding a link to a unknown node.
                        # Links get a second chance here.  A better solution would be to put events on a queue
                        # in the server to ensure they are delivered in a more meaningful order.
                        n2 = self.node_dict[dst]
                        link = link_model(n1, n2)
                        n1.outgoing[dst] = link
                        n2.incoming[src] = link
                        self.link_list.append(link)
                        self.SendEvent(ADD_LINK, link)
                    link = n1.outgoing[dst]
                    self.SendEvent(FORWARD_PACKET, link)
    
    def ClearNetwork(self):
        self.node_dict.clear()
        self.link_list = []
        self.selected = None
        self.next_id = int(20000)
        self.filename = None
        self.SendEvent(NET_CHANGED, self)
    
    def LoadNetwork(self, filename):
        self.filename = filename
        file = open(filename, 'r')
        newnet = cPickle.load(file)
        file.close()
        
        self.node_dict = newnet.node_dict
        self.link_list = []
        self.selected = newnet.selected
        self.next_id = newnet.next_id
        self.SendEvent(NET_CHANGED, self)
    
    def SaveNetwork(self, filename):
        self.filename = filename
        file = open(filename, 'w')
        cPickle.dump(self, file)
        file.close()
        
    def GetFileName(self):
        return self.filename
        
    def SetFileName(self, name):
        self.filename = name
        
    def IterNodes(self):
        return self.node_dict.itervalues()
        
    def IterLinks(self):
        return iter(self.link_list)
        
    def GetNode(self, id):
        if self.node_dict.has_key(id):
            return self.node_dict[id]
        return None
    
    def Bind(self, id, callback, arg=None):
        if not self.callbacks.has_key(id):
            self.callbacks[id] = []
        self.callbacks[id].append((callback, arg))
    
    def SendEvent(self, id, *args):
        if self.callbacks.has_key(id):
            for callback, arg in self.callbacks[id]:
                callback(arg, *args)


# TODO extend this class for different node types?
class node_model:
    
    def __init__(self, id, x, y, binary):
        # It seems the file dialog returns Unicode strings that struct.pack() can't handle.
        # Convert the binary file name right here.
        self.binary = str(binary)
        self.id = id
        self.pos = (x, y)
        
        self.led = 0
        self.incoming = {}
        self.outgoing = {}        
        self.callbacks = {}
    
    def __getstate__(self):
        """Return a tuple of the state we want to pickle.  Don't pickle the callbacks or radio links."""
        # These fields are the parts of the model we want to persist.
        # links depend on the radio model and therefore are not part of the persistent state
        return (self.binary, self.id, self.pos)
        
    def __setstate__(self, state):
        """Restore our pickled state."""
        self.binary = state[0]
        self.id = state[1]
        self.pos = state[2]
        self.led = 0
        self.incoming = {}
        self.outgoing = {}        
        self.callbacks = {}
    
    def SetLedState(self, state):
        self.led = state
        self.SendEvent(LED_CHANGED, self)
    
    def GetLedState(self):
        return self.led
        
    def GetPosition(self):
        return self.pos
        
    def SetPosition(self, x, y):
        self.pos = (x, y)
        # TODO callback?
    
    def Bind(self, id, callback, arg=None):
        if not self.callbacks.has_key(id):
            self.callbacks[id] = []
        self.callbacks[id].append((callback, arg))
    
    def SendEvent(self, id, *args):
        if self.callbacks.has_key(id):
            for callback, arg in self.callbacks[id]:
                    callback(arg, *args)
    
    def __eq__(self, other):
        return isinstance(other, node_model) and self.id == other.id
        
    def __hash__(self):
        return self.id


# For more flexibility in modeling, assume links are unidirectional.
# For a bidirectional link use two link_model instances.
class link_model:
    
    def __init__(self, src, dst):
        self.src = src
        self.dst = dst
        
    def __eq__(self, other):
        return isinstance(other, link_model) and \
            self.src==other.src and self.dst==other.dst
        
    def __hash__(self):
        # For better efficiency, make sure hashes of bidirectional links don't collide
        dhash = hash(self.dst)
        return hash(self.src) ^ (dhash << 8) ^ (dhash >> 24)
        
