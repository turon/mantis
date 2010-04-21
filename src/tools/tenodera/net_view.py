# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import wx, thread
import net_model

class node_view:
    def __init__(self, model, color = 'BLUE'):
        self.node_radius = 10 # Radius of a node
        self.node_color = 'GREEN'    # TODO not currently used
        self.node_outline = 'BLACK'  # TODO not currently used
        # Setting this flag prevents drawing this node and links while dragging
        self.dragging = False        

        self.model = model

        # Now setup the node's bitmap so we can just blit to the screen
        # rather than having to re-draw every time.
        #self.bmp = wx.EmptyBitmap(2 * self.node_radius + 4, 2 * self.node_radius + 4)
        self.bmp = wx.EmptyBitmap(2 * self.node_radius, 3 * self.node_radius)
        
        self.Update()

    def HitTest(self, point):
        rect = self.GetRect()
        return rect.InsideXY(point.x, point.y)

    def GetRect(self):
        x, y = self.model.GetPosition()
        return wx.Rect(x-self.node_radius, y-self.node_radius,
                      self.bmp.GetWidth(), self.bmp.GetHeight())

    def Erase(self, dc):
        if self.dragging:
            return
        dc.SetBrush(wx.Brush("WHITE"))
        dc.SetPen(wx.Pen("WHITE"))
        x, y = self.model.GetPosition()
        #dc.DrawRectangle(x-self.node_radius, y-self.node_radius,
        #                self.node_radius * 2 + 4, self.node_radius * 2 + 4)
        dc.DrawRectangle(x-self.node_radius, y-self.node_radius,
                        2 * self.node_radius, 3 * self.node_radius)


    def Draw(self, dc, op = wx.COPY):
        if self.dragging:
            return True
        if self.bmp.Ok():
            memDC = wx.MemoryDC()
            memDC.SelectObject(self.bmp)

            x, y = self.model.GetPosition()
            dc.Blit(x-self.node_radius, y-self.node_radius,
                    self.bmp.GetWidth(), self.bmp.GetHeight(),
                    memDC, 0, 0, op, True)

            return True
        else:
            return False

    def Update(self):
        #self.led = state
        # create a DC for drawing in to the bitmap memory
        bdc = wx.MemoryDC(); 
        bdc.SelectObject(self.bmp); 

        # First clear the background
        #bdc.SetBrush(wx.Brush("WHITE"))
        #bdc.SetPen(wx.Pen("WHITE"))
        #bdc.DrawRectangle(0, 0, self.node_radius * 2 + 4, self.node_radius * 2 + 4)

        # Now draw our default node
        #bdc.SetBrush(wx.Brush(self.node_color))

        #if self.model.GetLedState() == 1:
        #    bdc.SetPen(wx.Pen(self.node_outline, 4))
        #else:
        #    bdc.SetPen(wx.Pen("RED", 4))

        #bdc.DrawEllipse(0, 0, self.node_radius * 2, self.node_radius * 2)
        bdc.SetBrush(wx.Brush("DARKGREEN"))
        bdc.SetPen(wx.Pen("DARKGREEN"))
        bdc.DrawRectangle(0, 0, 2 * self.node_radius, 3 * self.node_radius)
        
        # Now draw the led line
        if self.model.led & 1:
            bdc.SetBrush(wx.Brush("YELLOW"))
            bdc.SetPen(wx.Pen("YELLOW"))
            bdc.DrawRectangle(0, 16, self.node_radius*3/2, 8)
        if self.model.led & 2:    # green
            bdc.SetBrush(wx.Brush("GREEN"))
            bdc.SetPen(wx.Pen("GREEN"))
            bdc.DrawRectangle(0, 8, self.node_radius*3/2, 8)
        if self.model.led & 4:    # red
            bdc.SetBrush(wx.Brush("RED"))
            bdc.SetPen(wx.Pen("RED"))
            bdc.DrawRectangle(0, 0, self.node_radius*3/2, 8)

        # must disconnect the bitmap from the dc so we can use it later
        bdc.SelectObject(wx.NullBitmap);

        # Create a mask so that we only blit the colored part
        #if "__WXGTK__" not in wx.PlatformInfo:
        #mask = wx.Mask(self.bmp, wx.WHITE)
        mask = wx.Mask(self.bmp)
        mask.colour = wx.WHITE

        self.bmp.SetMask(mask)
            
    def __str__(self):
        return 'node_view:'+str(self.model.id)
            
            
class link_view:
   
    def __init__(self, src, dst):
        self.src = src
        self.dst = dst
        self.flashcount = 0

    def Erase(self, dc):
        if self.src.dragging or self.dst.dragging:
            return
        pen = wx.Pen("WHITE")
        pen.SetWidth(4)
        dc.SetPen(pen)
        dc.DrawLine(self.src.model.pos[0], self.src.model.pos[1], self.dst.model.pos[0], self.dst.model.pos[1])
        
    def Draw(self, dc, op = wx.COPY):
        if self.src.dragging or self.dst.dragging:
            return
        if self.flashcount:
            pen = wx.Pen("GOLD")
        else:
            pen = wx.Pen("BLUE")
        pen.SetWidth(4)
        dc.SetPen(pen)
        dc.DrawLine(self.src.model.pos[0], self.src.model.pos[1], self.dst.model.pos[0], self.dst.model.pos[1])


class event_queue:
    "Queue for storing net events and their callbacks.  See net_view.DispatchEvent()."
    
    def __init__(self):
        self.lock = thread.allocate_lock()
        self.list = []
        
    def put(self, obj):
        "Add an object to the queue atomically."
        self.lock.acquire()
        self.list.append(obj)
        self.lock.release()
        
    def get(self):
        "Return the entire queue as a list and clear the queue atomically."
        self.lock.acquire()
        list = self.list
        self.list = []
        self.lock.release()
        return list
        

class net_view(wx.ScrolledWindow):
    "This component does the drawing of the network model."
    
    def __init__(self, parent, id, model):
        wx.ScrolledWindow.__init__(self, parent, id, style=wx.NO_FULL_REPAINT_ON_RESIZE)
        self.model = model
        self.node_dict = {}
        self.link_dict = {}
        self.node_size = 25
        self.dragNode = None
        self.dragImage = None
        self.queue = event_queue()
    
        self.SetBackgroundColour("WHITE")
        self.SetCursor(wx.StockCursor(wx.CURSOR_ARROW))

        # Mouse buttons and motion
        wx.EVT_LEFT_DOWN(self, self.OnLeftDown)
        wx.EVT_LEFT_UP(self, self.OnLeftUp)
        wx.EVT_MOTION(self, self.OnMotion)

        wx.EVT_PAINT(self, self.OnPaint)
        wx.EVT_IDLE(self, self.OnIdle)
        
        self.SetMode("Select")
        
        # Register network events callback DispatchEvent.
        # See net_view.DispatchEvent() for details.
        model.Bind(net_model.ADD_NODE, self.DispatchEvent, self.add_node)
        model.Bind(net_model.REMOVE_NODE, self.DispatchEvent, self.del_node)
        model.Bind(net_model.ADD_LINK, self.DispatchEvent, self.add_radio_link)
        model.Bind(net_model.REMOVE_LINK, self.DispatchEvent, self.del_radio_link)
        model.Bind(net_model.NET_CHANGED, self.DispatchEvent, self.new_network)
        model.Bind(net_model.FORWARD_PACKET, self.DispatchEvent, self.forward_radio_packet)
        
    def DispatchEvent(self, callback, *args):
        """"Queue a net event to be handled on the GUI thread.
        
        Many wxPython functions do not work when invoked from a thread other
        than the main GUI thread.  This is a problem for network events, because
        they occur during the listen thread that was spawned by simClient.py.
        The solution is to register a meta-callback, this method, with the
        network model.  When DispatchEvent is invoked by the network model,
        it puts the original GUI callback, along with the arguments, 
        on self.queue and then calls wx.WakeUpIdle().  This causes OnIdle to be
        invoked on the main GUI thread, which in turn invokes every callback
        that is on the queue, and these callbacks can invoke wxPython functions
        without fear of being on the wrong thread.  This greatly simplifies the
        implementation of the callbacks (trust me)."""
        self.queue.put((callback, args))
        # Cause an idle event to occur, which will invoke our idle handler.
        wx.WakeUpIdle()

    def FindNode(self, point):
        "Return the node that contains the point."
        for n in self.node_dict.itervalues():
            if n.HitTest(point):
                return n
        return None

    def OnLeftDown(self, evt):
        node = self.FindNode(evt.GetPosition())
        if node:
            self.dragNode = node
            self.dragStartPos = evt.GetPosition()

    def OnLeftUp(self, evt):
        if not self.dragImage or not self.dragNode:
            self.dragImage = None
            self.dragNode = None
            return

        # Hide the image, end dragging, and nuke out the drag image.
        self.dragImage.Hide()
        self.dragImage.EndDrag()
        self.dragImage = None

        dc = wx.ClientDC(self)
        # reposition and draw the shape
        self.dragNode.model.pos = (
            self.dragNode.model.pos[0] + evt.GetPosition()[0] - self.dragStartPos[0],
            self.dragNode.model.pos[1] + evt.GetPosition()[1] - self.dragStartPos[1]
            )
            
        self.dragNode.dragging = False
        self.dragNode.Draw(dc)
        # Update the network model.
        self.model.MoveNode(self.dragNode.model.id, self.dragNode.model.pos[0], self.dragNode.model.pos[1])
        self.dragNode = None

    def OnRightDown(self, event):
        pass

    def OnRightUp(self, event):
        pass

    def OnMotion(self, evt):
        # Ignore mouse movement if we're not dragging.
        if not self.dragNode or not evt.Dragging() or not evt.LeftIsDown():
            return

        # if we have a node, but haven't started dragging yet
        if self.dragNode and not self.dragImage:
            # only start the drag after having moved a couple pixels
            tolerance = 2
            pt = evt.GetPosition()
            dx = abs(pt.x - self.dragStartPos.x)
            dy = abs(pt.y - self.dragStartPos.y)
            if dx <= tolerance and dy <= tolerance:
                return

            # Create a DragImage to draw this node while it is moving
            # (The drag image will update even as the bitmap is updating.  Magical!) 
            self.dragImage = wx.DragImage(self.dragNode.bmp,
                wx.StockCursor(wx.CURSOR_HAND))
            hotspot = self.dragStartPos - self.dragNode.model.pos + [self.dragNode.node_radius, self.dragNode.node_radius]
            self.dragImage.BeginDrag(hotspot, self, False)
            self.dragImage.Move(pt)

            # erase the node since it will be drawn by the DragImage now
            dc = wx.ClientDC(self)   
            for link in self.dragNode.model.incoming.itervalues():
                if link not in self.link_dict: continue
                l = self.link_dict[link]
                l.Erase(dc)
                l.src.Draw(dc)
            for link in self.dragNode.model.outgoing.itervalues():
                if link not in self.link_dict: continue
                l = self.link_dict[link]
                l.Erase(dc)
                l.dst.Draw(dc)
            self.dragNode.Erase(dc)
            self.dragNode.dragging = True
            self.dragImage.Show()

        # if we have node and image then move it
        elif self.dragNode and self.dragImage:
            self.dragImage.Move(evt.GetPosition())

    def OnSize(self, event):
        pass
        
    def OnIdle(self, event):
        """Handle queued network events.  See net_view.DispatchEvent()."""
        for callback, args in self.queue.get():
            callback(*args)

    def OnPaint(self, event):
        """ Window expose events come here to refresh. """
        dc = wx.PaintDC(self)
        self.Draw(dc)
        
    def Draw(self, dc):
        dc.BeginDrawing()    # for Windows compatibility
        # Since we are a scrolling window we need to prepare the DC
        self.PrepareDC(dc)

        dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
        dc.Clear()

        for link in self.link_dict.itervalues():
            link.Draw(dc)
        for node in self.node_dict.itervalues():
            node.Draw(dc)
        dc.EndDrawing()        

    def SetMode(self, mode):
        self.mode = mode

        if self.mode == "Select":
            self.SetCursor(wx.StockCursor(wx.CURSOR_ARROW))
        else:
            self.SetCursor(wx.StockCursor(wx.STANDARD_CURSOR))

    # TODO do something about this color parm
    def add_node(self, nodemodel, color = 'BLUE'):
        n = node_view(nodemodel, color)
        self.node_dict[nodemodel] = n
        nodemodel.Bind(net_model.LED_CHANGED, self.DispatchEvent, self.node_state_changed)
        n.Update()
        
        dc = wx.ClientDC(self)
        n.Draw(dc)
    
    def del_node(self, node):
        if self.node_dict.has_key(node):
            dc = wx.ClientDC(self)
            self.node_dict[node].Erase(dc)
            del self.node_dict[node]
       
    def node_state_changed(self, node):
        if self.node_dict.has_key(node):
            n = self.node_dict[node]
            n.Update()

            dc = wx.ClientDC(self)
            n.Draw(dc)

    def add_radio_link(self, link):
        if self.node_dict.has_key(link.src) and self.node_dict.has_key(link.dst):
            src = self.node_dict[link.src]
            dst = self.node_dict[link.dst]
            l = link_view(src, dst)
            self.link_dict[link] = l
            
            dc = wx.ClientDC(self)
            l.Draw(dc)
            l.src.Draw(dc)
            l.dst.Draw(dc)

    def del_radio_link(self, link):
        if self.link_dict.has_key(link):
            l = self.link_dict[link]
            dc = wx.ClientDC(self)
            l.Erase(dc)
            l.src.Draw(dc)
            l.dst.Draw(dc)
            del self.link_dict[link]
        
    def new_network(self, model):
        self.node_dict.clear()
        self.link_dict.clear()
        self.dragNode = None
        self.dragImage = None
        dummy = self.queue.get()    # empties the list
        
        for nodemodel in model.IterNodes():
            n = node_view(nodemodel, 'BLUE')
            self.node_dict[nodemodel] = n
            nodemodel.Bind(net_model.LED_CHANGED, self.DispatchEvent, self.node_state_changed)
            n.Update()
        for link in model.IterLinks():
            l = link_view(self.node_dict[link.src], self.node_dict[link.dst])
            self.link_dict[link] = l
            
        dc = wx.ClientDC(self)
        self.Draw(dc)
        
    def forward_radio_packet(self, link):
        if link in self.link_dict:
            l = self.link_dict[link]
            l.flashcount += 1
            # Return the link to its original color after a delay.
            wx.FutureCall(500, self.flash_link_off, l, link)
            
            dc = wx.ClientDC(self)
            l.Draw(dc)
            l.src.Draw(dc)
            l.dst.Draw(dc)
            
    def flash_link_off(self, link, linkmodel):
        # make sure this link hasn't been deleted
        if linkmodel in self.link_dict:
            link.flashcount -= 1
            dc = wx.ClientDC(self)
            link.Draw(dc)
            link.src.Draw(dc)
            link.dst.Draw(dc)
