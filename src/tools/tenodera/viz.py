#!/usr/bin/python
# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import os, sys, time, string
import wx

from node_window import *
from net_view import *
import net_model, simClient

# Menu ID numbers
ID_FILE_EXIT  = 102
ID_FILE_ADD  = 103
ID_FILE_AGAIN  = 104

ID_SIM_NEW    = 201
ID_SIM_LOAD   = 202
ID_SIM_SAVE   = 203
ID_SIM_START  = 204
ID_SIM_STOP   = 205

ID_WIN_NODE   = 301

ID_HELP_ABOUT = 101

# File filters for open and save dialogs
cwildcard = "Configurations (*.ten)|*.ten|"     \
           "All files (*.*)|*.*"
bwildcard = "Binaries (*.elf)|*.elf|"     \
           "All files (*.*)|*.*"

# class viz_splash(splashscreen):
#     def __init__(self):
#         path = "bitmaps/splash.jpg"
#         adjusted_path = apply(os.path.join, tuple(path.split('/')))
#         bmp = Image(adjusted_path, wxBITMAP_TYPE_JPEG).ConvertToBitmap()
#         SplashScreen.__init__(self, bmp,
#                                 wx.SPLASH_CENTRE_ON_SCREEN|wx.SPLASH_TIMEOUT,
#                                 4000, None, -1,
#                                 style = wx.SIMPLE_BORDER|wx.FRAME_NO_TASKBAR|wx.STAY_ON_TOP)
#         wx.EVT_CLOSE(self, self.OnClose)

#     def OnClose(self, evt):
#         frame = viz_frame(NULL, -1, "MANTIS Visualizer")
#         frame.Show()
#         evt.Skip()  # Make sure the default handler runs too...

class viz_frame(wx.Frame):
    "Main visualizer window."
        
    def __init__(self, parent, ID, model):
        wx.Frame.__init__(self, parent, ID, "Visualizer",
                         wx.DefaultPosition, wx.Size(800, 600),
                         style = wx.DEFAULT_FRAME_STYLE | wx.NO_FULL_REPAINT_ON_RESIZE)
         
        self.model = model
        self.lastbindir = os.getcwd() + "/binaries"
        self.lastconfigdir = os.getcwd() + "/configurations"
        self.lastnode = None

        # Statusbar
        self.CreateStatusBar()
        self.SetStatusText("This is the statusbar")

        # File Menu
        file_menu = wx.Menu()
        file_menu.Append(ID_FILE_ADD, "&Add Node...", "Load a node")
        file_menu.Append(ID_FILE_AGAIN, "Add A&gain", "Load the same node type again")
        file_menu.AppendSeparator()
        file_menu.Append(ID_FILE_EXIT, "E&xit", "Terminate the program")

        # Simulation Menu
        sim_menu = wx.Menu()
        sim_menu.Append(ID_SIM_NEW, "&New", "Develop a new simulation.")
        sim_menu.Append(ID_SIM_LOAD, "&Load", "Load a simulation.")
        sim_menu.Append(ID_SIM_SAVE, "&Save", "Save the current simulation.")
        sim_menu.AppendSeparator()
        sim_menu.Append(ID_SIM_START, "S&tart", "Start the current simulation")
        sim_menu.Append(ID_SIM_STOP, "St&op", "Stop the current simulation")

        win_menu = wx.Menu()
        win_menu.Append(ID_WIN_NODE, "Node Window", "Open the node window")

        help_menu = wx.Menu()
        help_menu.Append(ID_HELP_ABOUT, "&About", "More information about this program")

        # Menubar
        menuBar = wx.MenuBar()
        menuBar.Append(file_menu, "&File");
        menuBar.Append(sim_menu, "&Simulation")
        menuBar.Append(win_menu, "&Window")
        menuBar.Append(help_menu, "&Help")

        self.SetMenuBar(menuBar)

        # Handle menu item events
        wx.EVT_MENU(self, ID_FILE_ADD, self.OnAddNode)
        wx.EVT_MENU(self, ID_FILE_AGAIN, self.OnNodeAgain)
        wx.EVT_MENU(self, ID_HELP_ABOUT, self.OnAbout)
        wx.EVT_MENU(self, ID_FILE_EXIT,  self.OnQuit)
        wx.EVT_MENU(self, ID_WIN_NODE, self.OnWinNode)
        wx.EVT_MENU(self, ID_SIM_NEW, self.OnNew)
        wx.EVT_MENU(self, ID_SIM_LOAD, self.OnLoad)
        wx.EVT_MENU(self, ID_SIM_SAVE, self.OnSave)
        
        # Handle close window event
#        self.Bind(wx.EVT_CLOSE, self.OnWindowClose)
        wx.EVT_CLOSE(self, self.OnWindowClose)
        
        self.node_win = None

#        splash = viz_splash()
#        splash.Show()

    #
    # Menu event handlers
    #
    
    def OnAbout(self, event):
        dlg = wx.MessageDialog(self, "This is a general purpose visualization tool\n"
                              "for the development and deployment of sensor networks.\n",
                              "About The Mantis Visualizer", wx.OK | wx.ICON_INFORMATION)
        dlg.ShowModal()
        dlg.Destroy()

    def OnQuit(self, event):
        self.Close(True)

    def OnWindowClose(self, event):
        self.model.ClearNetwork()
        self.Destroy()

    def OnWinNode(self, event):
        if self.node_win is None:
            self.node_win = node_window(self)

        self.node_win.Show(True)
        
    def OnNew(self, event):
        self.model.SetFileName(None)
        self.model.ClearNetwork()
        
    def OnLoad(self, event):
        dlg = wx.FileDialog(
            self, message="Choose a file", defaultDir=self.lastconfigdir, 
            defaultFile="", wildcard=cwildcard, style=wx.OPEN | wx.HIDE_READONLY | wx.FILE_MUST_EXIST
            )

        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            # Remember the last directory we loaded from.
            self.lastconfigdir = path[:string.rfind(path,"/")]
            self.model.LoadNetwork(path)
 
    def OnAddNode(self, event):
        dlg = wx.FileDialog(
            self, message="Choose an XMOS node binary", defaultDir=self.lastbindir, 
            defaultFile="", wildcard=bwildcard, style=wx.OPEN | wx.HIDE_READONLY | wx.FILE_MUST_EXIST
            )

        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            # Remember the last direcotory we chose a node from.
            self.lastbindir = path[:string.rfind(path,"/")]
            # For 'Add Node Again', remember the last node we added.
            self.lastnode = path
            self.model.LaunchNode(path, 100, 100)
            
    def OnNodeAgain(self, event):
        if self.lastnode:
            self.model.LaunchNode(self.lastnode, 100, 100)
        else:
            self.OnAddNode(event)
 
    def OnSave(self, event):
        path = self.model.GetFileName()
        # Show dialog for new files.
        if not path:
            dlg = wx.FileDialog(
                self, message="Choose a file", defaultDir=self.lastconfigdir, 
                defaultFile="", wildcard=cwildcard, style=wx.SAVE | wx.OVERWRITE_PROMPT
                )    
            if dlg.ShowModal() == wx.ID_OK:
                path = dlg.GetPath()
                self.lastconfigdir = path[:string.rfind(path,"/")]
                if not os.access(path, os.F_OK) and string.rfind(path,".")==-1:
                    # If we're creating a new file and the user did not type an extension...
                    path = path + ".ten"
        if path:
            self.model.SaveNetwork(path)

        
class viz_app(wx.App):
    "Visualizer application."
    
    # TODO I don't know what 'arg' is for, give it a better name
    def __init__(self, arg, model):
        self.model = model
        wx.App.__init__(self, arg)
    
    def OnInit(self):        
        # Viz Frame
        frame = viz_frame(None, -1, self.model)
        nview = net_view(frame, -1, self.model)
        frame.Window = nview
        
        frame.Show()
        self.SetTopWindow(frame)
#        splash = viz_splash()
#        splash.Show()
        
        return True

# "Main entry point", start the visualizer.
# This data structure holds a local mirror of the XMOS network.
model = net_model.net_model()
# Hook it up to the sim server to keep each other updated.
client = simClient.simClient(model)
# Hook it up to the visualizer, too.
app = viz_app(0, model)
app.MainLoop()
