# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import os, re

import wx
from wx.lib.filebrowsebutton import DirBrowseButton

class node_window(wx.Dialog):
    def __init__(self, parent):
        wx.Dialog.__init__(self, parent, -1, "Node Selector", wx.DefaultPosition,
                             (400,200), wx.DEFAULT_DIALOG_STYLE)

        sizer = wx.BoxSizer(wx.VERTICAL)

        self.dbb = DirBrowseButton(self, -1, wx.Point(20,80), wx.Size(450, -1),
                                   changeCallback = self.OnDbb)

        self.SetModal(False)

        wx.EVT_CLOSE(self, self.OnClose)

        self.node_list = []

    def OnClose(self, event):
        self.Show(False)

    def OnDbb(self, event):
        dir = event.GetString()

        files = os.listdir(dir)

        elffile_reg = re.compile(r'(?P<nodename>.*)\.elf$')
        for elffile in files:
            match = elffile_reg.match(elffile)
            if (match):
                self.node_list.append((elffile, match.group('nodename')))

        print self.node_list
