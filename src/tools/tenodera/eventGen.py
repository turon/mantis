#!/usr/bin/python
# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import socket, struct, sys

sys.path.append('modules')
import events
sys.path.pop()

PORT = 1521
HOST = '' # empty string meanslocalhost 

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def start_node(binary, id, x, y):
    packstr = "!5i" + str(len(binary)) + "s"
    data_size = 12 + len(binary)
    pkt = struct.pack(packstr, data_size, events.NEW_NODE_EVENT, id, x, y, binary)
    s.sendto(pkt, (HOST, PORT))

id = 0
x = 10
y = 10

for i in range(0, 3):
    start_node('binaries/sense_forward_recv.elf', id, x, y)
    id += 1
    x += 1
    y += 1

start_node('binaries/sense_forward_send.elf', id, x, y)

while 1:
    time.sleep(1)

s.close()
