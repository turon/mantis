#!/usr/bin/python

import socket, signal, sys, atexit, os, threading, thread, termios, struct, time

import xml.sax.handler
import getprops

class netevents:

    def graceful_quit (self, one, two):
        print ("Exiting..")
        client_sock.close()
        sys.exit (0)

    def listen(self):
        # retrieve header of size, type, timeval
        data = client_sock.recv(12)
        if len(data) == 0:
            print 'Server Exited...'
            return
        data = struct.unpack("!iil", data)
        size = data[0]
        type = data[1]
        tv = data[2]

        # retrieve payload, add 4 for the node id
        data = client_sock.recv(size + 4)

        return data, type, tv

    def __init__(self):
        self.running = 1
        signal.signal (signal.SIGKILL, self.graceful_quit)
        signal.signal (signal.SIGSTOP, self.graceful_quit)
        signal.signal (signal.SIGQUIT, self.graceful_quit)
        signal.signal (signal.SIGTERM, self.graceful_quit)
        signal.signal (signal.SIGINT,  self.graceful_quit)
        atexit.register (self.graceful_quit, 1, 2)
        
        HOST = ''
        PORT = 4200

       
        global client_sock
        client_sock = socket.socket (socket.AF_INET, socket.SOCK_STREAM)

        try:
            client_sock.connect((HOST, PORT))
        except:
            print "Connecting to host failed."
            return
