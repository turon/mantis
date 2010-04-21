# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

from twisted.spread import pb
from twisted.internet import reactor

class radio_model(pb.Root):
    def remote_register(self, node_id):
        print 'registering: ', node_id
        return

if __name__ == '__main__':
    reactor.listenTCP(1529, pb.PBServerFactory(radio_model()))
    reactor.run()
