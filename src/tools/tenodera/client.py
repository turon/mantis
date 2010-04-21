# This file is part of MANTIS OS, Operating System
# See http://mantis.cs.colorado.edu/
#
# Copyright (C) 2003-2005 University of Colorado, Boulder
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the mos license (see file LICENSE)

import sys
from twisted.spread import pb
from twisted.internet import reactor
from twisted.python import util

# Connect to the server
factory = pb.PBClientFactory()
reactor.connectTCP("localhost", 1529, factory)

# Get our node id
node_id = sys.argv[1]

# Since it could take a while to connect and get the object, getRootObject
# returns a deferred object.  This is an object we can attach callbacks to
# so that when a connection is eventually made it will be handled.  In the
# meantime we don't have to block, we just start running.
d = factory.getRootObject()
d.addCallback(lambda object: object.callRemote("register", node_id))
d.addErrback(lambda reason: 'error: '+str(reason.value))
d.addCallback(util.println)
d.addCallback(lambda _: reactor.stop())
reactor.run()
