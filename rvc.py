#!/usr/bin/env python

import sys,os

# Import the CORBA module
from omniORB import CORBA

# Import the stubs for the CosNaming and RoboViewer modules
import CosNaming, RoboViewer

# Initialise the ORB
orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

# Obtain a reference to the root naming context
obj         = orb.resolve_initial_references("NameService")
rootContext = obj._narrow(CosNaming.NamingContext)

if rootContext is None:
    print "Failed to narrow the root naming context"
    sys.exit(1)

# Resolve the name "test.my_context/ExampleEcho.Object"
name = [CosNaming.NameComponent("test", "my_context"),
        CosNaming.NameComponent("Request", "Object")]

try:
    obj = rootContext.resolve(name)

except CosNaming.NamingContext.NotFound, ex:
    print "Name not found"
    sys.exit(1)

# Narrow the object to an RoboViewer::Request
eo = obj._narrow(RoboViewer.Request)

if eo is None:
    print "Object reference is not an Example::Echo"
    sys.exit(1)

# Invoke the echoString operation
message = " ".join(sys.argv[1:])
path=os.getcwd()+"/ "
result  = eo.req(path+message)

if result!="":
    print result
