#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2010
# Authors Duong Dang

import getopt,sys, warnings, imp, os, imp

path = imp.find_module('robotviewer')[1]
sys.path.append(path+'/corba')
del path
# Import the CORBA module
from omniORB import CORBA

# Import the stubs for the CosNaming and hppCorbaServer modules
import hppCorbaServer
import CosNaming

# Initialise the ORB
orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

# Obtain a reference to the root naming context
obj         = orb.resolve_initial_references("NameService")
rootContext = obj._narrow(CosNaming.NamingContext)

if rootContext is None:
    print "Failed to narrow the root naming context"
    sys.exit(1)

# Resolve the name "test.my_context/ExampleEcho.Object"
name = [CosNaming.NameComponent("sot", "context"),
        CosNaming.NameComponent("coshell", "servant")]

try:
    obj = rootContext.resolve(name)

except CosNaming.NamingContext.NotFound, ex:
    print "Name not found"
    sys.exit(1)

# Narrow the object to an hppCorbaServer::SOT_Server_Command
req_obj = obj._narrow(hppCorbaServer.SOT_Server_Command)

if req_obj is None:
    print "Object reference is not an Example::Echo"
    sys.exit(1)

# Invoke the echoString operation

def get_HRP_pos():
    """
    """
    return req_obj.readVector("OpenHRP.state")

def get_wst():
    return req_obj.readVector("dyn.ffposition")


import robotviewer.client as rvcli
from time import sleep
def main():
    """
    """
    while True:
        sleep(0.02)
        pos = get_HRP_pos()
        wst = get_wst()
        
        if len(wst) < 6:
            print "Couldn't get waist position and/or orientation. Do nothin!"
        for i in range(len(wst)):
            pos[i] = wst[i]
                
        pos[2] += 0.105
        rvcli.updateConfig('hrp',pos)
if __name__ == '__main__':
    main()
