#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2010
# Authors Duong Dang

import getopt,sys, warnings, imp, os, imp

path = imp.find_module('robotviewer')[1]
sys.path.append(path+'/corba')
import RobotViewer, RobotViewer__POA
del path

# Import the CORBA module
from omniORB import CORBA

# Import the stubs for the CosNaming and RobotViewer modules
import imp
path = imp.find_module('robotviewer')[1]
sys.path.append(path)
import RobotViewer
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
name = [CosNaming.NameComponent("robotviewer", "context"),
        CosNaming.NameComponent("Request", "Object")]

try:
    obj = rootContext.resolve(name)

except CosNaming.NamingContext.NotFound, ex:
    print "Name not found"
    sys.exit(1)

# Narrow the object to an RobotViewer::Request
eo = obj._narrow(RobotViewer.Request)

if eo is None:
    print "Object reference is not an Example::Echo"
    sys.exit(1)

# Invoke the echoString operation

def execute(cmd,str_args,conf):
    """Send raw command using format described in idl
    """
    return eo.req(cmd, str_args,conf)

def updateConfig(element_name,conf):
    """Update configuration of an element (robot, object, etc.)
    """
    return execute('updateElementConfig',element_name,conf)

def enable(element_name):
    """Make an element visible    
    """
    return execute('enableElement',element_name,[])

def disable(element_name):
    """Make an element invisible    
    """
    return execute('disableElement',element_name,[])

def create(etype,element_name,description):
    """Create a new element
    """
    return execute('createElement',' '.join(etype,element_name,description),[])

def list():
    """List all elements present in the server
    """
    print execute('list','',[])

def help():
    """Print help message
    """
    print  '''
Avalable commands
help()                              print this message
list()                              list all objects on server
disable(name)                       make something invisible
enable(name)                        make somethin visible
create(type,name,description)       create a new element 
                                      e.g. create('robot','hrp','/path/to/HRP2.wrl'
updateConfig(name,config)           move an element around
'''
