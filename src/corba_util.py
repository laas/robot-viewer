# Copyright (c) 2010-2011, Duong Dang <mailto:dang.duong@gmail.com>
# This file is part of robot-viewer.

# robot-viewer is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# robot-viewer is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with robot-viewer.  If not, see <http://www.gnu.org/licenses/>.
#! /usr/bin/env python


__author__ = "Duong Dang"
__version__ = "1.0"

import sys, logging
# Import the CORBA module
from omniORB import CORBA, PortableServer
import CosNaming

logger = logging.getLogger("robotviewer.corba_util")
ch = logging.StreamHandler()

def GetObject(module_name, obj_name, context_name , poa_path = None):
    """
    def GetObject(module_name, obj_name, context_name , poa_path = None):
    Arguments:
    - `module_name`:
    - `context_name`: list of names to the object [("name1","context1"),("name2","context2"),...,("name","object")]
    - `obj_name`: typically "module.interface as" defined in idl file
    - `poa_path`: path to omniidl generated interface

    Example:
         loscli=corba_util.GetObject("robotviewer_corba","robotviewer_corba.Localstepper",[('robotviewer_corba','context'),('localstepper','object')])
    """
    if poa_path:
        sys.path = [poa_path] + sys.path

    exec("import %s"%module_name)

    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

    # Obtain a reference to the root naming context
    obj         = orb.resolve_initial_references("NameService")
    rootContext = obj._narrow(CosNaming.NamingContext)

    if context_name:
        try:
            name = [ CosNaming.NameComponent(component[0],component[1]) for component in context_name]
        except Exception,e:
            raise Exception("Failed to construct name %s"%str(context_name))
        else:
            obj =  rootContext.resolve(name)

    exec("req_obj = obj._narrow(%s)"%obj_name)

    if req_obj is None:
        raise Exception("Object ref is not an %s"%obj_name)

    return req_obj


def StartServer( server_instance, module_name, obj_name, context_name , poa_path = None):
    if poa_path:
        sys.path = [poa_path] + sys.path
    exec("import %s"%module_name)
    exec("import %s__POA"%module_name)

    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

    # Find the root POA
    poa = orb.resolve_initial_references("RootPOA")
    # Create an object reference, and implicitly activate the object
    obj         = orb.resolve_initial_references("NameService")
    rootContext = obj._narrow(CosNaming.NamingContext)

    if rootContext is None:
        logger.fatal("Failed to narrow the root naming context.\n"+\
            "Is omniNames running?")
        sys.exit(1)
    testContext = None
    name = [ CosNaming.NameComponent(component[0],component[1]) for component in context_name]
    try:
        testContext = rootContext.bind_new_context(name)
        logger.info( "New %s context bound"%str(context_name))
    except:
        logger.info( "%s context already exists"%str(context_name))
        obj = rootContext.resolve(name)
        testContext = obj._narrow(CosNaming.NamingContext)
        if testContext is None:
            logger.info( "robotviewer.context exists but is not a NamingContext")
            sys.exit(1)

    # Bind the Echo object to the test context
    components = obj_name.split(".")
    if len(components) != 2:
        raise Exception ("Invalid objectname %s"%obj_name)
    name = [CosNaming.NameComponent(components[0], components[1])]

    obj_ref = server_instance._this()
    try:
        testContext.bind(name, obj_ref)
        logger.info( "New Request object bound")

    except CosNaming.NamingContext.AlreadyBound:
        testContext.rebind(name, obj_ref)
        logger.info( "Request binding already existed -- rebound")

    # Note that is should be sufficient to just call rebind() without
    # calling bind() first. Some Naming service implementations
    # incorrectly raise NotFound if rebind() is called for an unknown
    # name, so we use the two-stage approach above

    # Activate the POA
    poaManager = poa._get_the_POAManager()
    poaManager.activate()

    # Everything is running now, but if this thread drops out of the end
    # of the file, the process will exit. orb.run() just blocks until the
    # ORB is shut down
