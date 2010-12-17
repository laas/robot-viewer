#! /usr/bin/env python


# Copyright LAAS/CNRS 2009-2010
# Authors Duong Dang
__author__ = "Duong Dang"
__version__ = "1.0"

import sys, logging
# Import the CORBA module
from omniORB import CORBA, PortableServer
import CosNaming

logger = logging.getLogger("corba_util")
ch = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s:%(name)s:%(levelname)s - %(message)s")
ch.setFormatter(formatter)
logger.addHandler(ch)
ch.setLevel(logging.INFO)
logger.setLevel(logging.INFO)

def GetObject(module_name, obj_name, context_name , poa_path = None):
    """
    def GetObject(module_name, obj_name, context_name , poa_path = None):
    Arguments:
    - `module_name`:
    - `context_name`: list of names to the object [("name1","context1"),("name2","context2"),...,("name","object")]
    - `obj_name`: typically "module.interface as" defined in idl file
    - `poa_path`: path to omniidl generated interface

    Example:
         loscli=corba_util.GetObject("hpp","hpp.Localstepper",[('hpp','context'),('localstepper','object')])
    """
    global logger
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
    global logger
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
        textContext = rootContext.bind_new_context(name)
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
