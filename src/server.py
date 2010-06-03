#! /usr/bin/env python

import sys,os
from configparser import parseConfig

from displayserver import *

corba_path = os.path.dirname(os.path.abspath(__file__)) + '/corba'
sys.path = [corba_path] + sys.path

import RobotViewer, RobotViewer__POA

class DisplayServerCorba(DisplayServer):
    """Corba implement of OpenGL server
    """
    
    def __init__(self, element_dict = dict()):
        """
        
        Arguments:
        - `element_dict`: dictionary containing all elements to be rendered
        """
        DisplayServer.__init__(self,element_dict)
        self._initCorba()

    def _initCorba(self):
        edict = self._element_dict
        ds = self
        ##################################
        #      omniORB
        ##################################
        from omniORB import CORBA, PortableServer

        # Import the stubys for the Naming service
        import CosNaming

        # Import the stubs and skeletonsg
        import imp
        import RobotViewer, RobotViewer__POA
        corbaUsage=""" Request(cmd,str_args,config)

Example:

cmd                  str_args                             conf
---                  ---                                  ---
createElement        robot hrp ./HRP.wrl                  []
destroyElement       hrp                                  []
enableElement        hrp                                  []
disableElement       hrp                                  []
updateElementConfig  hrp                                  [0,0,0,0...] <vector of length 6+40>
listElements         ""                                   [] 
"""


        # Define an implementation of the Echo interface
        class Request_i (RobotViewer__POA.Request):
            def req(self, cmd, str_args, conf):
#                print "receive", cmd, str_args, conf
                if cmd == "createElement":
                    words=str_args.split()
                    if len(words) < 3:
                        return corbaUsage
                    etype = words[0]
                    name  = words[1]
                    desc  = re.sub(r"^\s*\w+\s*\w+\s*",'',str_args)
                    ds.pendingObjects.append((etype,name,desc))

                elif cmd == "destroyElement":
                    words=str_args.split()
                    if len(words) !=1:
                        return corbaUsage
                    name = words[0]
                    try:
                        ds.destroyElement(name)
                    except Exception,error:
                        return str(error)     
                elif cmd == "enableElement":
                    words=str_args.split()
                    if len(words) !=1:
                        return corbaUsage
                    name = words[0]
                    try:
                        ds.enableElement(name)
                    except Exception,error:
                        return str(error)                 
                elif cmd == "disableElement":
                    words=str_args.split()
                    if len(words) !=1:
                        return corbaUsage
                    name = words[0]
                    try:
                        ds.disableElement(name)
                    except Exception,error:
                        return str(error) 
                elif cmd == "updateElementConfig":
                    config = conf
                    words=str_args.split()
                    if len(words) !=1:
                        return corbaUsage
                    name = words[0]
                    try:
                        ds.updateElementConfig(name,config)
                    except Exception,error:
                        return str(error) 
                elif cmd == "list":
                    s=""
                    for (name,element) in edict.items():
                        s += name
                        s += "\n" + str(element)
                    
                    return s
                else:
                    return corbaUsage
                return "OK"

        # Initialise the ORB
        orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

        # Find the root POA
        poa = orb.resolve_initial_references("RootPOA")

        # Create an instance of Request_i
        ri = Request_i()

        # Create an object reference, and implicitly activate the object
        ro = ri._this()

        # Obtain a reference to the root naming context
        obj         = orb.resolve_initial_references("NameService")
        rootContext = obj._narrow(CosNaming.NamingContext)

        if rootContext is None:
            print "Failed to narrow the root naming context.\n"+\
                "Is omniNames running?"
            sys.exit(1)

        # Bind a context named "test.my_context" to the root context
        name = [CosNaming.NameComponent("robotviewer", "context")]

        try:
            testContext = rootContext.bind_new_context(name)
            print "New robotviewer context bound"

        except CosNaming.NamingContext.AlreadyBound, ex:
            print "robotviewer context already exists"
            obj = rootContext.resolve(name)
            testContext = obj._narrow(CosNaming.NamingContext)
            if testContext is None:
                print "robotviewer.context exists but is not a NamingContext"
                sys.exit(1)

        # Bind the Echo object to the test context
        name = [CosNaming.NameComponent("Request", "Object")]

        try:
            testContext.bind(name, ro)
            print "New Request object bound"

        except CosNaming.NamingContext.AlreadyBound:
            testContext.rebind(name, ro)
            print "Request binding already existed -- rebound"

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


def update_hrp_joint_link(robot_name, joint_rank_xml):
    """
    """
    if not os.path.isfile(joint_rank_xml):
        return 

    pattern=re.compile(r"\s*<Link>\s*(\w+)\s*(\d+)\s*<\/Link>\s*")
    lines = open(joint_rank_xml).readlines()
    correct_joint_dict = dict()
        
    for line in lines:
        m = pattern.match(line)
        if m:
            correct_joint_dict[m.group(1)] = int(m.group(2)) -6 
            print m.group(1), "\t",m.group(2)

    for joint in ds._element_dict[robot_name]._robot.joint_list:
        if correct_joint_dict.has_key(joint.name):
            joint.id = correct_joint_dict[joint.name]

    ds._element_dict[robot_name]._robot.update_joint_dict() 

def main():
    """Main function
    """
    configs = dict()
    configs = parseConfig(config_file)
    ds=DisplayServerCorba()          
    if configs.has_key('robots'):
        robots = configs['robots']
        for (robot_name,robot_config) in robots.items():
            if not os.path.isfile(robot_config):
                print "WARNING: Couldn't load %s. Are you sure %s exists?"\
                    %(robot_name,robot_config)
                continue
            ds.createElement('robot',robot_name,robot_config)
            ds.enableElement(robot_name)        
    else:
        print """Couldn't any default robots. Loading an empty scene
You might need to load some robots yourself. 
See documentation"""

    if configs.has_key('joint_ranks'):
        jranks = configs['joint_ranks']
        for (robot_name, joint_rank_config) in robots.items():
            if not ds._element_dict.has_key(robot_name):
                continue
            if not os.path.isfile(joint_rank_config):
                continue
            update_hrp_joint_link(robot_name,joint_rank_config)

    if configs.has_key('scripts'):
        scripts = configs['scripts']
        for (name, script_file) in scripts.items():
            if not os.path.isfile(script_file):
                warnings.warn('Could not find %s'%script_file)                
                continue
            description = open(script_file).read()
            ds.createElement('script',name,description)
            ds.enableElement(name)
    

    ds.run()

if __name__ == '__main__':
    main()

