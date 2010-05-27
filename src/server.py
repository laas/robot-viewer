#! /usr/bin/env python

import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys
import robo,robotLoader
import pickle
from openglaux import IsExtensionSupported,ReSizeGLScene, GlWindow
from dsElement import *
import re,imp
from configparser import parseConfig

path = imp.find_module('robotviewer')[1]
sys.path.append(path+'/corba')
import RobotViewer, RobotViewer__POA
del path

class DisplayServer(object):
    """OpenGL server
    """
    
    def __init__(self, element_dict = dict()):
        """
        
        Arguments:
        - `element_dict`: dictionary containing all elements to be rendered
        """
        self._element_dict = element_dict
        
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
        glutInitWindowSize(640, 480)
        glutInitWindowPosition(0, 0)
        window = glutCreateWindow("Display Server")
        glutDisplayFunc(self.DrawGLScene)
        glutIdleFunc(self.DrawGLScene)
        glutReshapeFunc(ReSizeGLScene)
        self._glwin=GlWindow(640, 480, "Display Server")
        self._initCorba()
        self.pendingObjects=[]


    def createElement(self,etype,ename,edescription):
        """        
        Arguments:
        - `self`:
        - `etype`:        string, element type (e.g. robot, GLscript)
        - `name`:         string, element name
        - `description`:  string, description  (e.g. wrl path)
        """
        if self._element_dict.has_key(ename):
            raise KeyError,"Element with that name exists already"
        
        if etype == 'robot':
            new_robot = robotLoader.robotLoader(edescription,True) 
            new_element = DsRobot(new_robot)
            self._element_dict[ename] = new_element

        elif etype == 'script':
            new_element = None
            new_element = DsScript(edescription)
            self._element_dict[ename] = new_element
        else:
            raise TypeError,"Unknown element type"

    def destroyElement(self,name):
        """        
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self._element_dict.has_key(name):
            raise KeyError,"Element with that name does not exist"

        del self._element_dict[name]
        self._element_dict.pop(name)


    def enableElement(self,name):
        """
        
        Arguments:
        - `self`:
        - `name`:
        """
        if not self._element_dict.has_key(name):
            raise KeyError,"Element with that name does not exist"

        self._element_dict[name].enable()

    def disableElement(self,name):
        """        
        Arguments:
        - `self`:
        - `name`:
        """
        if not self._element_dict.has_key(name):
            raise KeyError,"Element with that name does not exist"

        self._element_dict[name].disable()

    def updateElementConfig(self,name,config):
        """        
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self._element_dict.has_key(name):
            raise KeyError,"Element with that name does not exist"

        self._element_dict[name].updateConfig(config)

    def run(self):
        glutMainLoop()

    def DrawGLScene(self):
        if len(self.pendingObjects) > 0:
            obj = self.pendingObjects.pop()
            print "creating", obj[0], obj[1], obj[2]
            self.createElement(obj[0],obj[1],obj[2])
        # Clear Screen And Depth Buffer
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
        glLoadIdentity ();
        # # Reset The Modelview Matrix        
        # # Get FPS
        # milliseconds = win32api.GetTickCount() 
        self._glwin.updateFPS()
        self._glwin._g_nFrames += 1 
        p=self._glwin.camera.position
        f=self._glwin.camera.lookat
        u=self._glwin.camera.up
        gluLookAt(p[0],p[1],p[2],f[0],f[1],f[2],u[0],u[1],u[2])

        for item in self._element_dict.items():
            ele = item[1]
#            print item[0], item[1]._enabled
            ele.render()

        glutSwapBuffers()

        return True

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
                    return str(edict)
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
    config_dir = os.environ['HOME']+'/.robotviewer/'
    config_file = config_dir + 'config'
    configs = dict()
    configs = parseConfig(config_file)
    print "parsed config:",configs
    ds=DisplayServer()          
    if configs.has_key('robots'):
        robots = configs['robots']
        for (robot_name,robot_config) in robots.items():
            if not os.path.isfile(robot_config):
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

