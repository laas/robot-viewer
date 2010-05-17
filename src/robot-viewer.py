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

    def createElement(self,etype,name,description):
        """        
        Arguments:
        - `self`:
        - `etype`:        string, element type (e.g. robot, GLscript)
        - `name`:         string, element name
        - `description`:  string, description  (e.g. wrl path)
        """
        if self._element_dict.has_key(name):
            raise KeyError,"Element with that name exists already"
        
        if etype == 'robot':
            new_robot = robotLoader.robotLoader(description,True)
            new_element = DsRobot(new_robot)
            self._element_dict[name] = new_element
        elif etype == 'script':
            new_element = DsScript(description)
            self._element_dict[name] = new_element
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

        self._element_dict[name].UpdateConfig()

    def run(self):
        glutMainLoop()

    def DrawGLScene(self):
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
                if cmd == "createElement":
                    words=str_args.split()
                    if len(words) < 3:
                        return corbaUsage
                    etype = words[0]
                    name  = words[1]
                    desc  = words[2:]
                    try:
                        ds.createElement(etype,name,desc)
                    except Exception,error:
                        return error
                elif cmd == "destroyElement":
                    words=str_args.split()
                    if len(words) !=1:
                        return corbaUsage
                    name = words[0]
                    try:
                        ds.destroyElement(name)
                    except Exception,error:
                        return error     
                elif cmd == "enableElement":
                    words=str_args.split()
                    if len(words) !=1:
                        return corbaUsage
                    name = words[0]
                    try:
                        ds.enableElement(name)
                    except Exception,error:
                        return error                 
                elif cmd == "disableElement":
                    words=str_args.split()
                    if len(words) !=1:
                        return corbaUsage
                    name = words[0]
                    try:
                        ds.disableElement(name)
                    except Exception,error:
                        return error 
                elif cmd == "updateElementConfig":
                    config = conf
                    words=str_args.split()
                    if len(words) !=1:
                        return corbaUsage
                    name = words[0]
                    try:
                        ds.updateElementConfig(name,config)
                    except Exception,error:
                        return error 
                elif cmd == "list":
                    return str(edict)
                else:
                    return corbaUsage
                return "Done!"

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
            print "Failed to narrow the root naming context"
            sys.exit(1)

        # Bind a context named "test.my_context" to the root context
        name = [CosNaming.NameComponent("test", "my_context")]

        try:
            testContext = rootContext.bind_new_context(name)
            print "New test context bound"

        except CosNaming.NamingContext.AlreadyBound, ex:
            print "Test context already exists"
            obj = rootContext.resolve(name)
            testContext = obj._narrow(CosNaming.NamingContext)
            if testContext is None:
                print "test.mycontext exists but is not a NamingContext"
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

def main():
    """Main function
    """
    
    ds=DisplayServer()

    lp=[]
    N=5
    L=0.5*N

    for i in range(-N,N+1):   
        lp.append([i*0.5,L,0.005])
        lp.append([i*0.5,-L,0.005])
        lp.append([L,i*0.5,0.005])
        lp.append([-L,i*0.5,0.005] )

    script = "glMaterialfv(GL_FRONT_AND_BACK,  GL_AMBIENT_AND_DIFFUSE, [1,1,1,1])\n"
    script+= "glMaterialfv(GL_FRONT_AND_BACK,  GL_SPECULAR           , [1,1,1,1])\n"
    script+= "glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0)\n"
    script+= "glBegin(GL_LINES)\n"    
    for point in lp:
        script+= "glVertex3f(%f, %f, %f)\n"%(point[0],point[1],point[2])
    script+= "glEnd()"
    ds.createElement('script','floor',script)
    ds.enableElement('floor')       

    ds.createElement('robot','hrp',(os.environ['HOME']+'/licenses/'+\
                                      'HRP2JRL/model/HRP2JRLmain.wrl'))
    ds.enableElement('hrp')
    
    ds.run()

if __name__ == '__main__':
    main()

