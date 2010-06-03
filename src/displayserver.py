#! /usr/bin/env python

import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import robo,robotLoader
import pickle

from openglaux import IsExtensionSupported,ReSizeGLScene, GlWindow
from dsElement import *
import re,imp

import pickle
config_dir = os.environ['HOME']+'/.robotviewer/'
config_file = config_dir + 'config'

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
        window = glutCreateWindow("Robotviewer Server")
        glutDisplayFunc(self.DrawGLScene)
        glutIdleFunc(self.DrawGLScene)
        glutReshapeFunc(ReSizeGLScene)
        self._glwin=GlWindow(640, 480, "Robotviewer Server")
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
            edes = edescription.replace("/","_")
            cached_file = config_dir+"/%s.cache"%edes
            if os.path.isfile(cached_file):
                print "Using cached file %s.\n Remove it to reparse the wrl/xml file"%cached_file
                new_robot = pickle.load(open(cached_file))
            else:                    
                new_robot = robotLoader.robotLoader(edescription,True) 
                f = open(cached_file,'w')
                pickle.dump(new_robot,f)
                f.close()
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

