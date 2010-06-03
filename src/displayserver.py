#! /usr/bin/env python

import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import robo,robotLoader
import pickle
from configparser import parseConfig
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
        self.initGL()
        self.pendingObjects=[]
        self.parseConfig()

    def initGL(self):
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
        glutInitWindowSize(640, 480)
        glutInitWindowPosition(0, 0)
        window = glutCreateWindow("Robotviewer Server")
        glutDisplayFunc(self.DrawGLScene)
        glutIdleFunc(self.DrawGLScene)
        glutReshapeFunc(ReSizeGLScene)
        self._glwin=GlWindow(640, 480, "Robotviewer Server")

    def update_hrp_joint_link(self,robot_name, joint_rank_xml):
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

        for joint in self._element_dict[robot_name]._robot.joint_list:
            if correct_joint_dict.has_key(joint.name):
                joint.id = correct_joint_dict[joint.name]

        self._element_dict[robot_name]._robot.update_joint_dict() 
        return

    def parseConfig(self):
        configs = dict()
        configs = parseConfig(config_file)
        if configs.has_key('robots'):
            robots = configs['robots']
            for (robot_name,robot_config) in robots.items():
                if not os.path.isfile(robot_config):
                    print "WARNING: Couldn't load %s. Are you sure %s exists?"\
                        %(robot_name,robot_config)
                    continue
                self.createElement('robot',robot_name,robot_config)
                self.enableElement(robot_name)        
        else:
            print """Couldn't any default robots. Loading an empty scene
    You might need to load some robots yourself. 
    See documentation"""

        if configs.has_key('joint_ranks'):
            jranks = configs['joint_ranks']
            for (robot_name, joint_rank_config) in robots.items():
                if not self._element_dict.has_key(robot_name):
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
                self.createElement('script',name,description)
                self.enableElement(name)
        return

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

