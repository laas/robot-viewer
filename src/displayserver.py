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
from camera import Camera
import pickle
config_dir = os.environ['HOME']+'/.robotviewer/'
config_file = config_dir + 'config'
import logging
import ConfigParser

def updateView(camera):
    """

    Arguments:
    - `camera`:
    """
    p=camera.position
    f=camera.lookat
    u=camera.up
    glLoadIdentity()
    gluLookAt(p[0],p[1],p[2],f[0],f[1],f[2],u[0],u[1],u[2])


class DisplayServer(object):
    """OpenGL server
    """

    def __init__(self,options = None, args = None):
        """

        Arguments:
        """
        if options and options.verbose:
            self.log_level = logging.DEBUG
        else:
            self.log_level = logging.INFO
        self.logger = logging.getLogger("RobotViewer")
        self.logger.setLevel(logging.DEBUG)
        # create console handler and set level to debug
        self.ch = logging.StreamHandler()
        self.ch.setLevel(self.log_level)
        formatter = logging.Formatter("%(asctime)s:%(name)s:%(levelname)s - %(message)s")
        self.ch.setFormatter(formatter)
        self.logger.addHandler(self.ch)


        self._element_dict = dict()
        self.initGL()
        self.pendingObjects=[]
        self.parseConfig()
        self.camera = Camera()
        self._mouseButton = None
        self._oldMousePos = [ 0, 0 ]

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
        self.bindEvents()

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
                self.logger.info( m.group(1)+ "\t" + m.group(2))

        for joint in self._element_dict[robot_name]._robot.joint_list:
            if correct_joint_dict.has_key(joint.name):
                joint.id = correct_joint_dict[joint.name]

        self._element_dict[robot_name]._robot.update_joint_dict()
        return

    def parseConfig(self):
        def replace_env_var(s):
            matches = re.findall(r'\$(\w+)',s)
            for m in matches:
                var = m
                val = os.environ[var]
                s = s.replace(var,val)
            s = s.replace('$','')
            return s

        config = ConfigParser.ConfigParser()
        config.read(config_file)
        self.logger.info( 'parsed_config %s'%config)
        if config.has_section('robots'):
            robot_names = config.options('robots')
            for robot_name in robot_names:
                robot_config = config.get('robots',robot_name)
                robot_config = replace_env_var(robot_config)
                self.logger.info( 'robot_config=%s'%robot_config)
                if not os.path.isfile(robot_config):
                    self.logger.info( "WARNING: Couldn't load %s. Are you sure %s exists?"\
                        %(robot_name,robot_config))
                    continue
                self.createElement('robot',robot_name,robot_config)
                self.enableElement(robot_name)
        else:
            self.logger.info( """Couldn't any default robots. Loading an empty scene
    You might need to load some robots yourself.
    See documentation""")

        if config.has_section('joint_ranks'):
            robot_names = config.options('joint_ranks')
            for robot_name in robot_names:
                joint_rank_config = config.get('joint_ranks',robot_name)
                joint_rank_config = replace_env_var(joint_rank_config)
                if not self._element_dict.has_key(robot_name):
                    continue
                if not os.path.isfile(joint_rank_config):
                    continue
                update_hrp_joint_link(robot_name,joint_rank_config)

        if config.has_section('scripts'):
            script_names = config.options('scripts')
            for script_name in script_names:
                script_file = config.get('scripts',script_name)
                script_file = replace_env_var(script_file)
                if not os.path.isfile(script_file):
                    warnings.warn('Could not find %s'%script_file)
                    continue
                description = open(script_file).read()
                self.createElement('script',script_name,description)
                self.enableElement(script_name)
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
                self.logger.info( "Using cached file %s.\n Remove it to reparse the wrl/xml file"%cached_file)
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

    def getElementConfig(self,name):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self._element_dict.has_key(name):
            raise KeyError,"Element with that name does not exist"

        return self._element_dict[name].getConfig()

    def listElement(self):
        return [name for name in self._element_dict.keys() ]

    def run(self):
        glutMainLoop()

    def Ping(self):
        return "pong"

    def DrawGLScene(self):
        if len(self.pendingObjects) > 0:
            obj = self.pendingObjects.pop()
            self.logger.info( "creating %s %s %s"%( obj[0], obj[1], obj[2]))
            self.createElement(obj[0],obj[1],obj[2])
        # Clear Screen And Depth Buffer
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity ();
        # # Reset The Modelview Matrix
        # # Get FPS
        # milliseconds = win32api.GetTickCount()

        if hasattr(self, '_glwin'):
            self._glwin.updateFPS()
            self._glwin._g_nFrames += 1

            updateView(self.camera)

        for item in self._element_dict.items():
            ele = item[1]
#            self.logger.info( item[0], item[1]._enabled)
            ele.render()

        glutSwapBuffers()

        return True

    def bindEvents(self):

        def keyPressedFunc(*args):
            # If escape is pressed, kill everything.
            if args[0] == ESCAPE : # exit when ESCAPE is pressed
                sys.exit ()
            return

        def mouseButtonFunc( button, mode, x, y ):
            """Callback function (mouse button pressed or released).

            The current and old mouse positions are stored in
            a	global renderParam and a global list respectively"""

            if mode == GLUT_DOWN:
                    self._mouseButton = button
            else:
                    self._mouseButton = None
            self._oldMousePos[0], self._oldMousePos[1] = x, y
            glutPostRedisplay( )

        def mouseMotionFunc( x, y ):
            """Callback function (mouse moved while button is pressed).

            The current and old mouse positions are stored in
            a	global renderParam and a global list respectively.
            The global translation vector is updated according to
            the movement of the mouse pointer."""
            dx = x - self._oldMousePos[ 0 ]
            dy = y - self._oldMousePos[ 1 ]

            if ( glutGetModifiers() == GLUT_ACTIVE_SHIFT and\
                   self._mouseButton == GLUT_LEFT_BUTTON  ):
                self.camera.moveBackForth(dy)

            elif self._mouseButton == GLUT_LEFT_BUTTON:
                self.camera.rotate(dx,dy)

            elif self._mouseButton == GLUT_RIGHT_BUTTON:
                self.camera.moveSideway(dx,dy)

            self._oldMousePos[0], self._oldMousePos[1] = x, y

            glutPostRedisplay( )

        glutMouseFunc( mouseButtonFunc )
        glutMotionFunc( mouseMotionFunc )
        glutSpecialFunc(keyPressedFunc)
        glutKeyboardFunc(keyPressedFunc)

