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
import os
import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GLX import *
from OpenGL.GL.EXT.framebuffer_object import *
import kinematic_chain
import ml_parser
import pickle
from openglaux import _is_extension_supported
from display_element import DisplayObject, DisplayRobot, GlPrimitive
import display_element
import re,imp
from camera import Camera
import pickle
config_dir = os.environ['HOME']+'/.robotviewer/'
import logging
import ConfigParser
import time, datetime
import subprocess
import code
ESCAPE = 27
import version
import copy
import threading
import math
import collections
import StringIO, socket
import distutils.version
import numpy
from mathaux import *
from ctypes import *
logger = logging.getLogger("robotviewer.displayserver")

try:
    from opencv import highgui, cv, adaptors
except ImportError:
    highgui = None
    adaptors = None
    cv = None
class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger.addHandler(NullHandler())


class CustomConfigParser(ConfigParser.ConfigParser):
    def get(self,*args, **kwargs ):
        try:
            return ConfigParser.ConfigParser.get(self, *args, **kwargs)
        except ConfigParser.NoOptionError:
            return None

class GlWindow(object):
    def __init__(self, title = "Robotviewer Server"):
        self.id = glutCreateWindow(title)
        self._g_dwLastFPS = 0
        self._fps = 0
        self._g_dwLastFPS = 0
        self._g_nFrames = 0
        self.title = title
        self.cameras = []
        self.active_camera = 0
        self.extra_info = ""

    @property
    def camera(self):
        return self.cameras[self.active_camera]

    def update_fps(self):
        """
        Arguments:
        - `self`:
        """
        milliseconds = glutGet( GLUT_ELAPSED_TIME )
        if (milliseconds - self._g_dwLastFPS >= 1000):
            # # When A Second Has Passed...
            # g_dwLastFPS = win32api.GetTickCount();
            # # Update Our Time Variable
            self._g_dwLastFPS = milliseconds
            self._fps = self._g_nFrames;
            # # Save The FPS
            self._g_nFrames = 0;
            # # Reset The FPS Counter
            glutReshapeWindow(self.camera.width, self.camera.height)
        szTitle = ""
        if self.id > 1:
            szTitle += "%d "%self.id
        szTitle += "%s "%self.title
        if self.camera.name:
            szTitle += "[%s] "%self.camera.name

        szTitle += "%sfps"%(self._fps)

        if self.extra_info:
            szTitle += self.extra_info

        glutSetWindowTitle ( szTitle );
        self._g_nFrames += 1

    def change_camera(self):
        if self.active_camera == len(self.cameras) - 1:
            self.active_camera = 0
        else:
            self.active_camera += 1
        glutReshapeWindow(self.camera.width, self.camera.height)
        self.camera.update_perspective()
        logger.info("Change camera to %d %s"%(self.active_camera,
                                              self.camera.name))

class DisplayServer(object):
    """OpenGL server
    """

    width = 640
    height = 480
    def __init__(self,options = None, args = None):
        """

        Arguments:
        """
        self.config_file = None
        self.no_cache = False
        self.use_vbo = False
        self.strict = False
        self.off_screen = False
        self.stream = None
        self.refresh_rate = None
        self.num_windows = 1
        self.run_once = False
        self.__dict__.update(options.__dict__)

        if not self.config_file:
            self.config_file = os.path.join(config_dir,"config")

        if self.use_vbo:
            display_element.USE_VBO = True

        self._element_dict = dict()

        self.fbo_id = -1
        self.rbo_id = -1

        self.video_fps = 30
        self.recording = False
        self.video_fn = None
        self.video_writer = None
        self.record_saved = False
        self.udp_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

        self.capture_images = []
        self.last_refreshed = {}

        self.modelAmbientLight = 0.3
        self.lightAttenuation = 0.2
        self.active_cameras = {}
        self.world_cameras = []

        logger.debug("Initializing OpenGL")
        self.init_gl()
        self.pendingObjects=[]
        self.parse_config()
        self.add_cameras()
        self._mouseButton = None
        self._oldMousePos = [ 0, 0 ]
        self.wired_frame_flag = False

        if options and options.skeleton:
            self.render_mesh_flag = False
            self.render_skeleton_flag = True
        else:
            self.render_mesh_flag = True
            self.render_skeleton_flag = False

        self.skeleton_size = 1
        self.transparency = 0

        # key interaction from console
        self.queued_keys = collections.deque()
        self.quit = False
        self.usage="Keyboard shortcuts:\n"
        for key, effect in [("q", "Quit the program"),
                            ("m", "Turn meshes on/off"),
                            ("s", "Turn skeletons on/off"),
                            ("w", "Turn wireframe on/off"),
                            ("+", "Skeleton size up"),
                            ("-", "Skeleton size down"),
                            ("l", "lighter scene"),
                            ("d", "dimmer scene"),
                            ("o", "light ATTENUATION down"),
                            ("e", "light ATTENUATION up"),
                            ("t", "transparency up"),
                            ("r", "transparency down"),
                            ("c", "screen capture"),
                            ("v", "start/stop video recording"),
                            ("x", "change camera"),
                            ]:

            self.usage += "%.20s: %s\n"%(key, effect)

    def add_cameras(self):
        for key, value in self._element_dict.items():
            if isinstance(value, DisplayObject):
                cameras = value.get_list(Camera)
                for id, window in self.windows.items():
                    window.cameras += cameras

    def __del__(self):
        if self.recording:
            self.stop_record()
        del self

    def enter_leave_cb(self, state):
        message = ""
        if state == GLUT_LEFT:
            message += "Left"
        else:
            message += "Entered"
        message += " window {0}".format(glutGetWindow())
        logger.debug(message)

    def visible_cb(self, status):
        logger.debug("visible: {0}".format(status == GLUT_VISIBLE))

    def create_window(self):
        logger.info("Creating window")
        intel_card = False
        if os.name == 'posix':
            rt = subprocess.call("lspci | grep VGA | grep Intel",
                                 shell= True)
            if rt == 0:
                intel_card = True
        # Hack to catch segfaut on intel cards
        if intel_card:
            dummy_win = glutCreateWindow("Initializing...")
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA
                            | GLUT_DEPTH)
        if intel_card:
            glutDestroyWindow(dummy_win)
        logger.debug("Setting glut WindowSize")
        glutInitWindowSize(self.width, self.height)
        logger.debug("Creating glutWindow")
        self.windows = {}
        # self.windows.append(glutCreateWindow("Robotviewer Server"))
        for i in range(self.num_windows):
            window = GlWindow()
            self.windows[window.id] = window
            glutEntryFunc(self.enter_leave_cb);
            glutVisibilityFunc(self.visible_cb);
            glutDisplayFunc(self.draw_cb)
            glutReshapeFunc(self.resize_cb)
            self.bind_events()
            glutPositionWindow(i*self.width,20);
            self.init_lights()
            cam = Camera()
            cam.width = self.width
            cam.height = self.height
            self._element_dict["camera%d"%window.id] = cam
            self.world_cameras.append(cam)
            window.cameras.append(cam)


        glutIdleFunc(self.refresh_cb)

    def init_gl(self):
        logger.debug("Initializing glut")
        glutInit(sys.argv)
        logger.debug("Setting glut DisplayMode")

        if not self.off_screen:
        #if True:
            self.create_window()
        else:
            import oglc
            oglc.create_osmesa_context()
            #self.create_window()
            #glutHideWindow(self.window)

            logger.info("Creating context")
            self.create_render_buffer()
            self.window = []


    # The function called when our window is resized (which shouldn't happen if
    # you enable fullscreen, below)
    def resize_cb(self, width, height):
        win = glutGetWindow()
        camera = self.windows[win].camera

        if not camera in self.world_cameras:
            glutReshapeWindow(camera.width, camera.height)
            return
        camera.width = width
        camera.height = height
        camera.update_perspective()


    def create_render_buffer(self):
        self.fbo_id = glGenFramebuffersEXT(1)
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, self.fbo_id)

        self.rbo_id = glGenRenderbuffersEXT(1)
        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, self.rbo_id)
        glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT,
                                 GL_RGB, self.width, self.height)
        glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,
                                     GL_COLOR_ATTACHMENT0_EXT,
                                     GL_RENDERBUFFER_EXT, self.rbo_id)

    def set_robot_joint_rank(self,robot_name, joint_rank_xml):
        """
        """
        if robot_name not in self._element_dict.keys():
            return False


        if not os.path.isfile(joint_rank_xml):
            return False

        pattern=re.compile(r"\s*<Link>\s*(\w+)\s*(\d+)\s*<\/Link>\s*")
        lines = open(joint_rank_xml).readlines()
        correct_joint_dict = dict()

        for line in lines:
            m = pattern.match(line)
            if m:
                correct_joint_dict[m.group(1)] = int(m.group(2)) -6
                logger.info( m.group(1)+ "\t" + m.group(2))

        for joint in self._element_dict[robot_name].obj.joint_list:
            if correct_joint_dict.has_key(joint.name):
                joint.id = correct_joint_dict[joint.name]

        self._element_dict[robot_name].obj.update_joint_dict()
        return True


    def _replace_env_var(self,s):
        matches = re.findall(r'\$(\w+)',s)
        for m in matches:
            var = m
            val = os.environ[var]
            s = s.replace(var,val)
        s = s.replace('$','')
        return s

    def parse_config(self):
        prog_version = distutils.version.StrictVersion(version.__version__)
        config = CustomConfigParser()
        config.read(self.config_file)
        logger.info( 'parsed_config %s'%config)

        if not config.has_section('global'):
            self.parse_configLegacy(config)
            return

        for section in config.sections():
            for options in config.options(section):
                value = self._replace_env_var(config.get(section,options))
                config.set(section, options, value)

        conf_version = distutils.version.StrictVersion(config.get('global','version'))
        if prog_version < conf_version:
            raise Exception("Your config version ({0}) is newer than program version ({1})".
                            format(conf_version, prog_version))

        value =  config.get('global','background')
        if value:
            value = [float(e) for e in value.split(",")]
            for win in self.windows:
                glutSetWindow(win)
                glClearColor (value[0], value[1], value[2], 0.5);

        sections = config.sections()
        join_pairs = []

        for section in sections:
            words = section.split()
            otype = words[0]

            if otype in ["robot", "object"]:
                oname = words[1]
                if not words[1:]:
                    raise Exception("All robots must have a name.")

                geometry = config.get(section, 'geometry')
                if not geometry:
                    raise Exception("missing geometry section for {0}"
                                    .format(section))
                scale = config.get(section, 'scale')
                if not scale:
                    scale = [1,1,1]

                joint_rank = config.get(section, 'joint_rank')
                if otype == "robot" and joint_rank:
                    self.set_robot_joint_rank( oname, joint_rank)

                position = config.get(section, 'position')
                if not position:
                    postion = 6*[0.0]
                else:
                    position = [float(e) for e in position.split()]

                self._create_element(otype, oname,
                                     geometry, scale)
                if position:
                    self.updateElementConfig(oname, position)
                self.enableElement(oname)
                parent = config.get(section, 'parent')

                if parent:
                    self.disableElement(oname)
                    parent_name = parent.split(",")[0]
                    parent_joint_ids = parent.split(",")[1:]

                    for parent_joint_id in parent_joint_ids:
                        if parent_joint_id != "":
                            parent_joint_id = int(parent_joint_id)
                        else:
                            parent_joint_id = None
                        name = "{0}_{1}_{2}".format(oname,parent_name,
                                                    parent_joint_id)
                        join_pairs.append((oname, name, parent_name,
                                           parent_joint_id))

        for pair in join_pairs:
            orig_name = pair[0]
            child_name = pair[1]
            parent_name = pair[2]
            parent_joint_id = pair[3]
            try:
                parent_obj = self._element_dict[parent_name]
            except KeyError:
                logger.warning("Parent {0} does not exist. Skipping".
                               format(parent_name))
                continue
            new_el = copy.deepcopy( self._element_dict[orig_name] )
            parent_obj.get_op_point(parent_joint_id).add_child(new_el)
            parent_obj.init()
            self._element_dict[child_name] = new_el
            self.enableElement(child_name)

        for name, obj in self._element_dict.items():
            obj.update()

    def parse_configLegacy(self, config):
        logger.warning("Entering legacy config parsing")
        for section in config.sections():
            if section not in ['robots','default_configs',
                               'objects','joint_rank',
                               'preferences','scales']:
                raise Exception("Invalid section {0} in {1}".
                                format(sec,self.config_file))

        scales = {}
        if config.has_section('scales'):
            for key, value in config.items('scales'):
                value = [float(e) for e in value.split(",")]
                scales[key] = value


        if config.has_section('robots'):
            robot_names = config.options('robots')
            for robot_name in robot_names:
                robot_config = config.get('robots',robot_name)
                robot_config = self._replace_env_var(robot_config)
                logger.info( 'robot_config=%s'%robot_config)
                if not os.path.isfile(robot_config):
                    logger.info( "WARNING: Couldn't load %s. Are you sure %s exists?"\
                        %(robot_name,robot_config))
                    continue
                self._create_element('robot',robot_name,robot_config,
                                     scales.get(robot_name))
                self.enableElement(robot_name)
        else:
            logger.info( """Couldn't any default robots.
            Loading an empty scene
            You might need to load some robots yourself.
            See documentation""")

        if config.has_section('joint_ranks'):
            robot_names = config.options('joint_ranks')
            for robot_name in robot_names:
                joint_rank_config = config.get('joint_ranks',robot_name)
                joint_rank_config = self._replace_env_var(joint_rank_config)
                if not self._element_dict.has_key(robot_name):
                    continue
                if not os.path.isfile(joint_rank_config):
                    continue
                self.set_robot_joint_rank(self, robot_name,joint_rank_config)

        if config.has_section('objects'):
            object_names = config.options('objects')
            for object_name in object_names:
                object_file = config.get('objects',object_name)
                object_file = self._replace_env_var(object_file)
                if not os.path.isfile(object_file):
                    logger.warning('Could not find %s'%object_file)
                    continue
                self._create_element('object', object_name,
                                     object_file, scales.get(object_name))
                self.enableElement(object_name)

        if config.has_section('default_configs'):
            object_names = config.options('default_configs')
            for object_name in object_names:
                pos = config.get('default_configs',object_name)
                pos = [float(e) for e in pos.split()]
                self.updateElementConfig(object_name,pos)

        if config.has_section('preferences'):
            for key, value in config.items('preferences'):
                if key == 'background':
                    value = [float(e) for e in value.split(",")]
                    for win in self.windows:
                        glutSetWindow(win)
                        glClearColor (value[0], value[1], value[2], 0.5);

        return


    def _create_element(self, etype, ename, epath, scale = None):
        """
        Same as createElement but will not be called by outside world
        (CORBA) show will always be in the GL thread
        Arguments:
        - `self`:
        - `etype`:        string, element type (e.g. robot, object)
        - `name`:         string, element name
        - `path`:  string, description  (e.g. wrl path)
        """
        logger.debug("Creating {0} {1} {2} {3}".format(etype, ename, epath, scale))
        if self._element_dict.has_key(ename):
            logger.exception("Element with that name exists already")
            return

        if etype == 'robot':
            objs = ml_parser.parse(epath, not self.no_cache)
            robots = []
            for obj in objs:
                if isinstance(obj, kinematic_chain.Robot):
                    robots.append(obj)
            if len(robots) != 1:
                raise Exception("file %s contains %d robots, expected 1."
                                %(epath, len(robots)))
            new_robot = robots[0]
            if scale:
                new_robot.scale(scale)
            new_element = DisplayRobot(new_robot)
            self._element_dict[ename] = new_element

        elif etype == 'object':
            new_element = None
            # try to load as vrml and script
            ext = os.path.splitext(epath)[1].replace(".","")
            if ext == "py":
                logger.debug("Creating element from python script file %s."%epath)
                new_element = GlPrimitive(script = open(epath).read())
            elif ext in ml_parser.supported_extensions:
                logger.debug("Creating element from supported markup language file %s."%epath)
                objs = ml_parser.parse(epath, not self.no_cache)
                objs = [ o for o in objs
                         if isinstance(o, kinematic_chain.GenericObject)]
                if len(objs) == 0:
                    raise Exception('Found no object in file %s'%epath)
                elif len(objs) == 1:
                    group = objs[0]
                else:
                    group  = kinematic_chain.GenericObject()
                    for obj in objs:
                        group.add_child(obj)
                        group.init()
                if scale:
                    group.scale(scale)
                new_element = DisplayObject(group)
            else:
                logger.debug("Creating element from raw script")
                new_element = GlPrimitive(script = epath)
            if not new_element:
                raise Exception("creation of element from {0} failed".format(epath))
            logger.debug("Adding %s to internal dictionay"%(new_element))
            self._element_dict[ename] = new_element

        else:
            raise TypeError,"Unknown element type %s"%etype


    def createElement(self, etype, ename, epath):
        """
        Same as _create_element but could be called by outside world
        (CORBA) show will always be in the GL thread
        Arguments:
        - `self`:
        - `etype`:        string, element type (e.g. robot, GLscript)
        - `name`:         string, element name
        - `path`:  string, path  (e.g. wrl path)
        """
        TIMEOUT = 600
        self.pendingObjects.append((etype, ename, epath))
        wait = 0
        while ename not in self._element_dict.keys() and wait < TIMEOUT:
            time.sleep(0.1)
            wait += 0.1
        if wait >= TIMEOUT:
            logger.exception("Object took too long to create")
            return False
        return True

    def destroyElement(self,name):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self._element_dict.has_key(name):
            logger.exception("Element with that name does not exist")
            return False

        del self._element_dict[name]
        return True

    def enableElement(self,name):
        """

        Arguments:
        - `self`:
        - `name`:
        """
        if not self._element_dict.has_key(name):
            return False

        self._element_dict[name].enabled = True
        return True

    def disableElement(self,name):
        """
        Arguments:
        - `self`:
        - `name`:
        """
        if not self._element_dict.has_key(name):
            return False

        self._element_dict[name].enabled = False
        return True

    def updateElementConfig(self,name,config):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self._element_dict.has_key(name):
            logger.exception("Element with that name does not exist")
            return False

        self._element_dict[name].update_config(config)
        return True

    def getElementConfig(self,name):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self._element_dict.has_key(name):
            logger.exception(KeyError,
                             "Element with that name does not exist")
            return []
        cfg = self._element_dict[name].get_config()
        if not isinstance(cfg, list):
            return []
        else:
            return cfg

    def listElements(self):
        return [name for name in self._element_dict.keys() ]

    def listElementDofs(self, ename):
        l = ["X", "Y", "Z", "roll", "pitch", "yaw"]
        if not isinstance( self._element_dict[ename], DisplayObject):
            return l

        obj = self._element_dict[ename].obj
        if not isinstance(obj, kinematic_chain.Robot):
            return l
        for j in obj.moving_joint_list:
            l += [j.name]
        return l



    def run(self):
        logger.info(self.usage)
        class InteractThread(threading.Thread):
            def __init__(self, app, *args, **kwargs):
                self.app = app
                threading.Thread.__init__(self, *args, **kwargs )

            def run(self):
                while not self.app.quit:
                    try:
                        self.interact()
                    except KeyboardInterrupt:
                        self.app.quit = True
            def interact(self):
                from getch import getch
                key = getch()
                print("\r")
                self.app.queued_keys.append(key)

        if self.off_screen:
            t = InteractThread(self)
            t.start()
        if not self.run_once:
            glutMainLoop()
        else:
            glutMainLoopEvent()


    def Ping(self):
        return "pong"

    def draw_cb(self, *arg):
        if self.quit:
            glutLeaveMainLoop()
            #glutDestroyWindow(self.window)
            return False
        try:
            while len(self.queued_keys) > 0:
                key = self.queued_keys.popleft()
                self.key_pressed_func(key)
            current_t = glutGet( GLUT_ELAPSED_TIME )
            win = glutGetWindow()
            if ((not self.last_refreshed.get(win))
                or (current_t - self.last_refreshed.get(win)
                    >= 1000/self.refresh_rate)):
                res = self._draw_cb()
                self.last_refreshed[win] = current_t
                return res
            else:
                return True
        except KeyboardInterrupt:
            glutLeaveMainLoop()
            return False

    def _draw_cb(self):
        #if glGetError() > 0:
        if len(self.pendingObjects) > 0:
            obj = self.pendingObjects.pop()
            logger.info( "creating %s %s %s"%( obj[0], obj[1], obj[2]))
            self._create_element(obj[0],obj[1],obj[2])
        # Clear Screen And Depth Buffer
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity ();
        # # Reset The Modelview Matrix
        # # Get FPS
        # milliseconds = win32api.GetTickCount()
        win = glutGetWindow()
        self.windows[win].camera.update_view()

        if not self.off_screen:
            self.windows[win].update_fps()
        for name,ele in self._element_dict.items():
            #    logger.info( item[0], item[1]._enabled)
            try:
                if isinstance(ele, DisplayRobot):
                    ele.render(self.render_mesh_flag,
                               self.render_skeleton_flag,
                               )
                else:
                    ele.render()
            except:
                if self.strict:
                    glutLeaveMainLoop()
                    #glutDestroyWindow(self.window)
                else:
                    logger.exception("Failed to render element {0}"
                                     .format(ele))

        if self.recording or self.stream:
            import PIL.Image
            win = glutGetWindow()
            pixels = glReadPixels(0,0,self.windows[win].camera.width ,
                                  self.windows[win].camera.height ,
                                  GL_RGB, GL_UNSIGNED_BYTE)
            img = (PIL.Image.fromstring("RGB",
                                        (self.windows[win].camera.width ,
                                         self.windows[win].camera.height)
                                        ,pixels).
                   transpose(PIL.Image.FLIP_TOP_BOTTOM))
            if self.recording:
                self.capture_images.append((time.time(),img))
            if self.stream:
                s = StringIO.StringIO()
                im.save(s, "PNG")
                self.udp_sock.sendto(s.getvalue(), "localhost:{0}".
                                     format(self.stream))

        glutSwapBuffers()
        return True

    def stop_record(self):
        for id, window in self.windows.items():
            window.extra_info = None
        self.recording = False
        self.record_saved = False

        if not self.capture_images[:]:
            logger.info("No images have been capture for video.")
            return

        im0 = self.capture_images[0][1]
        fourcc = highgui.CV_FOURCC('P','I','M','1')
        width = im0.size[0]
        height = im0.size[1]
        cvsize = cv.cvSize(width, height)
        self.video_writer = highgui.cvCreateVideoWriter(self.video_fn,
                                                        fourcc,
                                                        self.video_fps,
                                                        cvsize,
                                                        True)

        def flush():
            t0 = self.capture_images[0][0]
            frame_no = 0
            for t, im in self.capture_images:
                t -= t0
                cv_im = adaptors.PIL2Ipl(im)
                no_needed_frames = int(math.floor( t*self.video_fps) - frame_no)
                for j in range(no_needed_frames):
                    highgui.cvWriteFrame( self.video_writer, cv_im)
                    frame_no += 1
            highgui.cvReleaseVideoWriter(self.video_writer)
            logger.info("saved to {0}".format(self.video_fn))
            self.video_writer = None
            self.record_saved = True

        t = threading.Thread()
        t.run = flush
        t.start()

    def start_record(self):
        self.capture_images = []
        self.recording = True
        suffix = datetime.datetime.now().strftime("%Y%m%d%H%M")
        self.video_fn = None
        self.video_fn = "/tmp/robotviewer_{0}.avi".format(suffix)
        i = 0
        while not self.video_fn or os.path.isfile(self.video_fn):
            i += 1
            self.video_fn = ("/tmp/robotviewer_{0}_{1}.avi".
                               format(suffix, i))
        logger.info("recording to {0}".format(self.video_fn))
        for id, window in self.windows.items():
            window.extra_info = " (Recording to {0})".format(self.video_fn)


    def key_pressed_func(self, *args):
        # If escape is pressed, kill everything.
        if args[0] == "q" : # exit when ESCAPE is pressed
            self.quit = True
            glutLeaveMainLoop()


        elif args[0] == 'm':
            self.render_mesh_flag = not self.render_mesh_flag
            print "render mesh:", self.render_mesh_flag
        elif args[0] == 's':
            self.render_skeleton_flag = not self.render_skeleton_flag
            print "render skeleton:", self.render_skeleton_flag

        elif args[0] == 'w':
            self.wired_frame_flag = not self.wired_frame_flag
            print "render mesh:", self.wired_frame_flag
            if self.wired_frame_flag:
                glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
            else:
                glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

        elif args[0] == '+':
            self.skeleton_size += 1
            for name, obj in self._element_dict.items():
                if isinstance(obj, DisplayRobot):
                    obj.set_skeleton_size(self.skeleton_size)

        elif args[0] == '-':
            if self.skeleton_size >1:
                self.skeleton_size -= 1
            for name, obj in self._element_dict.items():
                if isinstance(obj, DisplayRobot):
                    obj.set_skeleton_size(self.skeleton_size)
        elif args[0] == 't':
            if self.transparency < 1:
                self.transparency += 0.1
                for name, e in self._element_dict.items():
                    if isinstance(e, DisplayRobot):
                        e.set_transparency(self.transparency)

        elif args[0] == 'r':
            if self.transparency > 0:
                self.transparency -= .1
                for name, e in self._element_dict.items():
                    if isinstance(e, DisplayRobot):
                        e.set_transparency(self.transparency)
        elif args[0] == 'l':
            if self.modelAmbientLight < 1.0:
                self.modelAmbientLight += 0.1
                glLightModelfv(GL_LIGHT_MODEL_AMBIENT,
                               [self.modelAmbientLight,
                                self.modelAmbientLight,
                                self.modelAmbientLight,
                                1])
        elif args[0] == 'd':
            if self.modelAmbientLight >0 :
                self.modelAmbientLight -= 0.1
                glLightModelfv(GL_LIGHT_MODEL_AMBIENT,
                               [self.modelAmbientLight,
                                self.modelAmbientLight,
                                self.modelAmbientLight,
                                1])

        elif args[0] == 'e':
            if self.lightAttenuation < 1.0:
                self.lightAttenuation += 0.1
                glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION,
                         self.lightAttenuation)
        elif args[0] == 'o':
            if self.lightAttenuation > 0.1 :
                self.lightAttenuation -= 0.1
                glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION,
                         self.lightAttenuation)

        elif args[0] == 'c':
            import PIL.Image
            win = glutGetWindow()
            pixels = glReadPixels(0,0,self.windows[win].camera.width ,
                                  self.windows[win].camera.height ,
                                  GL_RGB, GL_UNSIGNED_BYTE)
            im = (PIL.Image.fromstring("RGB",
                                       (self.windows[win].camera.width,
                                        self.windows[win].camera.height),
                                       pixels).
                  transpose(PIL.Image.FLIP_TOP_BOTTOM))
            imsuff = datetime.datetime.now().strftime("%Y%m%d%H%M")
            imname = None
            imname = "/tmp/robotviewer_{0}.png".format(imsuff)
            i = 0
            while not imname or os.path.isfile(imname):
                i += 1
                imname = "/tmp/robotviewer_{0}_{1}.png".format(imsuff, i)
            logger.info("Saved to {0}".format(imname))
            im.save(imname)
        elif args[0] == 'v':
            if not self.recording:
                self.start_record()
            else:
                self.stop_record()

        elif args[0] == 'x':
            win = glutGetWindow()
            self.windows[win].change_camera()
            glutPostRedisplay( )

    def mouse_button_func( self, button, mode, x, y ):
        """Callback function (mouse button pressed or released).

        The current and old mouse positions are stored in
        a	global renderParam and a global list respectively"""

        if mode == GLUT_DOWN:
            self._mouseButton = button
        else:
            self._mouseButton = None
        self._oldMousePos[0], self._oldMousePos[1] = x, y
        glutPostRedisplay( )



    def mouse_motion_func( self, x, y ):
        """Callback function (mouse moved while button is pressed).

        The current and old mouse positions are stored in
        a	global renderParam and a global list respectively.
        The global translation vector is updated according to
        the movement of the mouse pointer."""
        win = glutGetWindow()
        camera = self.windows[win].camera

        if not camera in self.world_cameras:
            return

        dx = x - self._oldMousePos[ 0 ]
        dy = y - self._oldMousePos[ 1 ]

        if ( glutGetModifiers() == GLUT_ACTIVE_SHIFT and\
               self._mouseButton == GLUT_LEFT_BUTTON  ):
            camera.move_back_forth(dy)

        elif self._mouseButton == GLUT_LEFT_BUTTON:
            camera.rotate(dx,dy)

        elif self._mouseButton == GLUT_RIGHT_BUTTON:
            camera.move_sideway(dx,dy)


        self._oldMousePos[0], self._oldMousePos[1] = x, y

        glutPostRedisplay( )

    def bind_events(self):
        glutMouseFunc( self.mouse_button_func )
        glutMotionFunc( self.mouse_motion_func )
        glutSpecialFunc(self.key_pressed_func)
        glutKeyboardFunc(self.key_pressed_func)

    def init_lights(self, bg = [0,0,0]):
        glClearColor (bg[0], bg[1], bg[2], 0.5);
        # # Black Background
        glClearDepth (1.0);
        # # Depth Buffer Setup
        glDepthFunc (GL_LEQUAL);
        # # The Type Of Depth Testing
        glEnable (GL_DEPTH_TEST);
        # # Enable Depth Testing
        glShadeModel (GL_SMOOTH);
        # # Select Smooth Shading
        glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        # # Set Perspective Calculations To Most Accurate
        glEnable(GL_TEXTURE_2D);
        # # Enable Texture Mapping
        glColor4f (1.0, 6.0, 6.0, 1.0)

        glClearColor(0.,0.,0.,1.)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_CULL_FACE)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

        glEnable (GL_BLEND)
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        lightZeroPosition = [-3.0,3.0,3.0,1.0]
        lightZeroColor = [1.0,1.0,1.0,1.0] #green tinged
        glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
        glLightfv(GL_LIGHT0, GL_SPECULAR, lightZeroColor)
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0,0,0,1])
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [self.modelAmbientLight,
                                                self.modelAmbientLight,
                                                self.modelAmbientLight,1])

        # glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, )
        glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, self.lightAttenuation)
        # glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.03)

        glEnable(GL_LIGHT0)


    def refresh_cb (self):
        for win in self.windows.keys():
            glutSetWindow(win)
            glutPostRedisplay()

