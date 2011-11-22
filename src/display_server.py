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
# OpenGL.FORWARD_COMPATIBLE_ONLY = True
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GLX import *
from OpenGL.GL.EXT.framebuffer_object import *
import kinematics
import ml_parser
import pickle
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
import mathaux
from ctypes import *
from kinematic_server import KinematicServer
from shaders import *
from shader import Shader
import camera

try:
    from opencv import highgui, cv, adaptors
except ImportError:
    highgui = None
    adaptors = None
    cv = None

logger = logging.getLogger("robotviewer.display_server")
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
        logger.debug("Change camera to %d %s.\n%s"%(self.active_camera,
                                                   self.camera.name,
                                                   self.camera))

class DisplayServer(KinematicServer):
    """OpenGL server
    """
    lightZeroPosition = [5.0,4.0,5.0]
    lightZeroSpecularColor = [.8, .8, .8]
    lightZeroDiffuseColor = [.8, .8, .8]
    lightAmbient = [.1, .1, .1]
    width = 640
    height = 480
    cameras = []
    config_file = None
    no_cache = False
    use_vbo = False
    strict = False
    refresh_rate = None
    num_windows = 1
    run_once = False
    modern_shader = display_element.MODERN_SHADER
    use_shader = True
    pending_configs = {}
    pending_configs2 = {}
    last_update = 0

    def __init__(self,options = None, args = None):
        """

        Arguments:
        """
        if options:
            self.__dict__.update(options.__dict__)

        if not self.config_file:
            self.config_file = os.path.join(config_dir,"config")

        if self.use_vbo:
            display_element.USE_VBO = True

        self.display_elements = dict()
        self.windows = {}

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

        self.lightAttenuation = 0.01
        self.active_cameras = {}
        self.world_cameras = []
        KinematicServer.__init__(self, options, args)
        logger.debug("Initializing OpenGL")
        self.init_gl()
        self._mouseButton = None
        self._oldMousePos = [ 0, 0 ]
        self.wired_frame_flag = False
        self.parse_config()
        if options and options.skeleton:
            self.render_shape_flag = False
            self.render_skeleton_flag = True
        else:
            self.render_shape_flag = True
            self.render_skeleton_flag = False

        self.skeleton_size = 2
        self.transparency = 0
        self.add_cameras()

        # finish up and print instructions
        # key interaction from console
        self.queued_keys = collections.deque()
        self.quit = False
        self.usage="Keyboard shortcuts:\n"
        for key, effect in [("q", "Quit the program"),
                            ("m", "Turn shapes on/off"),
                            ("s", "Turn skeletons on/off"),
                            ("w", "Turn wireframe on/off"),
                            ("p", "Turn specular highlights on/off"),
                            ("[", "Toggle legacy shader on/off"),
                            ("+", "Skeleton size up"),
                            ("-", "Skeleton size down"),
                            # ("l", "lighter scene"),
                            # ("d", "dimmer scene"),
                            # ("o", "light ATTENUATION down"),
                            # ("e", "light ATTENUATION up"),
                            # ("t", "transparency up"),
                            # ("r", "transparency down"),
                            ("c", "screen capture"),
                            ("v", "start/stop video recording"),
                            ("x", "change camera"),
                            ]:

            self.usage += "%.20s: %s\n"%(key, effect)

    def add_cameras(self):
        for key, value in self.display_elements.items():
            if isinstance(value, DisplayObject):
                cameras = value.get_list(Camera)
                print key, cameras
                self.cameras += cameras

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


    def dummy_cb(self):
        return

    def create_window(self):
        logger.info("Creating window")
        dummy_win = glutCreateWindow("Initializing...")
        glutDisplayFunc(self.dummy_cb)
        glutHideWindow(dummy_win)

        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA
                            | GLUT_DEPTH)
        glutDestroyWindow(dummy_win)
        logger.debug("Setting glut WindowSize")
        glutInitWindowSize(self.width, self.height)
        logger.debug("Creating glutWindow")
        self.windows = {}
        # self.windows.append(glutCreateWindow("Robotviewer Server"))
        self.shaders = display_element.shaders
        for i in range(self.num_windows):
            window = GlWindow()
            glutDisplayFunc(self.draw_cb)
            glutReshapeFunc(self.resize_cb)
            self.windows[window.id] = window
            glutEntryFunc(self.enter_leave_cb);
            glutVisibilityFunc(self.visible_cb);
            self.bind_events()
            glutPositionWindow(i*self.width,20);
            self.init_lights()
            cam = Camera()
            cam.translation = [3.5, 0, 1]
            cam.width = self.width
            cam.height = self.height
            cam_name = "world_camera%d"%window.id
            cam.name = cam_name
            cam.init()
            self.display_elements[cam_name] = cam
            self.world_cameras.append(cam)
            self.cameras.append(cam)
            window.active_camera = len(self.world_cameras) -1
            window.cameras = self.cameras
            print "use_shader=", self.use_shader
            if not self.use_shader:
                continue

            p = os.path.abspath(os.path.dirname(__file__))
            vs_text = open(os.path.join(p,'vertex_shader.c')).read()
            fs_text = open(os.path.join(p,'fragment_shader.c')).read()
            program = compileProgram(compileShader(vs_text, GL_VERTEX_SHADER),
                                              compileShader(fs_text, GL_FRAGMENT_SHADER),
                                              )

            shader = Shader(program)
            glUseProgram(program)
            self.specular_highlights = True
            shader.uShowSpecularHighlights = self.specular_highlights

            shader.uAmbientLightingColor        = self.lightAmbient
            shader.uPointLightingLocation       = self.lightZeroPosition
            shader.uPointLightingDiffuseColor   = self.lightZeroDiffuseColor
            shader.uPointLightingSpecularColor  = self.lightZeroSpecularColor
            shader.uMaterialShininess = 1.0
            shader.uModernShader = self.modern_shader
            self.shaders[glutGetWindow()] = shader

        glutIdleFunc(self.refresh_cb)

    def init_gl(self):
        logger.debug("Initializing glut")
        glutInit(sys.argv)
        logger.debug("Setting glut DisplayMode")

        self.create_window()

    # The function called when our window is resized (which shouldn't happen if
    # you enable fullscreen, below)
    def resize_cb(self, width, height):
        win = glutGetWindow()
        camera = self.windows[win].camera

        if not camera.name.startswith('world'):
            return

        if not camera in self.world_cameras:
            glutReshapeWindow(camera.width, camera.height)
            return
        camera.width = width
        camera.height = height
        #camera.compute_opencv_params()
        camera.aspect = 1.0*camera.width/camera.height
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
        if robot_name not in self.display_elements.keys():
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
                logger.debug( m.group(1)+ "\t" + m.group(2))

        for joint in self.display_elements[robot_name].obj.joint_list:
            if correct_joint_dict.has_key(joint.name):
                joint.id = correct_joint_dict[joint.name]

        self.display_elements[robot_name].obj.update_joint_dict()
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
        KinematicServer.parse_config(self)
        bg = self.global_configs.get('background')
        if bg:
            for win in self.windows:
                glutSetWindow(win)
                glClearColor (bg[0], bg[1], bg[2], 0.5);


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
        if self.display_elements.has_key(ename):
            logger.exception("%s Element with that name exists already"%ename)
            return

        if etype == 'robot':
            objs = ml_parser.parse(epath, not self.no_cache)
            robots = []
            for obj in objs:
                if isinstance(obj, kinematics.Robot):
                    robots.append(obj)
            if len(robots) != 1:
                raise Exception("file %s contains %d robots, expected 1."
                                %(epath, len(robots)))
            new_robot = robots[0]

            new_element = DisplayRobot(new_robot)
            self.display_elements[ename] = new_element
            self.kinematic_elements[ename] = new_robot
            for cam in new_robot.get_list(camera.Camera):
                self.kinematic_elements[cam.name] = cam


        elif etype == 'object':
            new_element = None
            # try to load as vrml and script
            ext = os.path.splitext(epath)[1].replace(".","")
            if ext == "py":
                logger.debug("Creating element from python script file %s."%epath)
                new_element = GlPrimitive(script = open(epath).read())
                new_object = new_element
            elif ext in ml_parser.supported_extensions:
                logger.debug("Creating element from supported markup language file %s."%epath)
                objs = ml_parser.parse(epath, not self.no_cache)
                objs = [ o for o in objs
                         if isinstance(o, kinematics.GenericObject)]
                if len(objs) == 0:
                    raise Exception('Found no object in file %s'%epath)
                elif len(objs) == 1:
                    group = objs[0]
                else:
                    group  = kinematics.GenericObject()
                    for obj in objs:
                        group.add_child(obj)
                        group.init()
                if scale:
                    group.scale = scale
                new_object = group
                new_element = DisplayObject(group)
            else:
                logger.debug("Creating element from raw script")
                new_element = GlPrimitive(script = epath)
                new_object = new_element
            if not new_element:
                raise Exception("creation of element from {0} failed".format(epath))
            logger.debug("Adding %s to internal dictionay"%(new_element))
            self.display_elements[ename] = new_element
            self.kinematic_elements[ename] = new_object


        else:
            raise TypeError,"Unknown element type %s"%etype



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

        if not (self.run_once):
            glutMainLoop()
        elif self.run_once:
            glutMainLoopEvent()


    def Ping(self):
        return "pong"

    def post_draw(self):
        return

    def draw_cb(self, *arg):
        now = time.time()
        if now - self.last_update > 0.04:
            self.last_update = now
            for key, value in self.pending_configs.items():
                if value == None:
                    continue
                KinematicServer.updateElementConfig(self, key, value)
                self.pending_configs[key] = None


            for key, value in self.pending_configs2.items():
                if value == None:
                    continue
                KinematicServer.updateElementConfig2(self, key, *value)
                self.pending_configs2[key] = None

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
            logger.debug( "creating %s %s %s"%( obj[0], obj[1], obj[2]))
            self._create_element(obj[0],obj[1],obj[2])

        for name, obj in self.display_elements.items():
            if "pending_display_change" not in obj.__dict__.keys():
                continue
            if obj.pending_display_change:
                obj.pending_display_change = True
                for shape in obj.shape_list:
                    shape.gl_primitive.generate_gl_list()

        # Clear Screen And Depth Buffer
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity ();
        # # Reset The Modelview Matrix
        # # Get FPS
        # milliseconds = win32api.GetTickCount()
        win = glutGetWindow()
        self.windows[win].camera.update_view()

        self.windows[win].update_fps()
        for name,ele in self.display_elements.items():
            #    logger.info( item[0], item[1]._enabled)
            try:
                if isinstance(ele, DisplayRobot):
                    ele.render(self.render_shape_flag,
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

        glutSwapBuffers()
        pm = glReadPixels(0,0,self.windows[win].camera.width ,
                                                       self.windows[win].camera.height ,
                                                       GL_RGB, GL_UNSIGNED_BYTE)
        #pm = YFlipPixmap(pm)
        self.windows[win].camera.pixels = pm
        self.windows[win].camera.draw_t = time.time()
        self.windows[win].camera.frame_seq += 1

        if self.recording:
            import PIL.Image
            win = glutGetWindow()
            pixels = self.windows[win].camera.pixels
            img = (PIL.Image.fromstring("RGB",
                                        (self.windows[win].camera.width ,
                                         self.windows[win].camera.height)
                                        ,pixels).
                   transpose(PIL.Image.FLIP_TOP_BOTTOM))
            if self.recording:
                self.capture_images.append((time.time(),img))

        self.post_draw()

        if self.run_once:
            self.screen_capture()

        return True

    def screen_capture(self):
        import PIL.Image
        win = glutGetWindow()
        pixels = self.windows[win].camera.pixels
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

    def stop_record(self):
        for id, window in self.windows.items():
            window.extra_info = None
        self.recording = False
        self.record_saved = False

        if not self.capture_images[:]:
            logger.info("No images have been capture for video.")
            return

        im0 = self.capture_images[0][1]
        fourcc = highgui.CV_FOURCC('D','I','V','X')
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
            self.render_shape_flag = not self.render_shape_flag
            print "render shape:", self.render_shape_flag
        elif args[0] == 's':
            self.render_skeleton_flag = not self.render_skeleton_flag
            print "render skeleton:", self.render_skeleton_flag

        elif args[0] == 'p' and self.use_shader:
            self.specular_highlights = not self.specular_highlights
            for id in self.windows:
                glutSetWindow(id)
                self.shaders[id].uShowSpecularHighlights = self.specular_highlights

        elif args[0] == '[' and self.use_shader:
            self.modern_shader = not self.modern_shader
            display_element.MODERN_SHADER = self.modern_shader
            if self.modern_shader:
                legacy = "OFF"
            else:
                legacy = "ON"
            print "Legacy shader is ", legacy
            for id in self.windows:
                glutSetWindow(id)
                self.shaders[id].uModernShader = self.modern_shader


        elif args[0] == 'w':
            self.wired_frame_flag = not self.wired_frame_flag
            print "render shape:", self.wired_frame_flag
            if self.wired_frame_flag:
                glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
            else:
                glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

        elif args[0] == '+':
            self.skeleton_size += 1
            for name, obj in self.display_elements.items():
                if isinstance(obj, DisplayRobot):
                    obj.set_skeleton_size(self.skeleton_size)

        elif args[0] == '-':
            if self.skeleton_size >1:
                self.skeleton_size -= 1
            for name, obj in self.display_elements.items():
                if isinstance(obj, DisplayRobot):
                    obj.set_skeleton_size(self.skeleton_size)

        elif args[0] == 't':
            if self.transparency < 1:
                self.transparency += 0.05
                for name, e in self.display_elements.items():
                    if isinstance(e, DisplayRobot):
                        e.set_transparency(self.transparency)

        elif args[0] == 'r':
            if self.transparency > 0:
                self.transparency -= .05
                for name, e in self.display_elements.items():
                    if isinstance(e, DisplayRobot):
                        e.set_transparency(self.transparency)
        elif args[0] == 'l':
            if self.lightAmbient < 1.0:
                self.lightAmbient += 0.05
                glLightModelfv(GL_LIGHT_MODEL_AMBIENT,
                               [self.lightAmbient,
                                self.lightAmbient,
                                self.lightAmbient,
                                1])
            print "lightAmbient = ", self.lightAmbient


        elif args[0] == 'd':
            if self.lightAmbient >0 :
                self.lightAmbient -= 0.05
                glLightModelfv(GL_LIGHT_MODEL_AMBIENT,
                               [self.lightAmbient,
                                self.lightAmbient,
                                self.lightAmbient,
                                1])
            print "lightAmbient = ", self.lightAmbient


        elif args[0] == 'e':
            if self.lightAttenuation < 1.0:
                self.lightAttenuation += 0.01
                glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION,
                         self.lightAttenuation)
            print "lightAttenuation = ", self.lightAttenuation


        elif args[0] == 'o':
            if self.lightAttenuation > 0.01 :
                self.lightAttenuation -= 0.01
                glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION,
                         self.lightAttenuation)
            print "lightAttenuation = ", self.lightAttenuation


        elif args[0] == 'c':
            self.screen_capture()

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

        glLightfv(GL_LIGHT0, GL_POSITION, self.lightZeroPosition)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, self.lightZeroDiffuseColor + [1.])
        glLightfv(GL_LIGHT0, GL_SPECULAR, self.lightZeroSpecularColor + [1.])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0,0,0,1])
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, self.lightAmbient + [1.])

        #glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, self.)
        glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, self.lightAttenuation)
        #glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.03)

        glEnable(GL_LIGHT0)


    def refresh_cb (self):
        for win in self.windows.keys():
            glutSetWindow(win)
            glutPostRedisplay()



    def enableElement(self,name):
        """

        Arguments:
        - `self`:
        - `name`:
        """
        if not self.display_elements.has_key(name):
            return False

        self.kinematic_elements[name].enabled = True
        return True

    def disableElement(self,name):
        """
        Arguments:
        - `self`:
        - `name`:
        """

        if not self.display_elements.has_key(name):
            return False

        self.display_elements[name].enabled = False
        return True


    def setTransparency(self,name, t):
        """
        Arguments:
        - `self`:
        - `name`:
        """
        if name not in self.display_elements.keys():
            return False

        if t < 0 or t > 1:
            return False

        self.display_elements[name].set_transparency(t)
        return True


    def listCameras(self ):
        return [str(cam.name) for cam in self.cameras]

    def getCameraConfig(self, cam_name):
        cam_dict = dict((cam.name,cam) for cam in self.cameras)
        cam = cam_dict.get(cam_name)
        if not cam:
            return []
        return cam.get_config()

    def getCameraInfo(self, cam_name):
        cam_dict = dict((cam.name,cam) for cam in self.cameras)
        cam = cam_dict.get(cam_name)
        if not cam:
            return ""
        return str(cam)

    def setCameraOpenCVParams(self,cam_name, width, height,
                          fx, fy, cx, cy):
        cam_dict = dict((cam.name,cam) for cam in self.cameras)
        cam = cam_dict.get(cam_name)
        if not cam:
            return ""
        cam.set_opencv_params(width, height, fx, fy, cx, cy)

    def updateElementConfig(self, name, config):
        self.pending_configs[name] = config
        return True

    def updateElementConfig2(self, name, T, q):
        self.pending_configs2[name] = T,q
        return True
