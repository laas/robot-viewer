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
from openglaux import IsExtensionSupported, GlWindow
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


RMASK = 0x00ff0000
GMASK = 0x0000ff00
BMASK = 0x000000ff
AMASK = 0xff000000
BPP = 32
DEPTH = 4

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

class CustomConfigParser(ConfigParser.ConfigParser):
    def get(self,*args, **kwargs ):
        try:
            return ConfigParser.ConfigParser.get(self, *args, **kwargs)
        except ConfigParser.NoOptionError:
            return None

class DisplayServer(object):
    """OpenGL server
    """

    def __init__(self,options = None, args = None):
        """

        Arguments:
        """
        if options and options.config_file:
            self.config_file = options.config_file
        else:
            self.config_file = os.path.join(config_dir,"config")

        self.no_cache = False
        if options and options.no_cache:
            self.no_cache = True

        self._element_dict = dict()

        if options:
            self.strict = options.strict

        self.off_screen = False
        if options and options.__dict__.has_key("off_screen"):
            self.off_screen = options.off_screen

        self.fbo_id = -1
        self.rbo_id = -1

        self.video_fps = 30
        self.recording = False
        self.video_fn = None
        self.video_writer = None
        self.record_saved = False

        self.capture_images = []
        self.refresh_rate = None
        self.last_refreshed = None
        if options:
            self.refresh_rate = options.refresh_rate

        self.win_w = 640
        self.win_h = 480
        logger.debug("Initializing OpenGL")
        self.initGL()
        self.pendingObjects=[]
        self.parseConfig()
        self.camera = Camera()
        updateView(self.camera)
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



    def __del__(self):
        if self.recording:
            self.stop_record()
        del self

    def create_window(self):
        logger.info("Creating window")
        intel_card = False
        if os.name == 'posix':
            rt = subprocess.call("lspci | grep VGA | grep Intel", shell= True)
            if rt == 0:
                intel_card = True
        # Hack to catch segfaut on intel cards
        if intel_card:
            dummy_win = glutCreateWindow("Initializing...")
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
        if intel_card:
            glutDestroyWindow(dummy_win)
        logger.debug("Setting glut WindowSize")
        glutInitWindowSize(self.win_w, self.win_h)
        glutInitWindowPosition(0, 0)
        logger.debug("Creating glutWindow")
        self.window = glutCreateWindow("Robotviewer Server")

    def initGL(self):
        logger.debug("Initializing glut")
        glutInit(sys.argv)
        logger.debug("Setting glut DisplayMode")

        if not self.off_screen:
        #if True:
            self.create_window()
        else:
            #import oglc
            #oglc.create_gl_context()
            self.create_window()
            glutHideWindow(self.window)

            logger.info("Creating context")
            self.create_render_buffer()
            self.window = None

        glutIdleFunc(self.DrawGLScene)
        glutReshapeFunc(self.ReSizeGLScene)
        self._glwin=GlWindow(self.win_w, self.win_h, "Robotviewer Server")
        self.usage = ""
        self.bindEvents()


    # The function called when our window is resized (which shouldn't happen if you
    # enable fullscreen, below)
    def ReSizeGLScene(self, Width, Height):
        self.win_w = Width
        self.win_h = Height
        if Height == 0:
            # Prevent A Divide By Zero If The Window Is Too Small
            Height = 1

        glViewport(0, 0, Width, Height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # # field of view, aspect ratio, near and far
        # This will squash and stretch our objects as the window is resized.
        gluPerspective(45.0, float(Width)/float(Height), 0.1, 1000.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()


    def create_render_buffer(self):
        self.fbo_id = glGenFramebuffersEXT(1)
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, self.fbo_id)

        self.rbo_id = glGenRenderbuffersEXT(1)
        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, self.rbo_id)
        glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGB, self.win_w, self.win_h)
        glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                                     GL_RENDERBUFFER_EXT, self.rbo_id)

    def setRobotJointRank(self,robot_name, joint_rank_xml):
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

    def parseConfig(self):
        prog_version = float(version.__version__)
        config = CustomConfigParser()
        config.read(self.config_file)
        logger.info( 'parsed_config %s'%config)

        if not config.has_section('global'):
            self.parseConfigLegacy(config)
            return

        for section in config.sections():
            for options in config.options(section):
                value = self._replace_env_var(config.get(section,options))
                config.set(section, options, value)

        conf_version = float(config.get('global','version'))
        if prog_version < conf_version:
            raise Exception("Your config version ({0}) is newer than program version ({1})".
                            format(conf_version, prog_version))

        value =  config.get('global','background')
        if value:
            value = [float(e) for e in value.split(",")]
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
                    raise Exception("missing geometry section for {0}".format(section))
                scale = config.get(section, 'scale')
                if not scale:
                    scale = [1,1,1]

                joint_rank = config.get(section, 'joint_rank')
                if otype == "robot" and joint_rank:
                    self.setRobotJointRank( oname, joint_rank)

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
            parent_obj = self._element_dict[parent_name]
            new_el = copy.deepcopy( self._element_dict[orig_name] )
            parent_obj.get_op_point(parent_joint_id).addChild(new_el)
            self._element_dict[child_name] = new_el
            self.enableElement(child_name)

        for name, obj in self._element_dict.items():
            obj.update()

    def parseConfigLegacy(self, config):
        logger.warning("Entering legacy config parsing")
        for section in config.sections():
            if section not in ['robots','default_configs','objects','joint_rank',
                               'preferences','scales']:
                raise Exception("Invalid section {0} in {1}".format(sec,self.config_file))

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
            logger.info( """Couldn't any default robots. Loading an empty scene
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
                self.setRobotJointRank(self, robot_name,joint_rank_config)

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
                objs = [ o for o in objs if isinstance(o, kinematic_chain.GenericObject)]
                if len(objs) == 0:
                    raise Exception('Found no object in file %s %d'%len(objs,epath))
                elif len(objs) == 1:
                    group = objs[0]
                else:
                    group  = kinematic_chain.GenericObject()
                    for obj in objs:
                        group.addChild(obj)
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
            logger.exception(KeyError,"Element with that name does not exist")
            return []
        return self._element_dict[name].get_config()

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
        glutMainLoop()


    def Ping(self):
        return "pong"

    def DrawGLScene(self, *arg):
        if self.quit:
            glutDestroyWindow(self.window)
            return False
        try:
            while len(self.queued_keys) > 0:
                key = self.queued_keys.popleft()
                self.keyPressedFunc(key)
            current_t = glutGet( GLUT_ELAPSED_TIME )
            if ((not self.last_refreshed)
                or current_t - self.last_refreshed >= 1000/self.refresh_rate):
                res = self._draw_gl_scene()
                self.last_refreshed = current_t
                return res
            else:
                return True
        except KeyboardInterrupt:
            glutDestroyWindow(self.window)
            return False

    def _draw_gl_scene(self):
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
        updateView(self.camera)


        if (not self.off_screen) and hasattr(self, '_glwin'):
            self._glwin.updateFPS()
            self._glwin._g_nFrames += 1
        for item in self._element_dict.items():
            ele = item[1]
            #    logger.info( item[0], item[1]._enabled)
            try:
                if isinstance(ele, DisplayRobot):
                    ele.render(self.render_mesh_flag,
                               self.render_skeleton_flag, self.skeleton_size)
                else:
                    ele.render()
            except:
                if self.strict:
                    glutDestroyWindow(self.window)
                else:
                    logger.exception("Failed to render element {0}".format(ele))

        glutSwapBuffers()

        if self.recording:
            import PIL.Image
            pixels = glReadPixels(0,0,self.win_w ,self.win_h ,GL_RGB, GL_UNSIGNED_BYTE)
            img = (PIL.Image.fromstring("RGB",(self.win_w ,self.win_h),pixels).
                   transpose(PIL.Image.FLIP_TOP_BOTTOM))
            self.capture_images.append((time.time(),img))

        # if self.off_screen:
        #     import caca
        #     from caca.canvas import Canvas, CanvasError
        #     from caca.dither import Dither, DitherError
        #     canvas = Canvas(100, 70)
        #     img = img.convert('RGBA')

        #     dit = Dither(BPP, img.size[0], img.size[1], DEPTH * img.size[0],
        #                  RMASK, GMASK, BMASK, AMASK)
        #     s = canvas.export_to_memory('ansi')
            # sys.stdout.write("%s" %s)
        return True

    def stop_record(self):
        self._glwin.extra_info = None
        self.recording = False
        self.record_saved = False
        if not self.capture_images[:]:
            logger.info("No images have been capture for video.")
            return
        def flush():
            t0 = self.capture_images[0][0]
            frame_no = 0
            for t, im in self.capture_images:
                t -= t0
                cv_im = adaptors.PIL2Ipl(im)
                no_needed_frames = math.floor( t*self.video_fps) - frame_no
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
        self.video_writer = highgui.cvCreateVideoWriter(self.video_fn,
                                                        highgui.CV_FOURCC('P','I',
                                                                          'M','1'),
                                                        self.video_fps,
                                                        cv.cvSize(self.win_w, self.win_h),
                                                        True)
        logger.info("recording to {0}".format(self.video_fn))
        self._glwin.extra_info = " (Recording to {0})".format(self.video_fn)





    def keyPressedFunc(self, *args):
        # If escape is pressed, kill everything.
        if args[0] == "q" : # exit when ESCAPE is pressed
            self.quit = True
            glutDestroyWindow(self.window)

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

        elif args[0] == '-':
            if self.skeleton_size >1:
                self.skeleton_size -= 1

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
            if self._glwin._modelAmbientLight < 1.0:
                self._glwin._modelAmbientLight += 0.1
                glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [self._glwin._modelAmbientLight,
                                                        self._glwin._modelAmbientLight,
                                                        self._glwin._modelAmbientLight,1])
        elif args[0] == 'd':
            if self._glwin._modelAmbientLight >0 :
                self._glwin._modelAmbientLight -= 0.1
                glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [self._glwin._modelAmbientLight,
                                                        self._glwin._modelAmbientLight,
                                                        self._glwin._modelAmbientLight,1])

        elif args[0] == 'e':
            if self._glwin._lightAttenuation < 1.0:
                self._glwin._lightAttenuation += 0.1
                glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, self._glwin._lightAttenuation)
        elif args[0] == 'o':
            if self._glwin._lightAttenuation > 0.1 :
                self._glwin._lightAttenuation -= 0.1
                glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, self._glwin._lightAttenuation)

        elif args[0] == 'c':
            import PIL.Image
            pixels = glReadPixels(0,0,self.win_w ,self.win_h ,GL_RGB, GL_UNSIGNED_BYTE)
            im = (PIL.Image.fromstring("RGB",(self.win_w ,self.win_h),pixels).
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


    def mouseButtonFunc( self, button, mode, x, y ):
        """Callback function (mouse button pressed or released).

        The current and old mouse positions are stored in
        a	global renderParam and a global list respectively"""

        if mode == GLUT_DOWN:
                self._mouseButton = button
        else:
                self._mouseButton = None
        self._oldMousePos[0], self._oldMousePos[1] = x, y
        glutPostRedisplay( )

    def mouseMotionFunc( self, x, y ):
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

    def bindEvents(self):
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
                            ("v", "start/stop video recording")
                            ]:

            self.usage += "%.20s: %s\n"%(key, effect)

        glutMouseFunc( self.mouseButtonFunc )
        glutMotionFunc( self.mouseMotionFunc )
        glutSpecialFunc(self.keyPressedFunc)
        glutKeyboardFunc(self.keyPressedFunc)

