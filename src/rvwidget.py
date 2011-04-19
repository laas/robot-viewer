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


from displayserver import *
from openglaux import *
from camera import *
import sys

import gobject
import pygtk
pygtk.require('2.0')
import gtk
import gtk.gtkgl
import logging
import logging.handlers

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("robotviewer.rvwidget")
logger.addHandler(NullHandler())

class RvWidget(DisplayServer, gtk.gtkgl.DrawingArea):
    """
    """

    def __init__(self):
        """
        """

        self.camera = Camera()
        self.mouseButtons = [None,None,None]  ## (left,right,middle)
        self.oldMousePos = None
        major, minor = gtk.gdkgl.query_version()
        logger.info( "GLX version = %d.%d" % (major, minor))

        #
        # frame buffer configuration
        #

        # use GLUT-style display mode bitmask
        try:
            # try double-buffered
            self.glconfig = gtk.gdkgl.Config(mode=(gtk.gdkgl.MODE_RGB    |
                                              gtk.gdkgl.MODE_DOUBLE |
                                              gtk.gdkgl.MODE_DEPTH))
        except gtk.gdkgl.NoMatches:
            # try single-buffered
            self.glconfig = gtk.gdkgl.Config(mode=(gtk.gdkgl.MODE_RGB    |
                                              gtk.gdkgl.MODE_DEPTH))

        # use GLX-style attribute list
        # try:
        #     # try double-buffered
        #     self.glconfig = gtk.gdkgl.Config(attrib_list=(gtk.gdkgl.RGBA,
        #                                              gtk.gdkgl.DOUBLEBUFFER,
        #                                              gtk.gdkgl.DEPTH_SIZE, 1))
        # except gtk.gdkgl.NoMatches:
        #     # try single-buffered
        #     self.glconfig = gtk.gdkgl.Config(attrib_list=(gtk.gdkgl.RGBA,
        #                                              gtk.gdkgl.DEPTH_SIZE, 1))
        logger.info( "self.glconfig.is_rgba() =",            self.glconfig.is_rgba())
        logger.info( "self.glconfig.is_double_buffered() =", self.glconfig.is_double_buffered())
        logger.info( "self.glconfig.has_depth_buffer() =",   self.glconfig.has_depth_buffer())

        # get_attrib()
        logger.info( "gtk.gdkgl.RGBA = %d"         % self.glconfig.get_attrib(gtk.gdkgl.RGBA))
        logger.info( "gtk.gdkgl.DOUBLEBUFFER = %d" % self.glconfig.get_attrib(gtk.gdkgl.DOUBLEBUFFER))
        logger.info( "gtk.gdkgl.DEPTH_SIZE = %d"   % self.glconfig.get_attrib(gtk.gdkgl.DEPTH_SIZE))
        gtk.gtkgl.DrawingArea.__init__(self, self.glconfig)
        self.set_size_request(300, 300)

        self.connect('configure_event', self.reshape)
        self.connect('expose_event', self.DrawGLScene)


        def idle(widget):
            try:
                widget.window.invalidate_rect(widget.allocation, False)
                # Update window synchronously (fast).
                widget.window.process_updates(False)
            except Exception,error:
                try:
                    logger.exception()
                except:
                    print error
            return True

        self.timeout_src_id = gobject.timeout_add(40,idle,self)


        self.add_events(gtk.gdk.BUTTON_PRESS_MASK | gtk.gdk.BUTTON_RELEASE_MASK \
                            | gtk.gdk.POINTER_MOTION_MASK )

        self.connect("button-press-event", self.button_press_cb)
        self.connect("button-release-event", self.button_release_cb)
        self.connect('motion-notify-event', self.motion_notify_cb)
        self.connect('scroll-event', self.scroll_event_cb)




    def scroll_event_cb(self,widget,event):
        if event.direction == gtk.gdk.SCROLL_UP:
            self.camera.moveBackForth(-100)

        if event.direction == gtk.gdk.SCROLL_DOWN:
            self.camera.moveBackForth(100)


    def motion_notify_cb(self,widget,event):
        if self.oldMousePos == None:
            self.oldMousePos = (event.x, event.y)
            return
        dx = event.x - self.oldMousePos[0]
        dy = event.y - self.oldMousePos[1]

        if self.mouseButtons[0] == 1:
            self.camera.rotate(dx,dy)
        elif self.mouseButtons[2] == 1:
            self.camera.moveSideway(dx,dy)

        self.oldMousePos = (event.x, event.y)

    def button_press_cb(self,widget,event):
        self.mouseButtons[event.button-1] = 1
        return True

    def button_release_cb(self,widget,event):
        self.mouseButtons[event.button-1] = None
        return True

    def reshape(self, widget , event):
        # get GLContext and GLDrawable
        glcontext = self.get_gl_context()
        gldrawable = self.get_gl_drawable()

        # GL calls
        if not gldrawable.gl_begin(glcontext): return

        x, y, width, height = self.get_allocation()
        updateView(self.camera)

        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        if width > height:
             w = float(width) / float(height)
             glFrustum(-w/20, w/20, -1.0/20, 1.0/20, .1, 10.0)
        else:
             h = float(height) / float(width)
             glFrustum(-1.0/20, 1.0/20, -h/20, h/20, .1, 10.0)

        # # field of view, aspect ratio, near and far
        # This will squash and stretch our objects as the window is resized.
        # gluPerspective(45.0, float(width)/float(height), 1, 1000.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        gldrawable.gl_end()

        return True

    def initGL(self):
        return

    def setLight(self):

        # Setup GL States
        glClearColor (0.0, 0.0, 0.0, 0.5);
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
        lightZeroPosition = [-3.0,3.0,3.0,1.0]
        lightZeroColor = [1.0,1.0,1.0,1.0] #green tinged
        glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)

        glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.0)
        glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.0)
        glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.03)

        glEnable(GL_LIGHT0)

    def finalInit(self):
        '''
        Final initialization, to be called when an GL context has been created
        '''
        if not ( IsExtensionSupported ("GL_ARB_vertex_buffer_object") and\
                     glInitVertexBufferObjectARB()   ):
            raise Exception('Help!  No VBO support')
        DisplayServer.__init__(self)
        self.setLight()
        self.connect('configure_event', self.reshape)
        self.connect('expose_event', self.DrawGLScene)
        return

    def DrawGLScene(self, widget ,event):
        glcontext = self.get_gl_context()
        gldrawable = self.get_gl_drawable()

        if not gldrawable.gl_begin(glcontext):
            return

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        updateView(self.camera)
        for item in self._element_dict.items():
            ele = item[1]
            ele.render()

        if gldrawable.is_double_buffered():
            gldrawable.swap_buffers()
        else:
            glFlush()

        gldrawable.gl_end()

        return True
