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
from __future__ import division
import kinematics
import math
import numpy, numpy.linalg
import transformations as tf
import copy
import alias
from ctypes import *
from OpenGL.GL.ARB.framebuffer_object import *
from OpenGL.GL.EXT.framebuffer_object import *
import PIL.Image
def norm(a):
    return sqrt(numpy.dot(a,a))

from display_element import DisplayRobot

import logging
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("robotviewer.camera")
logger.addHandler(NullHandler())

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    logger.exception("Could not import OpenGL modules.")

OVERHEAD_THRESHOLD = 1 - 1e-3

class Camera(kinematics.GenericObject, alias.Aliaser):
    '''
    World camera and robot cameras. Use OpenHRP convention
    http://openrtp.info/openhrp3/en/create_model.html#VISIONSENSOR
    '''
    # http://www.felixgers.de/teaching/jogl/perspectiveProjection.html

    # OPENGL params
    frontClipDistance = 0.01
    backClipDistance = 100
    width = 320
    height = 240
    cam_type = "COLOR"
    fieldOfView = 45*math.pi/180
    translation = [0, 0, 0]
    focal = 3.5
    x0 = 0
    y0 = 0
    aspect = 320.0/240.0

    # OpenCV params
    fx = 1.
    fy = 1.
    cx = 160 # (width/2)
    cy = 120 # (height/2)


    def __init__(self, server = None):
        self.pixels = None
        self.draw_t = 0
        self.frame_seq = 0
        self.server = server
        self.frontClipDistance = 0.01
        self.backClipDistance = 100
        self.width = 320
        self.height = 240
        self.cam_type = "COLOR"
        self.fieldOfView = 45*math.pi/180
        self.translation = [0, 0, 0]
        self.focal = 3.5
        self.x0 = 1.
        self.y0 = 1.
        self.aspect = 320.0/240.0

        # OpenCV params
        self.fx = 0.
        self.fy = 0.
        self.cx = 160 # (width/2)
        self.cy = 120 # (height/2)
        self.R = numpy.eye(3)

        kinematics.GenericObject.__init__(self)
        self.localTransformation[:3,:3] = numpy.array([ [ 0 , 0 , 1],
                                                        [ 1 , 0 , 0],
                                                        [ 0 , 1 , 0],
                                                        ]
                                  )
        angle, direction, point = tf.rotation_from_matrix(self.localTransformation)
        self.rotation = list(direction) + [angle]
        self.moved = True
        self.lookat = None
        self.init()

        self.top    =  math.atan(self.fovy/2)*self.near
        self.bottom = -math.atan(self.fovy/2)*self.near

        self.right  = self.aspect*self.top
        self.left   = self.aspect*self.bottom

        self.compute_opencv_params()
        # print self.globalTransformation
        self.count = 0
        self.fbo = None
        self.texture = None
        self.render_buffer = None
        self.gl_error = None
        logger.info("{0} GL vals: {1}".format(self.name,
                                              (self.x0, self.y0,
                                               self.fovy, self.aspect,
                                               self.left, self.right,
                                               self.bottom, self.top,
                                               self.near, self.far)))

    class Alias:
        frontClipDistance = ('Near','near')
        backClipDistance  = ('Far','far')
        fieldOfView       = ('fovy',)
        cx                = ('u0')
        cy                = ('v0')

    @property
    def K(self):
        return numpy.array([ [ self.fx , 0 , self.cx],
                             [ 0 , self.fy , self.cy],
                             [ 0 , 0 , 1],
                             ]
                           )

    @property
    def P(self):
        return numpy.array([ [ self.fx , 0 , self.cx, 0],
                             [ 0 , self.fy , self.cy, 0],
                             [ 0 , 0 , 1, 0],
                             ]
                           )

    @property
    def cam_up(self):
        return self.globalTransformation[:3,1]

    @property
    def cam_position(self):
        return self.globalTransformation[:3,3]
    @property
    def cam_ray(self):
        return self.globalTransformation[:3,2]
    @property
    def cam_right(self):
        return - self.globalTransformation[:3,0]


    def compute_opengl_params(self):
        '''
        Compute gl params from opencv params
        '''
        self.x0= int(self.u0  - self.width/2.0)
        self.y0 = int(self.height/2.0 - self.v0)

        if int(self.cx) == self.width/2 and int(self.cy) == self.height:
            self.fovy = 2*math.atan(1.0*self.height/(2*self.fy))
            self.aspect = 1.0*self.width/self.height*self.fy/self.fx
        else: # excentric camera, fo                                 vy and aspect has no physical sense
            self.fovy = None
            self.aspect = None

        self.right   = self.near/self.fx*( self.width - self.cx )
        self.left    = self.near/self.fx*( -self.cx )
        self.top     = self.near/self.fy*( self.cy)
        self.bottom  = self.near/self.fy*( self.cy - self.height)
        # print self.cx, self.width, self.right, self.left
        # print("compute_opengl_params {0} GL vals: {1}\n".format(self.name,
        #                                       (self.x0, self.y0,
        #                                        self.fovy, self.aspect,
        #                                        self.left, self.right,
        #                                        self.bottom, self.top,
        #                                        self.near, self.far)))


    def compute_opencv_params(self):
        '''
        Compute gl params from opencv params
        '''
        self.cx = self.width/2.0
        self.cy = self.height/2.0
        self.u0 = int(self.x0 + self.width/2.0 )
        self.v0 = int(self.y0 + self.height/2.0)
        # print 'u0=', self.u0, ', cx=', self.cx
        self.fy = 1.0*self.height/math.tan(self.fovy/2.0)
        self.fx = 1.0*self.width/self.height*self.fy/self.aspect
        self.update_perspective()


    def set_opencv_params(self, width, height, fx, fy, cx, cy):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.compute_opengl_params()



    def simulate(self):
        if not self.texture:
            self.texture=glGenTextures( 1 )

        glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                         GL_REPEAT);
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
                         GL_REPEAT );
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, self.width, self.height, 0,GL_RGBA,
                     GL_UNSIGNED_INT, None)
        glBindTexture( GL_TEXTURE_2D, self.texture );


        # occupy width x height texture memory
        # This is the interesting part: render-to-texture is initialized here
        # generate a "Framebuffer" object and bind it
        # render to the texture
        # In case of errors or suspect behaviour, try this:
        #    print "Framebuffer Status:"
        #    print glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
        # Save Viewport configuration

        if not self.render_buffer:
            self.render_buffer = glGenRenderbuffersEXT(1)
        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, self.render_buffer)
        glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT,
                                 self.width, self.height);
        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);


        if not self.fbo:
            self.fbo = c_uint(1)
            glGenFramebuffers(1,self.fbo)
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, self.fbo);


        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                                 GL_TEXTURE_2D, self.texture, 0);
        glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
                             GL_RENDERBUFFER_EXT, self.render_buffer)

        if self.draw():
            self.pixels = glReadPixels(0,0, self.width, self.height, GL_RGB, GL_UNSIGNED_BYTE)

        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

        return

    def render(self):
        pass

    def draw(self):
        if self.server == None:
            return False

        status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
        if(status != GL_FRAMEBUFFER_COMPLETE_EXT):
            logger.warning("Framebuffer not ready.")
            return False

        #if self.server.gl_error != None:
        #    return
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity ();
        self.update_perspective()
        self.update_view()

        for name,ele in self.server.display_elements.items():
            #    logger.info( item[0], item[1]._enabled)
            if isinstance(ele, DisplayRobot):
                ele.render(self.server.render_shape_flag,
                           self.server.render_skeleton_flag,
                           )
            else:
                ele.render()

        return True

    def update(self):
        kinematics.GenericObject.update(self)
        lookatRel = numpy.eye(4)
        lookatRel[2,3] = - self.focal
        lookatT = numpy.dot(self.globalTransformation,
                            lookatRel)
        self.lookat = lookatT[:3,3]

    def rotate(self,dx,dy):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        factor = 0.002
        dup    = dy*factor
        dright = dx*factor*abs(self.cam_up[2])

        if ((abs(self.cam_ray/numpy.linalg.norm(self.cam_ray))[2] ) > OVERHEAD_THRESHOLD
            and dup*self.cam_ray[2] > 0 ):
            dup = 0

        new_z = self.cam_ray + dup*self.cam_up + dright*self.cam_right
        new_pos = self.lookat + new_z*self.focal

        # trans = new_pos - self.cam_position
        v = (numpy.cross(new_z, numpy.array([0,0,1])))
        v = v/numpy.linalg.norm(v)
        new_x = - v
        new_y = numpy.cross(new_z, new_x)
        new_R = numpy.eye(4)

        new_R[:3,0] =  new_x
        new_R[:3,1] =  new_y
        new_R[:3,2] =  new_z

        rpy = tf.euler_from_matrix(new_R)
        config = self.get_config()
        config[0] = new_pos[0]
        config[1] = new_pos[1]
        config[2] = new_pos[2]
        #config[3] = rpy[0]
        #config[4] = rpy[1]
        #config[5] = rpy[2]

        self.translation  = new_pos
        self.localTransformation = new_R
        self.localTransformation[:3,3] = new_pos

        #self.update()
        #print self.globalTransformation
        #print self.cam_position
        #print self.lookat
        self.update()
        return


    def move_sideway(self,dx,dy):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        factor = 0.01
        dup    = dy*factor
        dright = dx*factor

        d = self.cam_right*dright + self.cam_up*dup
        for i in range(3):
            self.translation[i] += d[i]
        self.globalTransformation[:3,3] =  self.translation
        self.update()


    def move_back_forth(self,dy):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        if self.focal < 0.01 and dy <=0:
            return
        factor = 0.02*0.1
        d = self.cam_ray*dy*factor
        self.focal += dy*factor
        for i in range(3):
            self.translation[i] += d[i]
        self.globalTransformation[:3,3] =  self.translation
        self.update()


    def update_view(self):
        """

        Arguments:
        - `camera`:
        """
        p = self.cam_position
        f = self.lookat
        u = self.cam_up
        glLoadIdentity()
        gluLookAt(p[0],p[1],p[2],f[0],f[1],f[2],u[0],u[1],u[2])

    def update_perspective(self):
        glViewport(0, 0, self.width, self.height)
        #glViewport(self.x0, self.y0, self.width, self.height)
        #glViewport(0, 0, self.width + abs(self.x0), self.height + abs(self.y0))

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()

        # # field of view, aspect ratio, near and far
        # This will squash and stretch our objects as the window is resized.

        if self.fovy:
            gluPerspective( self.fovy*180/math.pi,
                        self.aspect,
                        self.Near,
                        self.Far)
        else:
            glFrustum(self.left, self.right, self.bottom, self.top, self.near, self.far)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()


    def __str__(self):
        subs = copy.copy(self.__dict__)
        subs['cam_position'] = self.cam_position
        subs['lookat'] = self.lookat
        subs['cam_up'] = self.cam_up
        subs['parent'] = None
        subs['T_local']= self.localTransformation
        subs['T_world']= self.globalTransformation
        subs['Near']   = self.Near
        subs['Far']   = self.Far
        subs['fovy']  = self.fovy
        subs['cam_type'] = self.cam_type
        subs['K'] = self.K
        subs['cx'] = self.cx
        subs['cy'] = self.cy
        parent = self.parent
        if parent:
            subs['parent'] = "{0}: {1}".format(type(parent), parent.name)

        return """
name              : {name}

Kinematic parameters
====================
parent            : {parent}
translation       : {translation}
rotation.jointAxis,ang): {rotation}
rotation(euleur)  : {rpy}
T_parent(M44)     : \n{T_local}
T_world(M44)      : \n{T_world}


OpenGL  parameters
===================
Near              : {Near}
Far               : {Far}
width             : {width}
height            : {height}
type              : {cam_type}
fovy              : {fovy}
cam_position      : {cam_position}
lookat            : {lookat}
cam_up            : {cam_up}

OpenCV  parameters
===================
K                 : {K}
fx                : {fx}
fy                : {fy}
cx                : {cx}
cy                : {cy}
""".format(**subs)
