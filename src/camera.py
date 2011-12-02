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
from kinematics import Shape
def norm(a):
    return sqrt(numpy.dot(a,a))
from vrml.geometry import Sphere
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
    r = 3.5
    x0 = 0
    y0 = 0
    aspect = 320.0/240.0

    # OpenCV params
    fx = 1.
    fy = 1.
    cx = 160 # (width/2)
    cy = 120 # (height/2)


    def __init__(self, server = None):
        kinematics.GenericObject.__init__(self)
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
        self.r = 3.5
        self.x0 = 0.
        self.y0 = 0.
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
        logger.debug("{0} GL vals: {1}".format(self.name,
                                              (self.x0, self.y0,
                                               self.fovy, self.aspect,
                                               self.left, self.right,
                                               self.bottom, self.top,
                                               self.near, self.far)))


    class Alias:
        frontClipDistance = ('Near','near')
        backClipDistance  = ('Far','far')
        fieldOfView       = ('fovy',)

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
    def lookat(self):
        z = self.T[:3,2]
        return self.T[:3,3] - z*self.r

    # def init_local_transformation(self):
    #     kinematics.GenericObject.init_local_transformation(self)
    #     T = numpy.eye(4)
    #     T[:3,0] = [-1, 0, 0]
    #     T[:3,2] = [0 , 0,-1]
    #     self.localTranformation = numpy.dot(self.localTransformation, T)

    def compute_opengl_params(self):
        '''
        Compute gl params from opencv params
        '''
        self.x0= self.width/2.0
        self.y0 = self.height/2.0

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
        self.aspect = 1.0*self.width/self.height

        self.top    =  math.atan(self.fovy/2)*self.near
        self.bottom = -math.atan(self.fovy/2)*self.near
        self.right  = self.aspect*self.top
        self.left   = self.aspect*self.bottom

        self.fx = 0.5*self.width*self.near/self.right
        self.fy = 0.5*self.height*self.near/self.top

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

        for name,ele in self.server.elements.items():
            if ele.exclude_cameras:
                if self.name in ele.exclude_cameras.split():
                    continue
            ele.render()

        if self.server.show_names:
            for name,ele in self.server.elements.items():
                try:
                    u,v = self.project(ele.T)
                except:
                    logger.exception("Failed to project {0}".format(ele.name))
                else:
                    u = int(u)
                    v = int(v)

                    self.server.draw_string("{0}".format(name, u, v), u, v)

        return True

    def project(self, T):
        cam_T = self.globalTransformation
        rel_T = numpy.dot(numpy.linalg.inv(cam_T), T)
        x,y,z = rel_T[:3,3]
        u = -self.fx*x/z + self.cx
        v = -self.fy*y/z + self.cy

        # print "cam_T", cam_T
        # print "T", T
        # print "rel_T", rel_T
        # print u, v
        # print "---"
        return u, v


    def rotate(self,dx,dy):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        x = self.localTransformation[:3,0]
        y = self.localTransformation[:3,1]
        z = self.localTransformation[:3,2]
        p = self.localTransformation[:3,3]
        f = p - z*self.r

        new_z = z  - dx*x*0.005 + dy*y*0.005
        new_z /= numpy.linalg.norm(new_z)
        new_x = numpy.cross(y, new_z)
        new_x[2] = 0
        new_x /= numpy.linalg.norm(new_x)
        new_y = numpy.cross(new_z, new_x)
        #print z, new_z, f, p
        new_p = f + new_z*self.r

        self.localTransformation[:3,0] = new_x
        self.localTransformation[:3,1] = new_y
        self.localTransformation[:3,2] = new_z
        self.localTransformation[:3,3] = new_p

        self.update()
        return


    def move_sideway(self,du,dv):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        x = self.localTransformation[:3,0]
        y = self.localTransformation[:3,1]
        z = self.localTransformation[:3,2]
        p = self.localTransformation[:3,3]
        d = du*0.01*(-x) +  dv*y*0.01
        self.localTransformation[:3,3] += d
        self.update()


    def move_back_forth(self,dy):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        if self.r < 0.01 and dy <=0:
            return
        factor = 0.005

        x = self.localTransformation[:3,0]
        y = self.localTransformation[:3,1]
        z = self.localTransformation[:3,2]
        p = self.localTransformation[:3,3]

        d = z*dy*factor

        self.localTransformation[:3,3] += d
        self.r += dy*factor

        self.update()


    def update_view(self):
        """

        Arguments:
        - `camera`:
        """
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        Tinv = numpy.linalg.inv(self.T)
        glMultMatrixd(numpy.transpose(Tinv))



    def update_perspective(self):
        glViewport(0, 0, self.width, self.height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()

        glFrustum(self.left, self.right, self.bottom, self.top, self.near, self.far)

    def __str__(self):
        subs = copy.copy(self.__dict__)
        subs['cam_position'] = self.T[:3,3]
        subs['lookat'] = self.lookat
        subs['cam_up'] = self.T[:3,2]
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
