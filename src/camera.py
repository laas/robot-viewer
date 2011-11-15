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
from math import sqrt,sin,cos,atan2, pi, atan, tan
import numpy
from mathaux import *
import copy
import alias
def norm(a):
    return sqrt(numpy.dot(a,a))

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

def normalized(v):
    return v/norm(v)

OVERHEAD_THRESHOLD = 1 - 1e-3

class Camera(kinematics.GenericObject, alias.Aliaser):
    '''
    World camera and robot cameras. Use OpenHRP convention
    http://openrtp.info/openhrp3/en/create_model.html#VISIONSENSOR
    '''

    # OPENGL params
    frontClipDistance = 0.01
    backClipDistance = 100
    width = 320
    height = 240
    cam_type = "COLOR"
    fieldOfView = 45*pi/180
    translation = [0, 0, 0]
    focal = 3.5
    x0 = 0
    y0 = 0
    aspect = 320.0/240.0

    # OpenCV params
    fx = 0.
    fy = 0.
    cx = 160 # (width/2)
    cy = 120 # (height/2)

    class Alias:
        frontClipDistance = ('Near',)
        backClipDistance  = ('Far',)
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
        self.x0 = int(self.u0  - self.width/2.0)
        self.y0 = int(self.height/2.0 - self.v0)
        self.fovy = 2*atan(1.0*self.height/(2*self.fy))
        self.aspect = 1.0*self.width/self.height*self.fy/self.fx

    def compute_opencv_params(self):
        '''
        Compute gl params from opencv params
        '''
        self.cx = self.width/2.0
        self.cy = self.height/2.0
        self.u0 = int(self.x0 + self.width/2.0 )
        self.v0 = int(self.y0 + self.height/2.0)
        # print 'u0=', self.u0, ', cx=', self.cx
        self.fy = 1.0*self.height/tan(self.fovy/2.0)
        self.fx = 1.0*self.width/self.height*self.fy/self.aspect

    def set_opencv_params(self, width, height, fx, fy, cx, cy):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.compute_opengl_params()

    def __init__(self):
        kinematics.GenericObject.__init__(self)
        self.localR = numpy.array([ [ 0 , 0 , 1],
                                    [ 1 , 0 , 0],
                                    [ 0 , 1 , 0],
                                    ]
                                  )
        self.rotation = rot2AxisAngle(self.localR)
        self.moved = True
        self.lookat = None
        self.init()
        self.compute_opencv_params()
        # print self.globalTransformation

    def render(self):
        pass

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

        if (abs(normalized(self.cam_ray)[2] ) > OVERHEAD_THRESHOLD
            and dup*self.cam_ray[2] > 0 ):
            dup = 0

        new_z = self.cam_ray + dup*self.cam_up + dright*self.cam_right
        new_pos = self.lookat + new_z*self.focal

        # trans = new_pos - self.cam_position
        new_x = - normalized(numpy.cross(new_z, numpy.array([0,0,1])))
        new_y = numpy.cross(new_z, new_x)
        new_R = numpy.eye(3)

        new_R[:,0] =  new_x
        new_R[:,1] =  new_y
        new_R[:,2] =  new_z

        rpy = rot2rpy(new_R[:3,:3])
        config = self.get_config()
        config[0] = new_pos[0]
        config[1] = new_pos[1]
        config[2] = new_pos[2]
        #config[3] = rpy[0]
        #config[4] = rpy[1]
        #config[5] = rpy[2]

        self.translation  = new_pos
        self.localTransformation[:3,3] = new_pos
        self.localTransformation[:3,:3] = new_R

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
        if self.height == 0:
            # Prevent A Divide By Zero If The Window Is Too Small
            self.height = 1

        glViewport(0, 0, self.width, self.height)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # # field of view, aspect ratio, near and far
        # This will squash and stretch our objects as the window is resized.
        gluPerspective( self.fovy*180/pi,
                        self.aspect,
                        self.Near,
                        self.Far)

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
