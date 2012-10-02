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

try:
    from OpenGL.GL import *
    from OpenGL.GLUT import *
    from OpenGL.GLU import *
except:
    print "did not import OpenGL"
import parser
import standard_nodes as nodes

from abstract_geometry import Geometry

import logging
class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger = logging.getLogger("robotviewer.vrml.box")
logger.addHandler(NullHandler())

class Box(nodes.Box, Geometry):
    def render(self, scale = 3*[1.0]):
        sizex = self.size[0]*scale[0]
        sizey = self.size[1]*scale[1]
        sizez = self.size[2]*scale[2]
        count = 0
        v = []
        for x in [1, -1]:
            for y in [1, -1]:
                for z in [1, -1]:
                    v.append([x*sizex,y*sizey,z*sizez])
        v0 = v[0]
        v1 = v[1]
        v2 = v[2]
        v3 = v[3]
        v4 = v[4]
        v5 = v[5]
        v6 = v[6]
        v7 = v[7]
        glBegin(GL_QUADS)
        glNormal3f(1,0,0)
        glVertex3fv(v0)    # front face
        glVertex3fv(v2)
        glVertex3fv(v3)
        glVertex3fv(v1)

        glNormal3f(0,1,0)
        glVertex3fv(v0)    # right face
        glVertex3fv(v1)
        glVertex3fv(v5)
        glVertex3fv(v4)

        glNormal3f(0,0,1)
        glVertex3fv(v0)    # up face
        glVertex3fv(v4)
        glVertex3fv(v6)
        glVertex3fv(v2)

        glNormal3f(-1,0,0)
        glVertex3fv(v4)    # back face
        glVertex3fv(v5)
        glVertex3fv(v7)
        glVertex3fv(v6)

        glNormal3f(0,-1,0)
        glVertex3fv(v2)    # left face
        glVertex3fv(v6)
        glVertex3fv(v7)
        glVertex3fv(v3)

        glNormal3f(0,0,-1)
        glVertex3fv(v1)    # down face
        glVertex3fv(v3)
        glVertex3fv(v7)
        glVertex3fv(v5)
        glEnd()
