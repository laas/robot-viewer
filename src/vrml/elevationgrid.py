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


class ElevationGrid(nodes.ElevationGrid, Geometry):
    def render(self, scale = 3*[1.]):
        Xs = [self.xSpacing*i*scale[0] for i in range(self.xDimension)]
        Zs = [self.zSpacing*i*scale[2] for i in range(self.zDimension)]
        for i in range(self.zDimension - 1):
            for j in range(self.xDimension -1):
                A = Xs[j],   self.height[j   + (i)  *self.zDimension], Zs[i]
                B = Xs[j],   self.height[j   + (i+1)*self.zDimension], Zs[i+1]
                C = Xs[j+1], self.height[j+1 + (i+1)*self.zDimension], Zs[i+1]
                D = Xs[j+1], self.height[j+1 + (i)  *self.zDimension], Zs[i]

                count = j + i*(self.xDimension-1)
                color =  self.color.color[3*count:3*count+3]

                glColor3fv(color)
                glBegin(GL_QUADS)
                glNormal3f(0,1,0)
                glVertex3f(A[0], A[1], A[2])
                glVertex3f(B[0], B[1], B[2])
                glVertex3f(C[0], C[1], C[2])
                glVertex3f(D[0], D[1], D[2])
                glNormal3f(0,-1,0)
                glVertex3f(A[0], A[1], A[2])
                glVertex3f(D[0], D[1], D[2])
                glVertex3f(C[0], C[1], C[2])
                glVertex3f(B[0], B[1], B[2])

                # print count, color, A, B, C, D

                glEnd()

