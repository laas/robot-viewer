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

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

import parser
import standard_nodes as nodes

from abstract_geometry import Geometry

import logging
class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger = logging.getLogger("robotviewer.vrml.box")
logger.addHandler(NullHandler())


class Cylinder(nodes.Cylinder, Geometry):
    def render(self, scale = 3*[1.]):
        quad = gluNewQuadric()

        glRotatef(-90, 1, 0, 0)
        glTranslatef(0, 0, -self.height*scale[2]/2)

        if self.side:
            gluCylinder(quad, self.radius*scale[0], self.radius*scale[0],
                        self.height*scale[2], 20,20)
        if self.bottom:
            glPushMatrix()
            glRotatef(180, 1., 0., 0.)
            gluDisk(quad, 0., self.radius*scale[0], 20, 20)
            glPopMatrix()

        if self.top:
            glPushMatrix()
            glTranslatef(0, 0, self.height*scale[2])
            gluDisk(quad, 0., self.radius*scale[0], 20, 20)
            glPopMatrix()

        gluDeleteQuadric(quad)
