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


class IndexedLineSet(nodes.IndexedLineSet, Geometry):
    def render(self, scale = 3*[1.]):
        points = zip(self.coord.point[::3],
                     self.coord.point[1::3],
                     self.coord.point[2::3],
                     )
        if self.color:
            colors = zip(self.color.color[::3],
                         self.color.color[1::3],
                         self.color.color[2::3],
                         )
        else:
            colors = []

        lines = []
        line = []
        for idx in self.coordIndex:
            if idx == -1:
                lines.append(line)
                line = []
            line.append(idx)
        for i, line in enumerate(lines):
            if colors[:]:
                glColor(colors[self.colorIndex[i]])
            glBegin(GL_LINE_STRIP)
            for j in line:
                glVertex3fv( points[j] )
            glEnd()
        return

