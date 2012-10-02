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

import numpy
import numpy.linalg
import re
from math import sin, cos, isnan, pi, acos
from collections import deque
import logging, uuid
import __builtin__
import traceback
import parser
import standard_nodes as nodes
try:
    import OpenGL
    # OpenGL.FORWARD_COMPATIBLE_ONLY = True
    from OpenGL.GL import *
    from OpenGL.GLUT import *
    from OpenGL.GLU import *
    from OpenGL.GL.ARB.vertex_buffer_object import *
except:
    print "did not import OpenGL"
from abc import ABCMeta, abstractmethod
    
import numpy
import numpy.linalg
import pprint
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("robotviewer.geometry")
logger.addHandler(NullHandler())

from abstract_geometry import Geometry
class Text(nodes.Text, Geometry):
    def render(self, scale = 3*[1.]):
        logger.info("Trying to render '{0}'".format(self.string))
        logger.fatal("Text rendering has not been implented")
