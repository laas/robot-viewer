#! /usr/bin/env python
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
__author__ = "Duong Dang"
__version__ = "0.1"

import logging, sys, os

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("shader")
logger.addHandler(NullHandler())
logger.setLevel(logging.DEBUG)


import os
import OpenGL
OpenGL.FORWARD_COMPATIBLE_ONLY = True
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GLX import *
from OpenGL.GL.EXT.framebuffer_object import *
from OpenGL.GL.shaders import *


class Shader(object):
    """
    """

    def __init__(self, program = None):
        """
        """
        self.program = program
        self.created = True

    def __setattr__(self, name, value):
        if 'created' not in self.__dict__.keys():
            object.__setattr__(self, name, value)
            return

        location = glGetUniformLocation(self.program, name)

        if isinstance(value, int):
            glUniform1i(location, value)
            return

        elif isinstance(value, float):
            glUniform1f(location, value)
            return

        n = len(value)
        if isinstance(value[0], int):
            if n == 2:
                glUniform2i(location, value[0], value[1])
            elif n == 3:
                glUniform3f(location, value[0], value[1], value[2])
            elif n == 4:
                glUniform4i(location, value[0], value[1], value[3], value[4])
        else:
            if n == 2:
                glUniform2f(location, value[0], value[1])
            elif n == 3:
                glUniform3f(location, value[0], value[1], value[2])
            elif n == 4:
                glUniform4f(location, value[0], value[1], value[3], value[4])


def main():
    import optparse
    parser = optparse.OptionParser(
        usage='\n\t%prog [options]',
        version='%%prog %s' % __version__)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")
    (options, args) = parser.parse_args(sys.argv[1:])


if __name__ == '__main__':
    main()
