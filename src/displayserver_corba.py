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


import os, sys
import corba_util

sys.path = [os.path.dirname(os.path.abspath(__file__))+"/idl"] + sys.path
import robotviewer_corba, robotviewer_corba__POA
from displayserver import DisplayServer

class DisplayServerCorba(DisplayServer,robotviewer_corba__POA.RobotViewer):
    def __init__(self,options = None, args = None):
        DisplayServer.__init__(self, options, args)
        corba_util.StartServer(self, 'robotviewer_corba', 'RobotViewer.object', [('RobotViewer','context')])
