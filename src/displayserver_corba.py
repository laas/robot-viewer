#! /usr/bin/env python


import os, sys
import corba_util

sys.path = [os.path.dirname(os.path.abspath(__file__))+"/idl"] + sys.path
import hpp, hpp__POA
from displayserver import DisplayServer

class DisplayServerCorba(DisplayServer,hpp__POA.RobotViewer):
    def __init__(self,options = None, args = None):
        DisplayServer.__init__(self, options, args)
        corba_util.StartServer(self, 'hpp', 'RobotViewer.object', [('RobotViewer','context')])
