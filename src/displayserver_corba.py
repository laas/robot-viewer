#! /usr/bin/env python


import os, sys
import corba_util

sys.path = [os.path.dirname(os.path.abspath(__file__))+"/idl"] + sys.path
import hpp, hpp__POA
from displayserver import DisplayServer

class DisplayServerCorba(DisplayServer,hpp__POA.RobotViewer):
    def __init__(self):
        DisplayServer.__init__(self)

def main():
    """
    """
    server = DisplayServerCorba()
    corba_util.StartServer(server, 'hpp', 'RobotViewer.object', [('RobotViewer','context')])
    server.run()


if __name__ == '__main__':
    main()
