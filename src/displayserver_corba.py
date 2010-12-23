#! /usr/bin/env python


import os, sys
import corba_util

sys.path = [os.path.dirname(os.path.abspath(__file__))+"/idl"] + sys.path
import hpp, hpp__POA
from displayserver import DisplayServer

class RobotViewer_i(DisplayServer,hpp__POA.RobotViewer):
    def __init__(self):
        DisplayServer.__init__(self)

    # def getElementConfig(*args, **kwargs):
    #     try:
    #         DisplayServer.getElementConfig(*args, **kwargs)
    #     except:
    #         raise hpp.RobotViewer.InvalidKey, "Invalid element name %s"%name

    # def updateElementConfig(*args, **kwargs):
    #     try:
    #         DisplayServer.updateElementConfig(*args, **kwargs)
    #     except:
    #         raise hpp.RobotViewer.InvalidKey, "Invalid element name %s"%name

    # def enableElement(*args, **kwargs):
    #     try:
    #         DisplayServer.enableElement(*args, **kwargs)
    #     except:
    #         raise hpp.RobotViewer.InvalidKey, "Invalid element name %s"%name

    # def disableElement(*args, **kwargs):
    #     try:
    #         DisplayServer.disableElement(*args, **kwargs)
    #     except:
    #         raise hpp.RobotViewer.InvalidKey, "Invalid element name %s"%name

    # def destroyElement(*args, **kwargs):
    #     try:
    #         DisplayServer.destroyElement(*args, **kwargs)
    #     except:
    #         raise hpp.RobotViewer.InvalidKey, "Invalid element name %s"%name


def main():
    """
    """
    server = RobotViewer_i()
    corba_util.StartServer(server, 'hpp', 'RobotViewer.object', [('RobotViewer','context')])
    server.run()


if __name__ == '__main__':
    main()
