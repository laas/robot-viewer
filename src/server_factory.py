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


KINEMATIC, OPENGL, LIBCACA = 0,1,2
CORBA, XML_RPC, NONE = 0,1,2

import os, sys
import logging
logger = logging.getLogger("robotviewer.server_factory")
class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger.addHandler(NullHandler())
def create_server( stype, com_type, options, args):
    if stype == KINEMATIC:
        import kinematic_server
        ServerClass = kinematic_server.KinematicServer
    elif stype == OPENGL:
        import display_server
        ServerClass = display_server.DisplayServer
    elif stype == LIBCACA:
        import libcaca_server
        ServerClass = libcaca_server.LibcacaServer

    if com_type == CORBA:
        sys.path = [os.path.dirname(os.path.abspath(__file__))+"/idl"] + sys.path
        import robotviewer_corba, robotviewer_corba__POA, corba_util
        class Server(ServerClass,robotviewer_corba__POA.RobotViewer):
            def __init__(self, options = None, args = None):
                ServerClass.__init__(self, options, args)
                corba_util.StartServer(self, 'robotviewer_corba', 'RobotViewer.object', [('RobotViewer','context')])

    elif com_type == XML_RPC:
        from SimpleXMLRPCServer import SimpleXMLRPCServer
        from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
        import threading

        class Server(ServerClass):
            """
            """
            def __init__(self, options = None, args = None):
                """

                Arguments:
                - `options`:
                - `args`:
                """
                logger.debug("Creating generic ServerClass")
                ServerClass.__init__(self, options, args)
                # Create server
                self.server = SimpleXMLRPCServer(("localhost", 8001))
                self.server.register_introspection_functions()
                self.server.register_function(self.createElement , 'createElement')
                self.server.register_function(self.destroyElement, 'destroyElement')
                self.server.register_function(self.disableElement, 'disableElement')
                self.server.register_function(self.enableElement , 'enableElement')
                self.server.register_function(self.updateElementConfig , 'updateElementConfig')
                self.server.register_function(self.getElementConfig , 'getElementConfig')
                self.server.register_function(self.listElements , 'listElements')
                self.server.register_function(self.listElementDofs , 'listElementDofs')

                self.server.register_function(self.listCameras     , 'listCameras')
                self.server.register_function(self.getCameraConfig , 'getCameraConfig')
                self.server.register_function(self.getCameraInfo   , 'getCameraInfo')


                t = threading.Thread(target = self.server.serve_forever)
                t.start()
    elif com_type == NONE:
        import display_server
        Server = display_server.DisplayServer

    return Server(options, args)
