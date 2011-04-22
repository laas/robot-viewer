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
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
from displayserver import DisplayServer
import threading

import logging
logger = logging.getLogger("robotviewer.displayserver_xmlrpc")
class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger.addHandler(NullHandler())

class DisplayServerXmlrpc(DisplayServer):
    """
    """

    def __init__(self, options = None, args = None):
        """

        Arguments:
        - `options`:
        - `args`:
        """
        logger.debug("Creating generic DisplayServer")
        DisplayServer.__init__(self, options, args)
        # Create server
        self.server = SimpleXMLRPCServer(("localhost", 8000))
        self.server.register_introspection_functions()
        self.server.register_function(self.createElement , 'createElement')
        self.server.register_function(self.destroyElement, 'destroyElement')
        self.server.register_function(self.disableElement, 'disableElement')
        self.server.register_function(self.enableElement , 'enableElement')
        self.server.register_function(self.updateElementConfig , 'updateElementConfig')
        self.server.register_function(self.getElementConfig , 'getElementConfig')
        self.server.register_function(self.listElements , 'listElements')
        self.server.register_function(self.list_element_dofs , 'list_element_dofs')
        self.server.register_function(self.Ping , 'Ping')
        t = threading.Thread(target = self.server.serve_forever)
        t.start()

