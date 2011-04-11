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
        self.server.register_function(self.listElementDofs , 'listElementDofs')
        self.server.register_function(self.Ping , 'Ping')
        t = threading.Thread(target = self.server.serve_forever)
        t.start()

