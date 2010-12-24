#! /usr/bin/env python
import os, sys
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
from displayserver import DisplayServer
import threading

# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

def main():
    """
    """
    ds = DisplayServer()

    # Create server
    server = SimpleXMLRPCServer(("localhost", 8000))
    server.register_introspection_functions()
    server.register_function(ds.createElement , 'createElement')
    server.register_function(ds.destroyElement, 'destroyElement')
    server.register_function(ds.disableElement, 'disableElement')
    server.register_function(ds.enableElement , 'enableElement')
    server.register_function(ds.updateElementConfig , 'updateElementConfig')
    server.register_function(ds.getElementConfig , 'getElementConfig')
    server.register_function(ds.Ping , 'Ping')
    # server.serve_forever()
    t = threading.Thread(target = server.serve_forever)
    t.start()
    ds.run()

if __name__ == '__main__':
    main()
