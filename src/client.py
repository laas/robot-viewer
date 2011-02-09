#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2010
# Authors Duong Dang


import os, sys
import xmlrpclib


def client(server = "CORBA"):
    if server == "CORBA":
        import corba_util
        sys.path = [os.path.dirname(
                os.path.abspath(__file__))+"/idl"] + sys.path
        import hpp, hpp__POA
        return corba_util.GetObject('hpp','hpp.RobotViewer',
                                    [('RobotViewer','context'),
                                     ('RobotViewer','object')])
    elif server == "XML-RPC":
        return xmlrpclib.ServerProxy("http://localhost:8000/")
    else:
        raise Exception("Invalid server type")
