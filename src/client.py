#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2010
# Authors Duong Dang


import os, sys
import corba_util
sys.path = [os.path.dirname(os.path.abspath(__file__))+"/idl"] + sys.path
import hpp, hpp__POA

client = corba_util.GetObject('hpp','hpp.RobotViewer',\
                                  [('RobotViewer','context'),('RobotViewer','object')])

# leave sys.path as it was
sys.path = sys.path[2:]
