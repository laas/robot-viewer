#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang
import re, sys
import vrml_parser
import robo

def robotLoader(filename):
    re_ext = re.compile(r"\.(?P<EXT>[A-Za-z]+)$")
    m = re_ext.search(filename)
    if not m:
        raise Exception("Couldn't find file extension of %s"%filename)
    ext = m.group('EXT')
    if ext in  ["vrml","wrl"]:
        parsed_objects = vrml_parser.parse(filename)
        for obj in parsed_objects:
            if isinstance(obj,robo.BaseNode):
                obj.init()
                return obj
        raise Exception("No robot found in the scene")
    else:
        raise Exception("Unknown extension %s"%ext)

if __name__ == '__main__':
    print robotLoader(sys.argv[1])
