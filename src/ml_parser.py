#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang

import re, sys
import robo

parsers = { 'vrml': 'vrml_parser',
            'wrl': 'vrml_parser',
            'kxml': 'kxml_parser'
            }

def parse(filename):
    re_ext = re.compile(r"\.(?P<EXT>[A-Za-z]+)$")
    m = re_ext.search(filename)
    if not m:
        raise Exception("Couldn't find file extension of %s"%filename)
    ext = m.group('EXT')
    if ext not in parsers.keys():
        raise Exception("Unknown extension %s"%ext)
    parser = __import__(parsers[ext],globals(), locals())
    return parser.parse(filename)

if __name__ == '__main__':
    for obj in parse(sys.argv[1]):
        print obj
