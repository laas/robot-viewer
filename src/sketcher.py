#! /usr/bin/env python

__author__ = "Duong Dang"
__version__ = "0.1"

import logging, sys, os
import ml_parser

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("sketcher")
logger.addHandler(NullHandler())
logger.setLevel(logging.DEBUG)

def sketch(robot):
    for j in robot.joint_list:
        print j.name

def main():
    import optparse
    parser = optparse.OptionParser(
        usage='\n\t%prog [options]',
        version='%%prog %s' % __version__)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")
    (options, args) = parser.parse_args(sys.argv[1:])
    robot = ml_parser.parse(args[0])[0]
    sketch(robot)

if __name__ == '__main__':
    main()
