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
import logging, sys, os
import optparse
from simpleparse.parser import Parser


logger = logging.getLogger("grammar_tester")

class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger.addHandler(NullHandler())

def get_parser():
    usage = "usage: [options]"
    parser = optparse.OptionParser(usage=usage)
    parser.add_option("-g", "--grammar",
                      action = "store", dest = "grammar",
                      help = "grammar definition"
                      )

    parser.add_option("-r", "--root",
                      action = "store", dest = "root",
                      help = "root node name"
                      )

    parser.add_option("-i", "--input",
                      action = "store", dest = "input",
                      help = "input file"
                      )
    return parser

def main():
    oparser = get_parser()
    opts, args = oparser.parse_args()

    parser = Parser(open(opts.grammar).read(),
                    opts.root)
    success, tags, next = parser.parse( open(opts.input).read() )
    print tags

if __name__ == '__main__':
    main()

