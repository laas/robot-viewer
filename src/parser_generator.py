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
import logging, sys, os, re
import optparse
logger = logging.getLogger("parser_genrator")

class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger.addHandler(NullHandler())
prologue = """
import logging, os, sys
from simpleparse.common import numbers, strings, comments
from simpleparse.parser import Parser
from simpleparse.dispatchprocessor import *
import pprint
import optparse
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger()
logger.addHandler(NullHandler())

def get_parser():
    usage = "usage: [options]"
    parser = optparse.OptionParser(usage=usage)

    parser.add_option("-v", "--verbose",
                      action = "store_true", dest = "verbose",
                      help = "be verbose`"
                      )


    return parser
"""

method_doc = "FIXME"



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


    return parser

def caseinsensitive_sort(stringList):
    """case-insensitive string comparison sort
    doesn't do locale-specific compare
    though that would be a nice addition
    usage: stringList = caseinsensitive_sort(stringList)"""

    tupleList = [(x.lower(), x) for x in stringList]
    tupleList.sort()
    return [x[1] for x in tupleList]


def main():
    oparser = get_parser()
    opts, args = oparser.parse_args()

    s = open(opts.grammar).read()
    patt = re.compile(r"[\w\d]+\s*(?=::)")

    ext_name,ext = os.path.splitext(os.path.basename(opts.grammar))
    ext_name = ext_name.capitalize()

    if not opts.root:
        opts.root = "%sFile"%ext_name

    res = prologue
    res += """
class {0}Processor(DispatchProcessor):
""".format(ext_name)

    matches =  patt.findall(s)
    for i in range(len(matches)):
        matches[i] = matches[i].strip()

    matches = caseinsensitive_sort(matches)

    for definition in matches:
        res +="""
    def {0}(self,(tag,start,stop,subtags), buffer ):
        '''{1}'''
        logger.debug('dispatching {0}')
        return None

""".format(definition, method_doc)

    res += """
class {0}Parser(Parser):
    def buildProcessor(self):
        return {0}Processor()
""".format(ext_name)


    res +="""
def main():
    oparser = get_parser()
    opts, args = oparser.parse_args()
    logger = logging.getLogger()
    sh = logging.StreamHandler()
    if opts.verbose:
        sh.setLevel(logging.DEBUG)
        logger.setLevel(logging.DEBUG)


    logger.addHandler(sh)
    data = open(args[0]).read()
    DEF = open('{0}').read()
    parser = {1}Parser(DEF, '{2}')
    print parser.parse(data)


if __name__ == '__main__':
    main()
""".format(opts.grammar, ext_name, opts.root)

    print res



if __name__ == '__main__':
    main()
