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


import re, sys
import kinematic_chain
import os
import logging, pickle

logger = logging.getLogger("robotviewer.ml_parser")
class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger.addHandler(NullHandler())

parsers = { 'vrml': 'vrml_parser',
            'wrl': 'vrml_parser',
            'kxml': 'kxml_parser'
            }
supported_extensions = parsers.keys()

def load_cache(cached_file):
    try:
        objs = pickle.load(open(cached_file))
    except:
        logger.warning("Couldn't load cached file.")
        os.remove(cached_file)
        return None

    if type(objs)!= list:
        logger.warning("Old on invalid cached file. Removing %s"%cached_file)
        return None

    gen_objs = [ o for o in objs if isinstance(o, kinematic_chain.GenericObject) ]

    if not gen_objs[:]:
        logger.warning("Cached does not contain any object. Removing %s"%cached_file)
        return None

    return objs


def parse_nocache(filename):
    re_ext = re.compile(r"\.(?P<EXT>[A-Za-z]+)$")
    m = re_ext.search(filename)
    if not m:
        raise Exception("Couldn't find file extension of %s"%filename)
    ext = m.group('EXT')
    if ext not in parsers.keys():
        raise Exception("Unknown extension %s"%ext)
    parser = __import__(parsers[ext],globals(), locals())
    return parser.parse(filename)

def parse(filename, use_cache = True):
    alpha_name = filename.replace("/","_")
    if re.match(r".*\.cache", filename):
        cached_file = filename
    else:
        cached_file = os.path.join(os.environ['HOME'],
                                   '.robotviewer',"%s.cache"%alpha_name )

    if not use_cache and os.path.isfile(cached_file):
        os.remove(cached_file)

    if os.path.isfile(cached_file):
        objs = load_cache(cached_file)
        if objs:
            return objs

    objs =  parse_nocache(filename)
    logger.warning("Saving new cache to %s"%cached_file)
    f = open(cached_file,'w')
    pickle.dump(objs,f)
    f.close()
    logger.warning("Finished saving new cache to %s"%cached_file)
    logger.debug("Loaded %s"%str(objs))
    return objs

if __name__ == '__main__':
    for obj in parse(sys.argv[1]):
        print obj
