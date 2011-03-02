#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang

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
    return objs

if __name__ == '__main__':
    for obj in parse(sys.argv[1]):
        print obj
