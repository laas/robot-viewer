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

__author__ = "Duong Dang"
__version__ = "0.1"

import logging, os, sys

from simpleparse.common import numbers, strings, comments
from simpleparse.parser import Parser
from simpleparse.dispatchprocessor import *
from numbers import Number
import pprint

from kinematics import Mesh, Appearance, Geometry
from kinematics import GenericObject, Joint, Robot
from camera import Camera
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("robotviewer.vrml_parser")
logger.addHandler(NullHandler())

path = os.path.abspath(os.path.dirname(__file__))
grammar_file = os.path.join(path,"vrml.sbnf" )
VRMLPARSERDEF = open(grammar_file).read()


class VrmlNode(dict):
    def __init__(self, n = None):
        self.name = n
        self['name'] = n

class Definition(str):
    pass

class Attribute(tuple):
    pass

class VrmlProcessor(DispatchProcessor):
    """
    """
    def __init__(self,root_path = None):
        self.def_dict = {}
        self.root_path = root_path

    def DefNode(self,(tag,start,stop,subtags), buffer ):
        key, n = dispatchList(self, subtags, buffer)
        self.def_dict[key] = n
        return n

    def inline(self,(tag,start,stop,subtags), buffer ):
        attrs = dispatchList(self, subtags, buffer)
        for key, val in attrs:
            if key == 'url':
                vrml_path = val
                if self.root_path:
                    vrml_path = os.path.join(self.root_path, vrml_path)
                    if os.path.isfile(vrml_path):
                        data = parse(vrml_path)
                        return data[0]
                    else:
                        logger.exception("Couldnt find %s"%(vrml_path))

    def genericObject(self,(tag,start,stop,subtags), buffer ):
        node = GenericObject()
        s = buffer[start:stop][:200]
        attrs = dispatchList(self, subtags, buffer)
        scale = None
        for key, val in attrs:
            if key == 'children':
                l = val
                for c in val:
                    if not isinstance(c, GenericObject):
                        logger.debug("Ignoring child %s"%str(c))
                        continue
                    node.add_child(c)
            elif key in ['translation','rotation']:
                node.__dict__[key] = val
            elif key == "scale":
                scale = val
        if scale:
            node.init()
            try:
                node.scale(scale)
            except TypeError:
                logger.warning("Failed to scale.\nratio:{0}\nShape:{1}...".format(scale, s) )
        return node

    def visionSensor(self,(tag,start,stop,subtags), buffer ):
        node = Camera()
        attrs = dispatchList(self, subtags, buffer)
        for key, val in attrs:
            if key == 'children':
                l = val
                for c in val:
                    if not isinstance(c, GenericObject):
                        logger.debug("Ignoring child %s"%str(c))
                        continue
                    node.add_child(c)
            elif key in ['translation','rotation']:
                node.__dict__[key] = val
        return node

    def forceSensor(self,(tag,start,stop,subtags), buffer ):
        return self.genericObject((tag,start,stop,subtags), buffer )

    def humanoid(self,(tag,start,stop,subtags), buffer ):
        node = Robot()
        attrs = dispatchList(self, subtags, buffer)
        for key, val in attrs:
            if key == "joints":
                node.joint_names = val
            elif key == 'segments':
                node.segment_names = val
            elif key == 'humanoidBody':
                l = val
                for v in l:
                    node.add_child(v)
            elif key in ['translation','rotation']:
                node.__dict__[key] = val

        return node

    def joint(self,(tag,start,stop,subtags), buffer ):
        node = Joint()
        attrs = dispatchList(self, subtags, buffer)

        for key, val in attrs:
            if key == "jointId":
                node.id = val
            elif key == "jointType":
                node.jointType = val
            elif key == 'jointAxis':
                node.axis = val
            elif key in ['translation','rotation']:
                node.__dict__[key] = val
            elif key == "children":
                for c in val:
                    if not isinstance(c, GenericObject):
                        logger.debug("Ignoring child %s"%str(c))
                        continue
                    node.add_child(c)
        return node


    def shape(self,(tag,start,stop,subtags), buffer ):
        node = Mesh()
        attrs = dispatchList(self, subtags, buffer)
        s = buffer[start:stop][:200]
        for key, val in attrs:
            if key == "appearance":
                try:
                    node.app = val[0][0]
                except TypeError:
                    logger.debug("Failed to set app for shape:" + s)
            elif key == "geometry":
                try:
                    node.geo = val[0]
                except TypeError:
                    logger.warning("Failed to set app for shape:" + s)

        return node


    def appearance(self,(tag,start,stop,subtags), buffer ):
        attrs = dispatchList(self, subtags, buffer)
        for key, val in attrs:
            if key == "material":
                return val

    def material(self,(tag,start,stop,subtags), buffer ):
        app = Appearance()
        attrs = dispatchList(self, subtags, buffer)
        for key, val in attrs:
            app.__dict__[key] = val
        return app

    def indexedFaceSet(self,(tag,start,stop,subtags), buffer ):
        attrs = dispatchList(self, subtags, buffer)
        geo = Geometry()
        for key, val in attrs:
            if key == "coordIndex":
                geo.idx = val
            elif key == "coord":
                geo.coord = val[0]
            elif key == "normal":
                pass
                #geo.norm = val[0]
                #geo.normals = val[0]
        return geo

    def coordinate(self,(tag,start,stop,subtags), buffer ):
        attrs = dispatchList(self, subtags, buffer)
        for key, val in attrs:
            if key == "point":
                return val

    def normal(self,(tag,start,stop,subtags), buffer ):
        attrs = dispatchList(self, subtags, buffer)
        for key, val in attrs:
            if key == "vector":
                return val

    def unknownNode(self,(tag,start,stop,subtags), buffer ):
        s = buffer[start:stop][:500]
        name = dispatch(self, subtags[0], buffer)
        logger.warning("unknown Node: {0}. Excerpt of ignored node\n{1}".format(name,s))
        return

    def name(self,(tag,start,stop,subtags), buffer ):
        return str(buffer[start:stop])

    def Attr(self,(tag,start,stop,subtags), buffer ):
        name = dispatch(self,subtags[0],buffer)
        value = dispatch(self,subtags[1],buffer)
        return Attribute((name, value))

    def Def(self,(tag,start,stop,subtags), buffer ):
        name = dispatch(self, subtags[0], buffer)
        return Definition(name)

    def Field(self,(tag,start,stop,subtags), buffer ):
        l =  dispatchList(self, subtags, buffer)
        if ( len(l) == 1 and (isinstance(l[0], Number)
                              or isinstance(l[0],basestring) )):
            return l[0]
        else:
            return l


    def SFNumber(self,(tag,start,stop,subtags), buffer ):
        s = buffer[start:stop]
        try:
            return int(s)
        except:
            return float(s)

    def SFBool(self,(tag,start,stop,subtags), buffer ):
        s = buffer[start:stop]
        if s.upper() == "TRUE":
            return True
        elif s.upper() == "FALSE":
            return False
        else:
            raise Exception("Failed to read a bool %s"%s)

    def SFString(self,(tag,start,stop,subtags), buffer ):
        s = buffer[start:stop]
        s=s.replace('"',"")
        return s

    def IS(self,(tag,start,stop,subtags), buffer ):
        return None
        s = buffer[start:stop]

        if s not in self.def_dict.keys():
            raise Exception("IS used before node definition: %s"%s[:100])

        return self.def_dict[s]


    def USE(self,(tag,start,stop,subtags), buffer ):
        return None
        s = buffer[start:stop]

        if s not in self.def_dict.keys():
            raise Exception("USE used before node definition. %s"%s[:100])

        return self.def_dict[s]

class VrmlParser(Parser):
    def __init__(self, root_path, *args, **kwargs):
        Parser.__init__(self,*args, **kwargs)
        self.root_path = root_path


    def buildProcessor( self ):
        return VrmlProcessor(self.root_path)

def parse(filename):
    vrml_dir_path = os.path.abspath(os.path.dirname(filename))
    parser = VrmlParser(vrml_dir_path, VRMLPARSERDEF, "vrmlScene" )
    data = open(filename).read()
    objs = parser.parse(data)[1]
    objs = [ o for o in objs if isinstance(o,GenericObject)]
    for obj in objs:
        obj.init()
        logger.debug("Load {0} from {1}, with {2} mesh(es)".format(obj.name, filename, len(obj.mesh_list)))
    return objs


def main():
    import optparse
    logger = logging.getLogger("kinematics")
    logger.setLevel(logging.DEBUG)
    sh = logging.StreamHandler()
    sh.setLevel(logging.DEBUG)
    logger.addHandler(sh)
    parser = optparse.OptionParser(
        usage='\n\t%prog [options]',
        version='%%prog %s' % __version__)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")
    (options, args) = parser.parse_args(sys.argv[1:])
    res = parse(args[0])
    for r in res:
        #print r.mesh_list
        print r

if __name__ == '__main__':
    main()
