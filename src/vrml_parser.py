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

class VrmlProcessor(DispatchProcessor):
    """
    """
    def __init__(self,root_path = None):
        self.def_dict = {}
        self.root_path = root_path

    def Node(self,(tag,start,stop,subtags), buffer ):
        defname = None
        if subtags[0][0] == "name":
            name = dispatch(self, subtags[0], buffer)
            vrml_node = VrmlNode(name)
            children = dispatchList(self, subtags, buffer)
            for pair in children[1:]:
                vrml_node[pair[0]] = pair[1]

        elif subtags[0][0] == "Def":
            name = dispatch(self, subtags[1], buffer)
            defname = dispatch(self, subtags[0], buffer)
            vrml_node = VrmlNode(name)
            children = dispatchList(self, subtags, buffer)
            for pair in children[2:]:
                vrml_node[pair[0]] = pair[1]

        processed_node = vrml_node

        if vrml_node.name == "Appearance":
            processed_node = Appearance()
            if 'material' not in vrml_node.keys():
                logger.warning("No material node found for %s"%vrml_node)
            else:
                try:
                    mat = vrml_node['material']
                    processed_node.__dict__.update(vrml_node['material'][0])
                except:
                    logger.exception("Exception on:\nvrml_node={0}\nbuffer={1}..."
                                 .format(vrml_node,buffer[start:max(stop,start+200)]))
                    raise

        elif vrml_node.name == "IndexedFaceSet":
            processed_node = Geometry()
            processed_node.coord = vrml_node['coord'][0]['point']
            processed_node.idx = vrml_node['coordIndex']

        elif ( isinstance(vrml_node,dict) and 'appearance' in vrml_node.keys()
               and 'geometry' in vrml_node.keys()):
            processed_node = Mesh()
            processed_node.app = vrml_node['appearance'][0]
            processed_node.geo = vrml_node['geometry'][0]

        elif vrml_node.name == "Transform":
            processed_node = GenericObject()
            for key in 'translation', 'rotation':
                if key in vrml_node.keys():
                    processed_node.__dict__[key] = vrml_node[key]

            if 'children' in vrml_node.keys():
                children = vrml_node['children']
                for child in children:
                    if isinstance(child, GenericObject):
                        if 'scale' in vrml_node.keys() and isinstance(child,GenericObject):
                            for grandchild in child.children:
                                grandchild.geo.scale(vrml_node['scale'])
                        processed_node.add_child(child)
                        child.parent = processed_node
                    else:
                        logger.debug("Ignoring transform child %s"%str(child))

        elif vrml_node.name == "Group":
            processed_node = GenericObject()
            if 'children' in vrml_node.keys():
                children = vrml_node['children']
                for child in children:
                    processed_node.add_child(child)


        elif vrml_node.name == "Humanoid":
            processed_node = Robot()
            processed_node.joint_names = vrml_node['joints']
            processed_node.segment_names = vrml_node['segments']
            body = vrml_node['humanoidBody'][0]
            processed_node.add_child(body)
            body.parent = processed_node

        elif vrml_node.name == "Segment":
            processed_node = GenericObject()
            if 'children' in vrml_node.keys():
                children = vrml_node['children']
                for child in children:
                    if isinstance(child, GenericObject):
                        processed_node.add_child(child)
                        child.parent = processed_node
                    else:
                        logger.debug("Ignoring segment child %s"%str(child))

        elif vrml_node.name == "Inline":
            vrml_path = vrml_node['url']
            if self.root_path:
                vrml_path = os.path.join(self.root_path, vrml_path)
            if os.path.isfile(vrml_path):
                processed_node = parse(vrml_path)[0]
            else:
                logger.exception("Couldnt find %s"%(vrml_path))

        elif vrml_node.name in  [ "ForceSensor"]:
            processed_node = GenericObject()

        elif vrml_node.name == "Joint":
            children = vrml_node['children']
            processed_node = Joint()
            for key in 'translation', 'rotation':
                if key in vrml_node.keys():
                    processed_node.__dict__[key] = vrml_node[key]

            if 'jointId' in vrml_node.keys():
                processed_node.id = vrml_node['jointId']
            if 'jointType' in vrml_node.keys():
                processed_node.jointType = vrml_node['jointType']
            if 'jointAxis' in vrml_node.keys():
                processed_node.axis = vrml_node['jointAxis']

            for child in children:
                if isinstance(child, GenericObject):
                    processed_node.add_child(child)
                    child.parent = processed_node
                else:
                    logger.debug("Ignoring child %s"%str(child))

        elif vrml_node.name == "VisionSensor":
            processed_node = Camera()
            processed_node.__dict__.update(vrml_node)

        if defname:
            self.def_dict[defname] = processed_node
            processed_node.name = defname
        return processed_node

    def name(self,(tag,start,stop,subtags), buffer ):
        return str(buffer[start:stop])

    def Attr(self,(tag,start,stop,subtags), buffer ):
        name = dispatch(self,subtags[0],buffer)
        value = dispatch(self,subtags[1],buffer)
        return (name, value)

    def Def(self,(tag,start,stop,subtags), buffer ):
        name = dispatch(self, subtags[0], buffer)
        return name

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
        s = buffer[start:stop]

        if s not in self.def_dict.keys():
            raise Exception("USE/IS used before node definition.")

        return self.def_dict[s]


    def USE(self,(tag,start,stop,subtags), buffer ):
        s = buffer[start:stop]

        if s not in self.def_dict.keys():
            raise Exception("USE/IS used before node definition.")

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
