#! /usr/bin/env python

__author__ = "Duong Dang"
__version__ = "0.1"

import logging, os, sys
logger = logging.getLogger("vrml_parser")
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
ch.setFormatter(formatter)
logger.addHandler(ch)

from simpleparse.common import numbers, strings, comments
from simpleparse.parser import Parser
from simpleparse.dispatchprocessor import *
from vrml_grammar import VRMLPARSERDEF
import pprint

from mesh import Mesh, Appearance, Geometry, Shape
from robo import GenericObject, Joint, BaseNode

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
            mat = vrml_node['material']
            processed_node.__dict__.update(vrml_node['material'][0])

        elif vrml_node.name == "IndexedFaceSet":
            processed_node = Geometry()
            processed_node.coord = vrml_node['coord'][0]['point']
            processed_node.idx = vrml_node['coordIndex']

        elif vrml_node.name == "Humanoid":
            processed_node = BaseNode()
            processed_node.joint_names = vrml_node['joints']
            processed_node.segment_names = vrml_node['segments']
            body = vrml_node['humanoidBody'][0]
            processed_node.addChild(body)
            body.parent = processed_node

        elif vrml_node.name == "Transform":
            processed_node = GenericObject()
            for key in 'translation', 'rotation':
                if key in vrml_node.keys():
                    processed_node.__dict__[key] = vrml_node[key]

            shapes = []
            for child in vrml_node['children']:
                if ( isinstance(child,dict) and 'appearance' in child.keys()
                     and 'geometry' in child.keys()):
                    sh = Shape()
                    sh.app = child['appearance'][0]
                    sh.geo = child['geometry'][0]
                    shapes.append(sh)

            if shapes[:]:
                mesh = Mesh()
                mesh.shapes = shapes
                processed_node.children.append(mesh)
                mesh.parent = processed_node


        elif vrml_node.name == "Segment":
            processed_node = GenericObject()
            if 'children' in vrml_node.keys():
                children = vrml_node['children']
                for child in children:
                    if isinstance(child, GenericObject):
                        processed_node.addChild(child)
                        child.parent = processed_node
                    else:
                        logger.debug("Ignoring segment child %s"%str(child))

        elif vrml_node.name == "Inline":
            vrml_path = vrml_node['url'][0]
            if self.root_path:
                vrml_path = os.path.join(self.root_path, vrml_path)
            if os.path.isfile(vrml_path):
                processed_node = parse(vrml_path)[0]
            else:
                raise Exception("Couldnt find %s"%(vrml_path))

        elif vrml_node.name in  [ "ForceSensor", "VisionSensor"]:
            processed_node = GenericObject()

        elif vrml_node.name == "Joint":
            children = vrml_node['children']
            processed_node = Joint()
            for key in 'translation', 'rotation':
                if key in vrml_node.keys():
                    processed_node.__dict__[key] = vrml_node[key]

            if 'jointId' in vrml_node.keys():
                processed_node.id = vrml_node['jointId'][0]
            if 'jointType' in vrml_node.keys():
                processed_node.jointType = vrml_node['jointType'][0]
            if 'jointAxis' in vrml_node.keys():
                processed_node.axis = vrml_node['jointAxis'][0]

            for child in children:
                if isinstance(child, GenericObject):
                    processed_node.addChild(child)
                    child.parent = processed_node
                else:
                    logger.debug("Ignoring child %s"%str(child))

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
        return dispatchList(self, subtags, buffer)

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
    return objs


def main():
    import optparse
    parser = optparse.OptionParser(
        usage='\n\t%prog [options]',
        version='%%prog %s' % __version__)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")
    (options, args) = parser.parse_args(sys.argv[1:])
    res = parse(args[0])
    for r in res:
        if isinstance(r,BaseNode):
            r.init()
            print r
            print r.mesh_list

if __name__ == '__main__':
    main()
