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

import vrml.parser
import logging, sys
logger = logging.getLogger("robotviewer.vrml_parser")
import kinematics
import geometry
import vrml.standard_nodes as nodes
from collections import defaultdict
import camera

class_map ={
    "Humanoid" : kinematics.Robot,
    "Group" : kinematics.GenericObject,
    "Joint" : kinematics.Joint,
    "Segment": kinematics.GenericObject,
    "Inline": kinematics.GenericObject,
    "Transform": kinematics.GenericObject,
    "Shape": kinematics.Shape,
    "Coordinate": nodes.Coordinate,
    "Appearance": nodes.Appearance,
    "Material": nodes.Material,
    "Normal": nodes.Normal,
    "Color": nodes.Color,
    "VisionSensor": camera.Camera,
    "IndexedFaceSet" : geometry.IndexedFaceSet,
    "IndexedLineSet" : geometry.IndexedLineSet,
    "ElevationGrid" : geometry.ElevationGrid,
    "Box" : geometry.Box,
    "Cone" : geometry.Cone,
    "Cylinder" : geometry.Cylinder,
    "Extrusion" : geometry.Extrusion,
    "Sphere" : geometry.Sphere,
    "PointSet" : geometry.PointSet,
    "Text" : geometry.Text,
    }

ignored_classes = defaultdict(int)

class UnknownNode(Exception):
    pass


def convert(obj):
    clsname =  obj.__class__.__name__
    # logger.debug("Converting {0} ({1})".format(clsname, repr(obj)[:100]))

    if not clsname in class_map.keys():
        if clsname != "NoneType" and clsname[0].isupper():
            logger.exception("Ignoring node {0}".format(clsname))
            ignored_classes[clsname] += 1
            raise UnknownNode()
        elif clsname == "list":
            return [convert(o) for o in obj]
        else:
            return obj
    res = class_map[clsname]()

    keys = [ key for key in dir(obj)
             if not (key.startswith('_')
                     or key == "children" or callable(getattr(obj, key))
                     )]
    for key in keys:
        try:
            value = convert(getattr(obj, key))
        except UnknownNode:
            continue

        if not key in dir(res):
            continue

        if callable(getattr(res, key)):
            continue
        try:
            res.__dict__[key] = value
        except AttributeError:
            logger.debug("Ignoring key {0} while converting {1}".format(key, repr(obj)[:100]))

    for child in obj.children:
        try:
            converted_child = convert(child)
        except UnknownNode:
            continue

        logger.debug("Adding {0} {1} to {2}".format(type(converted_child),
                                             id(converted_child), id(res)))
        res.add_child(converted_child)

    return res

def parse(filename):
    vrml_objs = vrml.parser.parse(filename)
    res = []
    for obj in vrml_objs:
        if isinstance(obj, type):
            continue
        try:
            converted = convert(obj)
        except UnknownNode:
            continue
        if (isinstance(converted, kinematics.GenericObject)):
            res.append(converted)
            converted.init()

    if ignored_classes.keys():
        s = "The following nodes have not been processed:\n"
        for clsname, no in ignored_classes.items():
            s += "{0} ({1} instances)\n".format(clsname, no)
        s += "If these nodes affect your scene, add them to {0} or contact robot-viewer's maintainer".format(__file__)
        logger.warning(s)

    return res

def main():
    import optparse
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    logger.handlers = []
    formatter = logging.Formatter("%(asctime)s:%(name)s:%(levelname)s:%(message)s",
                                  )

    sh = logging.StreamHandler()
    logger.addHandler(sh)
    sh.setFormatter(formatter)
    sh.setLevel(logging.INFO)
    parser = optparse.OptionParser(
        usage='\n\t%prog [options]',
        version='%%prog %s' % __version__)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")
    (options, args) = parser.parse_args(sys.argv[1:])
    if options.verbose:
        sh.setLevel(logging.DEBUG)
        logger.setLevel(logging.DEBUG)
    res = parse(args[0])

    for r in res:
        print r
        continue

if __name__ == '__main__':
    main()
