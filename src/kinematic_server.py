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

import os
import kinematics
import ml_parser
import logging
import ConfigParser
import time, datetime
import subprocess
import collections
import math
import distutils.version
import numpy
import mathaux
import version
import copy
import camera
import re
import inspect, sys
logger = logging.getLogger("robotviewer.kinematic_server")
class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger.addHandler(NullHandler())

class CustomConfigParser(ConfigParser.ConfigParser):
    def get(self,*args, **kwargs ):
        try:
            return ConfigParser.ConfigParser.get(self, *args, **kwargs)
        except ConfigParser.NoOptionError:
            return None


class KinematicServer(object):
    """OpenGL server
    """
    config_dir = os.environ['HOME']+'/.robotviewer/'
    config_file = os.path.join(config_dir,"config")
    idle_cbs = []
    config = None
    def __init__(self, options = {}, args = []):
        self.pendingObjects = []
        self.elements = {}
        self.parse_config()
        self.robots = []
        for name, obj in self.elements.items():
            if isinstance(obj, kinematics.Robot):
                self.robots.append(obj)

        self.cameras = []

    @property
    def shapes(self):
        res = []
        for name, obj in self.elements.items():
            res += obj.shape_list
        return res

    def set_robot_joint_rank(self,robot_name, joint_rank_xml):
        """
        """
        if robot_name not in self.elements.keys():
            return False


        if not os.path.isfile(joint_rank_xml):
            return False

        pattern=re.compile(r"\s*<Link>\s*(\w+)\s*(\d+)\s*<\/Link>\s*")
        lines = open(joint_rank_xml).readlines()
        correct_joint_dict = dict()

        for line in lines:
            m = pattern.match(line)
            if m:
                correct_joint_dict[m.group(1)] = int(m.group(2)) -6
                logger.info( m.group(1)+ "\t" + m.group(2))

        for joint in self.elements[robot_name].joint_list:
            if correct_joint_dict.has_key(joint.name):
                joint.id = correct_joint_dict[joint.name]

        self.elements[robot_name].update_joint_dict()
        return True


    def _replace_env_var(self,s):
        matches = re.findall(r'\$(\w+)',s)
        for m in matches:
            var = m
            val = os.environ[var]
            s = s.replace(var,val)
        s = s.replace('$','')
        return s

    def parse_config(self):
        prog_version = distutils.version.StrictVersion(
            version.__version__)
        self.config = CustomConfigParser()
        self.config.read(self.config_file)
        logger.info( 'parsed_config %s'%self.config)

        if not self.config.has_section('global'):
            self.parse_configLegacy(config)
            return

        for section in self.config.sections():
            for options in self.config.options(section):
                value = self._replace_env_var(self.config.get(section,options))
                self.config.set(section, options, value)

        conf_version = (distutils.version.StrictVersion
                        (self.config.get('global','version')))
        if prog_version < conf_version:
            raise Exception(("Your config version ({0}) "+
                             "is newer than program version ({1})").
                            format(conf_version, prog_version))

        sections = self.config.sections()
        join_pairs = []

        obj_tree = []

        for section in sections:
            words = section.split()
            otype = words[0]

            if otype in ["robot", "object"]:
                parser_file = self.config.get(section, "parser")
                if not parser_file:
                    parser = None
                else:
                    path , module = os.path.split(parser_file)
                    sys.path.append(path)
                    module = module.replace(".py","")
                    parser = __import__(module)
                exclude_cameras = self.config.get(section, 'excl_cams')
                if not words[1:]:
                    raise Exception("All robots must have a name.")
                oname = words[1]
                print oname, exclude_cameras

                geometry = self.config.get(section, 'geometry')
                if not geometry:
                    raise Exception("missing geometry section for {0}"
                                    .format(section))
                scale = self.config.get(section, 'scale')
                if not scale:
                    scale = "1 1 1"
                scale = [float(w) for w in scale.split()]

                joint_rank = self.config.get(section, 'joint_rank')
                if otype == "robot" and joint_rank:
                    self.set_robot_joint_rank( oname, joint_rank)

                position = self.config.get(section, 'position')
                if not position:
                    postion = 6*[0.0]
                else:
                    position = [float(e) for e in position.split()]

                parent = self.config.get(section, 'parent')
                if not parent:
                    self._create_element(otype, oname,
                                     geometry, scale, parser)
                    self.elements[oname].exclude_cameras = exclude_cameras
                    if position:
                        self.updateElementConfig(oname, position)
                else:
                    parent_name = parent.split(",")[0]
                    parent_joint_ids = parent.split(",")[1:]

                    for parent_joint_id in parent_joint_ids:
                        if parent_joint_id != "":
                            parent_joint_id = int(parent_joint_id)
                        else:
                            parent_joint_id = None
                        child_name = "{1}:{2}/{0}".format(oname,
                                                          parent_name,
                                                    parent_joint_id)
                        self._create_element( otype, child_name,
                                              geometry, scale, parser)
                        obj_tree.append((child_name,
                                         parent_name, parent_joint_id))

                        if not position:
                            position = 6*[0.]

                        self.updateElementConfig(child_name, position)
                        self.elements[child_name].exclude_cameras \
                            = exclude_cameras
                        #self.elements[child_name].origin.init()


        for child_name, parent_name, parent_joint_id in obj_tree:
            (self.elements[parent_name].get_op_point(parent_joint_id)
             .add_child(self.elements[child_name]))


        for name, obj in self.elements.items():
            obj.update()

    def parse_configLegacy(self, config):
        legacy.parse_config(self, config)


    def _create_element(self, etype, ename, epath,
                        scale = None, parser = None):
        """
        Same as createElement but will not be called by outside world
        (CORBA) show will always be in the GL thread
        Arguments:
        - `self`:
        - `etype`:        string, element type (e.g. robot, object)
        - `name`:         string, element name
        - `path`:  string, description  (e.g. wrl path)
        """

        if not parser:
            parser = ml_parser

        logger.debug("Creating {0} {1} {2} {3}".format(etype, ename,
                                                       epath, scale))
        if self.elements.has_key(ename):
            logger.exception("Element with the name '"+ ename +"' does not exist")
            return

        if etype == 'robot':
            objs = parser.parse(epath)
            robots = []
            for obj in objs:
                if isinstance(obj, kinematics.Robot):
                    robots.append(obj)
            if len(robots) != 1:
                raise Exception("file %s contains %d robots, expected 1."
                                %(epath, len(robots)))
            new_robot = robots[0]
            self.elements[ename] = new_robot
            cameras = new_robot.get_list(camera.Camera)
            for cam in cameras:
                self.elements[cam.name] = cam


        elif etype == 'object':
            ext = os.path.splitext(epath)[1].replace(".","")
            if ext in parser.supported_extensions:
                logger.debug(("Creating element from supported "+
                              "markup language file %s.")%epath)
                all_objs = parser.parse(epath)
                objs = [ o for o in all_objs
                         if isinstance(o, kinematics.GenericObject)]
                if len(objs) == 0:
                    raise Exception('Found no object in file {0}. \1{1}'
                                    .format(epath, all_objs))

                elif len(objs) == 1:
                    new_object = objs[0]
                else:
                    new_object  = kinematics.GenericObject()
                    for obj in objs:
                        new_object.add_child(obj)

            elif ext == "py":
                from vrml.script import Script
                new_object = kinematics.Shape()
                new_object.geometry = Script(open(epath).read())
                new_object.init()
            else:
                new_object = kinematics.GenericObject()

            self.elements[ename] = new_object
            new_object.name = ename
            if scale:
                new_object.scale = scale
            new_object.init()


    def createElement(self, *args, **kwargs):
        self._create_element(*args, **kwargs)
        return True

    def destroyElement(self,name):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self.elements.has_key(name):
            logger.exception("Element with the name '"+ name +"' does not exist")
            return False

        del self.elements[name]
        return True

    def enableElement(self,name):
        """

        Arguments:
        - `self`:
        - `name`:
        """
        return True


    def disableElement(self,name):
        """
        Arguments:
        - `self`:
        - `name`:
        """
        return True

    def setTransparency(self,name, t):
        """
        Arguments:
        - `self`:
        - `name`:
        """
        return True


    def updateElementConfig(self,name,config):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self.elements.has_key(name):
            logger.exception("Element with the name '"+ name +"' does not exist")
            return False

        self.elements[name].update_config(config)
        return True

    def getElementConfig(self,name):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self.elements.has_key(name):
            logger.exception(KeyError,
                             "Element with the name '"+ name +"' does not exist")
            return []
        cfg = self.elements[name].get_config()
        if not isinstance(cfg, list):
            return []
        else:
            return cfg

    def updateElementConfig2(self,name, T, q ):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self.elements.has_key(name):
            logger.exception("Element with that name does not exist")
            return False

        self.elements[name].update_config2(T, q)
        return True

    def test(self):
        return []

    def getElementConfig2(self,name):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self.elements.has_key(name):
            logger.exception(KeyError,
                             "Element with that name does not exist")
            return [],[]
        return self.elements[name].get_config2()


    def listElements(self):
        return [name for name in self.elements.keys() ]

    def listElementDofs(self, ename):
        l = ["X", "Y", "Z", "roll", "pitch", "yaw"]
        if not isinstance( self.elements[ename],
                           kinematics.GenericObject):
            return l

        obj = self.elements[ename]
        if not isinstance(obj, kinematics.Robot):
            return l
        for j in obj.moving_joint_list:
            l += [j.name]
        return l


    def setScale(self, name, scale):
        self.elements[name].scale = scale
        self.elements[name].init()
        return True

    def listShapees(self):
        results = []
        for name, element in self.elements.items():
            for shape in element.shape_list:
                results.append(shape.uuid)
        return results

    def printShape(self, uuid):
        import json
        try:
            shape = [ m for m in self.shapes if m.uuid == uuid][0]
            logger.exception("Invalid key, available keys are %s"
                             %str([m.uuid for m in self.shapes])
                             )
            return json.dumps({'vertexPositions' : shape.geo.coord,
                               'indices' : shape.geo.tri_idxs,
                               'vertexNormals' : shape.geo.norm,
                              })
        except KeyError:
            logger.exception("printShape failed")
            return json.dumps({})

    def getGlConfig(self, uuid):
        shape = [ m for m in self.shapes if m.uuid == uuid][0]
        Tmatrix = shape.globalTransformation
        R=Tmatrix[0:3,0:3]
        p=Tmatrix[0:3,3]
        agax=rot2AngleAxis(R)
        res = list(p) + list(agax)
        return [float(e) for e in res]

    def getDofNames(self):
        return "FIXME"

    def getTransformations(self):
        import yaml
        d = {}
        for name, obj in self.elements.items():
            do = {}
            d[str(name)] = do
            for desc in obj.descendants:
                if not desc.name:
                    continue
                do[str(desc.name)] = desc.T.tolist()
        return yaml.dump(d)
    def run(self):
        return


if __name__ == '__main__':
    import optparse, sys
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    sh = logging.StreamHandler()
    sh.setLevel(logging.INFO)
    logger.addHandler(sh)
    formatter = logging.Formatter("%(name)s:%(levelname)s:%(message)s")
    sh.setFormatter(formatter)
    __version__ = 1.9
    parser = optparse.OptionParser(
        usage='\n\t%prog [options]',
        version='%%prog %s' % __version__)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")
    (options, args) = parser.parse_args(sys.argv[1:])
    if options.verbose:
        sh.setLevel(logging.DEBUG)

    server = KinematicServer()
    #print server.listShapees()
    #print server.getGlConfig(server.listShapees()[0])
    print server.getTransformations()
    server.run()
