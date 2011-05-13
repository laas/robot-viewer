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
from mathaux import *
import version

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
    global_configs = {}
    def __init__(self, *args, **kwargs):
        self.pendingObjects = []
        self.kinematic_elements = {}
        self.parse_config()

    def set_robot_joint_rank(self,robot_name, joint_rank_xml):
        """
        """
        if robot_name not in self.kinematic_elements.keys():
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

        for joint in self.kinematic_elements[robot_name].joint_list:
            if correct_joint_dict.has_key(joint.name):
                joint.id = correct_joint_dict[joint.name]

        self.kinematic_elements[robot_name].update_joint_dict()
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
        prog_version = distutils.version.StrictVersion(version.__version__)
        config = CustomConfigParser()
        config.read(self.config_file)
        logger.info( 'parsed_config %s'%config)

        if not config.has_section('global'):
            self.parse_configLegacy(config)
            return

        for section in config.sections():
            for options in config.options(section):
                value = self._replace_env_var(config.get(section,options))
                config.set(section, options, value)

        conf_version = distutils.version.StrictVersion(config.get('global','version'))
        if prog_version < conf_version:
            raise Exception("Your config version ({0}) is newer than program version ({1})".
                            format(conf_version, prog_version))

        value =  config.get('global','background')
        if value:
            value = [float(e) for e in value.split(",")]
            self.global_configs['background'] = value

        sections = config.sections()
        join_pairs = []

        for section in sections:
            words = section.split()
            otype = words[0]

            if otype in ["robot", "object"]:
                oname = words[1]
                if not words[1:]:
                    raise Exception("All robots must have a name.")

                geometry = config.get(section, 'geometry')
                if not geometry:
                    raise Exception("missing geometry section for {0}"
                                    .format(section))
                scale = config.get(section, 'scale')
                if not scale:
                    scale = "1 1 1"
                scale = [float(w) for w in scale.split()]

                joint_rank = config.get(section, 'joint_rank')
                if otype == "robot" and joint_rank:
                    self.set_robot_joint_rank( oname, joint_rank)

                position = config.get(section, 'position')
                if not position:
                    postion = 6*[0.0]
                else:
                    position = [float(e) for e in position.split()]

                self._create_element(otype, oname,
                                     geometry, scale)
                if position:
                    self.updateElementConfig(oname, position)
                parent = config.get(section, 'parent')

                if parent:
                    self.disableElement(oname)
                    parent_name = parent.split(",")[0]
                    parent_joint_ids = parent.split(",")[1:]

                    for parent_joint_id in parent_joint_ids:
                        if parent_joint_id != "":
                            parent_joint_id = int(parent_joint_id)
                        else:
                            parent_joint_id = None
                        name = "{0}_{1}_{2}".format(oname,parent_name,
                                                    parent_joint_id)
                        join_pairs.append((oname, name, parent_name,
                                           parent_joint_id))

        for pair in join_pairs:
            orig_name = pair[0]
            child_name = pair[1]
            parent_name = pair[2]
            parent_joint_id = pair[3]
            try:
                parent_obj = self.kinematic_elements[parent_name]
            except KeyError:
                logger.warning("Parent {0} does not exist. Skipping".
                               format(parent_name))
                continue
            new_el = copy.deepcopy( self.kinematic_elements[orig_name] )
            parent_obj.get_op_point(parent_joint_id).add_child(new_el)
            parent_obj.init()
            self.kinematic_elements[child_name] = new_el

        for name, obj in self.kinematic_elements.items():
            obj.update()

    def parse_configLegacy(self, config):
        logger.warning("Entering legacy config parsing")
        for section in config.sections():
            if section not in ['robots','default_configs',
                               'objects','joint_rank',
                               'preferences','scales']:
                raise Exception("Invalid section {0} in {1}".
                                format(section,self.config_file))

        scales = {}
        if config.has_section('scales'):
            for key, value in config.items('scales'):
                value = [float(e) for e in value.split(",")]
                scales[key] = value


        if config.has_section('robots'):
            robot_names = config.options('robots')
            for robot_name in robot_names:
                robot_config = config.get('robots',robot_name)
                robot_config = self._replace_env_var(robot_config)
                logger.info( 'robot_config=%s'%robot_config)
                if not os.path.isfile(robot_config):
                    logger.info( "WARNING: Couldn't load %s. Are you sure %s exists?"\
                        %(robot_name,robot_config))
                    continue
                self._create_element('robot',robot_name,robot_config,
                                     scales.get(robot_name))
                self.enableElement(robot_name)
        else:
            logger.info( """Couldn't any default robots.
            Loading an empty scene
            You might need to load some robots yourself.
            See documentation""")

        if config.has_section('joint_ranks'):
            robot_names = config.options('joint_ranks')
            for robot_name in robot_names:
                joint_rank_config = config.get('joint_ranks',robot_name)
                joint_rank_config = self._replace_env_var(joint_rank_config)
                if not self.kinematic_elements.has_key(robot_name):
                    continue
                if not os.path.isfile(joint_rank_config):
                    continue
                self.set_robot_joint_rank(self, robot_name,joint_rank_config)

        if config.has_section('objects'):
            object_names = config.options('objects')
            for object_name in object_names:
                object_file = config.get('objects',object_name)
                object_file = self._replace_env_var(object_file)
                if not os.path.isfile(object_file):
                    logger.warning('Could not find %s'%object_file)
                    continue
                self._create_element('object', object_name,
                                     object_file, scales.get(object_name))
                self.enableElement(object_name)

        if config.has_section('default_configs'):
            object_names = config.options('default_configs')
            for object_name in object_names:
                pos = config.get('default_configs',object_name)
                pos = [float(e) for e in pos.split()]
                self.updateElementConfig(object_name,pos)

        if config.has_section('preferences'):
            for key, value in config.items('preferences'):
                if key == 'background':
                    value = [float(e) for e in value.split(",")]
                    for win in self.windows:
                        glutSetWindow(win)
                        glClearColor (value[0], value[1], value[2], 0.5);

        return


    def _create_element(self, etype, ename, epath, scale = None):
        """
        Same as createElement but will not be called by outside world
        (CORBA) show will always be in the GL thread
        Arguments:
        - `self`:
        - `etype`:        string, element type (e.g. robot, object)
        - `name`:         string, element name
        - `path`:  string, description  (e.g. wrl path)
        """
        logger.debug("Creating {0} {1} {2} {3}".format(etype, ename, epath, scale))
        if self.kinematic_elements.has_key(ename):
            logger.exception("Element with that name exists already")
            return

        if etype == 'robot':
            objs = ml_parser.parse(epath)
            robots = []
            for obj in objs:
                if isinstance(obj, kinematics.Robot):
                    robots.append(obj)
            if len(robots) != 1:
                raise Exception("file %s contains %d robots, expected 1."
                                %(epath, len(robots)))
            new_robot = robots[0]
            if scale:
                new_robot.scale(scale)
            self.kinematic_elements[ename] = new_robot

        elif etype == 'object':
            new_object = kinematics.GenericObject()
            new_object.init()
            self.kinematic_elements[ename] = new_object


    def createElement(self, etype, ename, epath):
        """
        Same as _create_element but could be called by outside world
        (CORBA) show will always be in the GL thread
        Arguments:
        - `self`:
        - `etype`:        string, element type (e.g. robot, GLscript)
        - `name`:         string, element name
        - `path`:  string, path  (e.g. wrl path)
        """
        TIMEOUT = 600
        self.pendingObjects.append((etype, ename, epath))
        wait = 0
        while ename not in self.kinematic_elements.keys() and wait < TIMEOUT:
            time.sleep(0.1)
            wait += 0.1
        if wait >= TIMEOUT:
            logger.exception("Object took too long to create")
            return False
        return True

    def destroyElement(self,name):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self.kinematic_elements.has_key(name):
            logger.exception("Element with that name does not exist")
            return False

        del self.kinematic_elements[name]
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

    def updateElementConfig(self,name,config):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self.kinematic_elements.has_key(name):
            logger.exception("Element with that name does not exist")
            return False

        self.kinematic_elements[name].update_config(config)
        return True

    def getElementConfig(self,name):
        """
        Arguments:
        - `self`:
        - `name`:         string, element name
        """
        if not self.kinematic_elements.has_key(name):
            logger.exception(KeyError,
                             "Element with that name does not exist")
            return []
        cfg = self.kinematic_elements[name].get_config()
        if not isinstance(cfg, list):
            return []
        else:
            return cfg

    def listElements(self):
        return [name for name in self.kinematic_elements.keys() ]

    def listElementDofs(self, ename):
        l = ["X", "Y", "Z", "roll", "pitch", "yaw"]
        if not isinstance( self.kinematic_elements[ename], kinematics.GenericObject):
            return l

        obj = self.kinematic_elements[ename]
        if not isinstance(obj, kinematics.Robot):
            return l
        for j in obj.moving_joint_list:
            l += [j.name]
        return l

    def listMeshes(self):
        results = []
        for name, element in self.kinematic_elements.items():
            for mesh in element.mesh_list:
                results.append(mesh.uuid)
        return results

    def getGlConfig(self, uuid):
        Tmatrix = kinematics.all_objects[uuid].globalTransformation
        R=Tmatrix[0:3,0:3]
        p=Tmatrix[0:3,3]
        agax=rot2AngleAxis(R)
        return list(p) + list(agax)

    def run(self):
        while True:
            if len(self.pendingObjects) > 0:
                obj = self.pendingObjects.pop()
                logger.debug( "creating %s %s %s"%( obj[0], obj[1], obj[2]))
                self._create_element(obj[0],obj[1],obj[2])
            time.sleep(1e-3)

if __name__ == '__main__':
    server = KinematicServer()
    print server.listMeshes()
    print server.getMeshConfig(server.listMeshes()[0])
    server.run()

