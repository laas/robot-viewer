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

import numpy
import numpy.linalg
import re
from math import sin,cos,isnan, pi
import transformations as tf
from collections import deque
import logging, uuid
import __builtin__
import traceback
import vrml.standard_nodes as nodes
import mathaux
class NullHandler(logging.Handler):
    def emit(self, record):
        pass
from display_object import DisplayObject

logger = logging.getLogger("robotviewer.kinematics")
logger.addHandler(NullHandler())

BASE_NODE_ID = -1

def find_relative_transformation( obj1, obj2 ):
    """
    Find transformation matrix of obj2 in reference frame of obj1
    """
    res = numpy.dot( numpy.linalg.inv(obj1.globalTransformation),
                      obj2.globalTransformation
                     )
    return res

def listify(a):
    return [list(r) for r in a]

class GenericObject(DisplayObject):
    """
    Base element in the kinematic tree
    """
    count = 0
    def __init__(self):
        DisplayObject.__init__(self)
        self.name=None
        self.jointType=""
        self.translation = [0,0,0]
        self.rpy=[0,0,0]
        self.rotation = [1,0,0,0]
        self.center=[0,0,0]
        self.parent=None
        self.children=[]
        self.localTransformation = numpy.identity(4)
        self.globalTransformation = numpy.identity(4)
        self.list_by_type = {}
        self.uuid = str(uuid.uuid1())
        self.scale = [1., 1., 1.]
        self._R_axis_angle = [1., 0., 0., 0.]

    def __del__(self):
        for child in self.children:
            del child
        object.__del__(self)


    def cumul_scale(self):
        if not self.parent:
            return self.scale
        parent_scale = self.parent.cumul_scale()
        return [self.scale[i]*parent_scale[i] for i in range(3)]

    def get_list(self, type):
        try:
            return self.list_by_type[type]
        except KeyError:
            return []

    @property
    def origin(self):
        if self.parent == None:
            return self
        return self.parent.origin

    @property
    def shape_list(self):
        return self.get_list(Shape)

    @property
    def cam_list(self):
        return self.get_list(camera.Camera)


    @property
    def joint_list(self):
        return self.get_list(Joint) + self.get_list(Robot)


    def get_op_point(self, id):
        if not isinstance(self, Robot):
            return self

        if id == None:
            return self.waist
        else:
            return self.moving_joint_list[id]

    def update_config(self,config):
        self.count += 1
        self.translation = config[:3]
        self.rpy = config[3:6]
        rot_mat = tf.euler_matrix(self.rpy[0], self.rpy[1], self.rpy[2])
        angle, direction, point = tf.rotation_from_matrix(rot_mat)
        self.rotation = [ direction[0],direction[1],direction[2], angle ]
        self.init_local_transformation()
        print self.name, config, self.localTransformation
        self.origin.update()

    def update_config2(self, T, q):
        self.count += 1
        T = numpy.array(T)
        self.localTransformation = T
        self.translation = T[:3,3]
        angle, direction, point = tf.rotation_from_matrix(T)
        self.rpy = tf.euler_from_matrix(T)
        self.rotation = [ direction[0],direction[1],direction[2], angle ]
        self.origin.update()


    def get_config(self):
        if type(self) not in [Robot, GenericObject]:
            return self.translation + list(self.rpy)
        rpy = tf.euler_from_matrix(self.globalTransformation)
        return self.translation + list(rpy)


    def get_config2(self):

        if type(self) not in [Robot, GenericObject]:
            res = listify(self.localTransformation), []
        else:
            res = listify(self.globalTransformation), []
        return res

    def __str__(self):
        s= "%s \t= %s\n"%(self.__class__.__name__,self.name)
        s+="jointType\t= %s\n"%self.jointType
        s+= "id\t\t= "+str(id(self))
        s+= "\ntranslation\t= "+str(self.translation)
        s+= "\nrotation\t= "+str(self.rotation)

        s+= "\nPARENT\t\t= "
        if self.parent:
            parent=self.parent
            s+= "%s "%(parent.type) + str(parent.name) +" id="+str(id(parent))+";  "

        s+= "\nCHILDREN (%d)\t= "%len(self.children)
        for child in self.children:
            s+= "%s "%(child.type) + str(child.name) +" id="+str(id(child))+";  "

        s+= "\nT=\n"+str(self.globalTransformation)
        s+= "\nlocalT=\n"+str(self.localTransformation)

        return s

    # def scale(self, scale):
    #     for i in range(3):
    #         self.translation[i]*=scale[i]
    #         sc = scale[i]
    #         self.localTransformation[i,3] *= sc

    #     for child in self.children:
    #         child.scale(scale)


    def add_child(self,a_child):
        """
        Add a :class:`kinematics.GenericObject` object to element's children list
        """
        self.children.append(a_child)
        a_child.parent = self

    def set_parent(self,parent):
        """
        Set a :class:`kinematics.GenericObject` object as element's parent
        """
        self.parent=parent


    def update_local_transformation(self):
        """ update local transformation w.r.t element's parent. Nothing to do,
        since nothing varies for a GenericObject

        .. seealso:: VRML transform calculation http://www.web3d.org/x3d/specifications/vrml/ISO-IEC-14772-VRML97/part1/nodesRef.html#Transform

        """
        pass

    def init_local_transformation(self):
        """
        compute local transformation w.r.t for the first time (compute everything)
        """
        # rotation part
        self.localTransformation = numpy.eye(4)
        # print type(self), id(self), self.rotation
        try:
            angle = self.rotation[3]
        except IndexError:
            logger.exception("Failed on {0}, rotation={1}".format(type(self),self.rotation))
            raise
        direction = self.rotation[:3]
        self.localTransformation[0:3,0:3] = tf.rotation_matrix(angle, direction)[:3,:3]
        self.rpy = tf.euler_from_matrix(self.localTransformation)

        # last column
        scale = [1,1,1]
        if self.parent:
            scale = self.cumul_scale()
        scale_translation = [self.translation[i]*scale[i] for i in range(3)]

        self.localTransformation[0:3,3] = numpy.array(scale_translation)+\
            numpy.dot(numpy.eye(3)-self.localTransformation[:3,:3],
                      numpy.array(self.center))

        # last line
        self.localTransformation[3,0:4]=[0,0,0,1]


    def update_global_transformation(self):
        """ update position and orientation in the world coordinates based on
        local transformation and parent's global transformation
        """
        if self.parent == None or self.parent.globalTransformation == None:
            self.globalTransformation = self.localTransformation
        else:
            self.globalTransformation=numpy.dot\
                (self.parent.globalTransformation,self.localTransformation)

        return


    def init(self):
        """ Do the following initializations in order:

         * call :func:`kinematics.GenericObject.init_local_transformation` on itself
         * call init() on all children
        """
        logger.debug("Initializing {0}".format(id(self)))
        try:
            self.init_local_transformation()
        except:
            logger.exception("Failed to init {0}".format(self))
            raise
        self.update_global_transformation()
        def _union(l1, l2):
            return list(set(l1).union(set(l2)))

        self.list_by_type[type(self)] = [ self ]
        for child in self.children:
            try:
                child.init()
                for key in child.list_by_type.keys():
                    if key not in self.list_by_type.keys():
                        self.list_by_type[key] = child.list_by_type[key]
                    else:
                        self.list_by_type[key] = _union(child.list_by_type[key],
                                                        self.list_by_type[key],)
            except Exception, error:
                print error, "on object %s"%child.name
        self.remove_generic_objects()
        self.update()

    def remove_generic_objects(self):
        generic_children = [child for child in self.children
                            if (type(child) == GenericObject and child.children[:])]
        while generic_children[:]:
            for child in generic_children:
                for grandchild in child.children[:]:
                    grandchild.localTransformation = numpy.dot(child.localTransformation,
                                                              grandchild.localTransformation)
                    grandchild.translation =  grandchild.localTransformation[:3,3]
                    angle, direction, point = tf.rotation_from_matrix(grandchild.localTransformation)
                    grandchild.rotation = list(direction) + [angle]
                    grandchild.rpy = tf.euler_from_matrix(grandchild.localTransformation)
                    grandchild.scale = [grandchild.scale[i]*child.scale[i] for i in range(3)]
                    self.add_child(grandchild)
                self.children.remove(child)
                del child
            generic_children = [child for child in self.children
                                if (type(child) == GenericObject and child.children[:])]
        for child in self.children:
            child.remove_generic_objects()

    def update(self):
        """ Do the following initializations in order:

         * call :func:`kinematics.GenericObject.update_local_transformation` on itself
         * call :func:`kinematics.GenericObject.update_global_transformation` on itself
         * call update() on all children
        """
        self.update_local_transformation()
        self.update_global_transformation()
        for child in self.children:
            child.update()

    def get_robot(self):
        """
        Get the highest level parent

        :returns: the Robot
        :rtype: :class:`kinematics.Robot`.
        """
        if self.parent==None:
            return self
        else:
            return self.parent.get_robot()

    def get_parent_joint(self):
        """
        Get the immediate parent joint

        :returns: the Robot
        :rtype: :class:`kinematics.Joint`.
        """
        if not self.parent:
            return None
        elif isinstance(self.parent,Joint):
            return self.parent
        else:
            return self.parent.get_parent_joint()

    def get_children_joints(self):

        children = []
        for child in self.children:
            if isinstance(child, Joint):
                children.append(child)
            else:
                children += child.get_children_joints()
        return children

#*****************************#
#           JOINT             #
#*****************************#
import alias
class Joint(GenericObject):
    """
    Joint class

    .. todo:: merge this with generic object or offload stuff from generic object to this
    """
    def __init__(self):
        GenericObject.__init__(self)
        self.jointType=None
        self.isRobot=False
        self.angle=0
        self.localR1=numpy.eye(3)  # due to cordinate offset
        self.localR2=numpy.eye(3)  # due to self rotation (revolute joint)
        self.op_point = GenericObject()
        self.add_child(self.op_point)
        self.jointId = None
        self.jointAxis = ""


    def __str__(self):
        s = GenericObject.__str__(self)
        s+= "\naxis\t\t= "+str(self.jointAxis)
        s+= "\nangle\t\t= "+str(self.angle)
        return s

    def update_local_transformation(self):
        """
        update local transformation w.r.t element's parent

        .. seealso:: VRML transform calculation http://www.web3d.org/x3d/specifications/vrml/ISO-IEC-14772-VRML97/part1/nodesRef.html#Transform
        """
        if self.jointType in ["free", "freeflyer"]:
            self.localTransformation = tf.euler_matrix(self.rpy[0], self.rpy[1], self.rpy[2])

            scale = [1,1,1]
            if self.parent:
                scale = self.cumul_scale()
            scale_translation = [self.translation[i]*scale[i] for i in range(3)]
            self.localTransformation[0:3,3]=numpy.array(scale_translation)+\
                numpy.dot(numpy.eye(3)-self.localTransformation[0:3,:3],numpy.array(self.center))

        elif ( type(self) == Joint and self.jointType in [ "rotate", "rotation", "revolute"]
               and self.jointId >= 0):
            if self.jointAxis in ["x","X"]:
                axis = [1, 0, 0]
            elif self.jointAxis in ["y","Y"]:
                axis = [0, 1, 0]
            elif self.jointAxis in ["z","Z"]:
                axis = [0, 0, 1]
            else:
                raise RuntimeError("Invalid joint axis {0}".format(self.jointAxis))
            self.localR2= tf.rotation_matrix(self.angle, axis)[:3,:3]
            self.localTransformation[0:3,0:3] = numpy.dot(self.localR1, self.localR2)

    def init_local_transformation(self):
        """
        compute local transformation w.r.t for the first time (compute everything)
        """
        self.localTransformation = numpy.eye(4)
        self.globalTransformation = numpy.eye(4)
        angle = self.rotation[3]
        direction = self.rotation[:3]
        self.localR1 = tf.rotation_matrix(angle, direction)[:3,:3]
        self.localTransformation[0:3,0:3] = self.localR1
        self.update_local_transformation()
        scale = [1,1,1]
        if self.parent:
            scale = self.cumul_scale()
        scale_translation = [self.translation[i]*scale[i] for i in range(3)]
        self.localTransformation[0:3,3]=numpy.array(scale_translation)+\
            numpy.dot(numpy.eye(3)-self.localTransformation[:3,:3],
                      numpy.array(self.center))
        self.localTransformation[3,3]=1


class Robot(Joint):
    """
    Typically represent a robot/model. First element in the kinematic chain
    """
    def __init__(self):
        Joint.__init__(self)
        self.ndof = 0
        self.jointId= BASE_NODE_ID
        self.joint_dict= {}
        self.joint_names = []
        self.segment_names = []
        self.waist=None
        self.moving_joint_list = []

    def __str__(self):
        s = Joint.__str__(self)
        s += "\nNumber of dof: %d"%self.ndof
        s += "\nNumber shapes: %d"%len(self.shape_list)
        return s

    def update_config(self, config):
        self.set_angles(config[6:])
        self.waist.translation = config[0:3]
        self.waist.rpy = config[3:6]
        self.init_local_transformation()
        self.update()

    def update_config2(self, T, q):
        if q != []:
            self.set_angles(q)
        if T != []:
            self.waist.update_config2(T,  [])
        self.update()

    def set_angles(self,angles):
        """
        Set joint angles

        :param angles: input joint angles
        :type angles: list of double
        """
        if len(angles) != self.ndof:
            raise Exception("Wrong dimension. Expected %d dof but got %d dof"%(self.ndof,
                                                                                  len(angles)) )
        for i,angle in enumerate(angles):
            self.moving_joint_list[i].angle = angle

    def print_joints(self):
        """
        Print all joints
        """
        for joint in self.joint_list:
            print "\n====\n",joint

    def get_config(self):
        vec = self.waist.translation
        vec += self.waist.rpy
        for i in range(len(self.joint_list)):
            if self.joint_dict.has_key(i):
                vec += [self.joint_dict[i].angle]
        return vec

    def get_config2(self):
        vec = []
        for i in range(len(self.joint_list)):
            if self.joint_dict.has_key(i):
                vec += [self.joint_dict[i].angle]
        return listify(self.waist.globalTransformation), vec

    def waist_pos(self,p):
        """
        Set waist position

        :param p: input position
        :type p: 3-double turple
        """
        self.waist.translation=p

    def waist_rpy(self,ori):
        """
        Set waist orientation

        :param ori: input raw, pitch, yaw angles
        :type ori: 3-double turple
        """
        self.waist.rpy=ori

    def update_joint_dict(self):
        self.joint_dict = dict()
        for joint in self.joint_list:
            self.joint_dict[joint.jointId] = joint

    def update_moving_joint_list(self):
        for j in self.joint_list:
            if j.jointId != None:
                j.jointId = int(j.jointId)
        self.moving_joint_list = [ j for j in self.joint_list if isinstance(j.jointId,int)
                                   and j.jointId != BASE_NODE_ID ]
        self.moving_joint_list.sort(key = lambda x: x.jointId)
        self.ndof = len(self.moving_joint_list)

    def init(self):
        """ Do the following initializations in order:

         * build joint_list and shape_list
         * call :func:`kinematics.Joint.init_local_transformation` on itself
         * call init() on all children
        """
        GenericObject.init(self)
        self.update()

        for joint in self.joint_list:
            if joint.jointType in ["free","freeflyer"]:
                self.waist=joint

        if not self.waist:
            raise Exception("Couldn't find the waist joint (freeflyer)")

        self.update_joint_dict()

        self.init_local_transformation()
        self.update_global_transformation()
        self.update_moving_joint_list()

        for shape in self.shape_list:
            if not shape.appearance:
                shape.appearance = nodes.Appearance()
            if not shape.appearance.material:
                shape.appearance.material = nodes.Material()

            shape.appearance.material.transparency = 0


        if not self.shape_list[:]:
            logger.warning("Robot contains 0 shape.")



class Shape(GenericObject, nodes.Shape):
    def __init__(self):
        GenericObject.__init__(self)
        nodes.Shape.__init__(self)
        self.appearance = nodes.Appearance()
        self.appearance.mateiral = nodes.Material()
        self.name=None
        self.localR1=numpy.eye(3)  # due to offset of coordonee

    def init(self):
        GenericObject.init(self)
        logger.debug("Computing normal for Shape {0}".format(id(self)))
        self.geometry.init()


    # def scale(self, scale_vec):
    #     GenericObject.scale(self,scale_vec)
    #     self.geometry.scale(scale_vec)

    def __str__(self):
        s=""
        s+="\nrotation="+str(self.rotation)
        s+="\ntranslation="+str(self.translation)
        s+="Apparence: %s\n"%str(self.appearance)
        s+="\n"
        s+="Geometry: %s\n"%str(self.geometry)
        return s

    def bounding_box_local(self):
        no_points = len(self.geometry.coord)/3
        xs = [self.geometry.coord[3*i] for i in range(no_points)]
        ys = [self.geometry.coord[3*i+1] for i in range(no_points)]
        zs = [self.geometry.coord[3*i+2] for i in range(no_points)]
        bbox =  [ [min(xs), min(ys), min(zs)],
                  [max(xs), max(ys), max(zs)]
                   ]
        return bbox

    def bounding_box_global(self):
        bbox = self.bounding_box_local()
        bbox[0] = numpy.dot(self.globalTransformation,
                         numpy.array(bbox[0] + [0]))[:3]
        bbox[1] = numpy.dot(self.globalTransformation,
                         numpy.array(bbox[1] + [0]))[:3]
        return bbox

    bounding_box = bounding_box_local


import camera
