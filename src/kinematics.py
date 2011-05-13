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
from math import sin,cos
from mathaux import *
from collections import deque
import logging

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("robotviewer.kinematics")
logger.addHandler(NullHandler())

BASE_NODE_ID = -1

all_objects = {}

def register_object(obj):
    obj.uuid = 0
    global all_objects
    while obj.uuid in all_objects.keys():
        obj.uuid += 1
    all_objects[obj.uuid] = obj
    return obj.uuid

def find_relative_transformation( obj1, obj2 ):
    """
    Find transformation matrix of obj2 in reference frame of obj1
    """
    res = numpy.dot( numpy.linalg.inv(obj1.globalTransformation),
                      obj2.globalTransformation
                     )
    return res

class GenericObject(object):
    """
    Base element in the kinematic tree
    """
    def __init__(self):
        self.type="GenericObject"
        self.name=None
        self.jointType=""
        self.translation=[0,0,0]
        self.rpy=[0,0,0]
        self.rotation=[1,0,0,0]
        self.center=[0,0,0]
        self.parent=None
        self.children=[]
        self.localTransformation = None
        self.globalTransformation = None
        self.localR=numpy.eye(3)
        self.id=None
        self.list_by_type = {}
        register_object(self)
    def get_list(self, type):
        try:
            return self.list_by_type[type]
        except KeyError:
            return []

    @property
    def mesh_list(self):
        return self.get_list(Mesh)

    @property
    def joint_list(self):
        return self.get_list(Joint)

    def get_op_point(self, id):
        if not isinstance(self, Robot):
            return self

        if id == None:
            return self.waist
        else:
            return self.moving_joint_list[id]

    def update_config(self,config):
        self.translation = config[:3]
        self.rpy = config[3:6]
        self.rotation = euleur2AxisAngle(self.rpy)
        self.init_local_transformation()
        self.update()

    def get_config(self):
        return self.translation + rot2rpy(self.globalTransformation)

    def __str__(self):
        s= "%s \t= %s\n"%(self.type,self.name)
        s+="jointType\t= %s\n"%self.jointType
        s+= "id\t\t= "+str(self.id)
        s+= "\ntranslation\t= "+str(self.translation)
        s+= "\nrotation\t= "+str(self.rotation)

        s+= "\nPARENT\t\t= "
        if self.parent:
            parent=self.parent
            s+= "%s "%(parent.type) + str(parent.name) +" id="+str(parent.id)+";  "

        s+= "\nCHILDREN (%d)\t= "%len(self.children)
        for child in self.children:
            s+= "%s "%(child.type) + str(child.name) +" id="+str(child.id)+";  "

        s+= "\nT=\n"+str(self.globalTransformation)
        s+= "\nlocalT=\n"+str(self.localTransformation)

        return s

    def scale(self, scale):
        for i in range(3):
            self.translation[i]*=scale[i]
            sc = scale[i]
            self.localTransformation[i,3] *= sc

        for child in self.children:
            child.scale(scale)


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
        self.localR = axis_angle2rot(self.rotation)
        self.localTransformation[0:3,0:3]=self.localR

        # last column
        self.localTransformation[0:3,3]=numpy.array(self.translation)+\
            numpy.dot(numpy.eye(3)-self.localR,numpy.array(self.center))

        # last line
        self.localTransformation[3,0:4]=[0,0,0,1]


    def update_global_transformation(self):
        """ update position and orientation in the world coordinates based on
        local transformation and parent's global transformation
        """
        if self.parent == None or self.parent.globalTransformation == None:
            self.globalTransformation = self.localTransformation
            return
        else:
            self.globalTransformation=numpy.dot\
                (self.parent.globalTransformation,self.localTransformation)
            return


    def init(self):
        """ Do the following initializations in order:

         * call :func:`kinematics.GenericObject.init_local_transformation` on itself
         * call init() on all children
        """
        self.init_local_transformation()
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
                    grandchild.localR = grandchild.localTransformation[:3,:3]
                    grandchild.rotaion = rot2AxisAngle(grandchild.localR)
                    grandchild.rpy = rot2rpy(grandchild.localR)
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

class Mesh(GenericObject):
    def __init__(self):
        GenericObject.__init__(self)
        self.type="mesh"
        self.name=None
        self.localR1=numpy.eye(3)  # due to offset of coordonee
        self.app=Appearance()
        self.geo=Geometry()

    def init(self):
        GenericObject.init(self)
        self.geo.compute_normals()



    def scale(self, scale_vec):
        GenericObject.scale(self,scale_vec)
        self.geo.scale(scale_vec)

    def __str__(self):
        s=""
        s+="\nrotation="+str(self.rotation)
        s+="\ntranslation="+str(self.translation)
        s+="Apparence: %s\n"%str(self.app)
        s+="\n"
        s+="Geometry: %s\n"%str(self.geo)
        return s

    def bounding_box_local(self):
        no_points = len(self.geo.coord)/3
        xs = [self.geo.coord[3*i] for i in range(no_points)]
        ys = [self.geo.coord[3*i+1] for i in range(no_points)]
        zs = [self.geo.coord[3*i+2] for i in range(no_points)]
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


#*****************************#
#           JOINT             #
#*****************************#

class Joint(GenericObject):
    """
    Joint class

    .. todo:: merge this with generic object or offload stuff from generic object to this
    """
    def __init__(self):
        GenericObject.__init__(self)
        self.type= "joint"
        self.jointType=None
        self.isRobot=False
        self.angle=0
        self.axis=""
        self.localR1=numpy.eye(3)  # due to cordinate offset
        self.localR2=numpy.eye(3)  # due to self rotation (revolute joint)
        self.op_point = GenericObject()
        self.add_child(self.op_point)

    def __str__(self):
        s = GenericObject.__str__(self)
        s+= "\naxis\t\t= "+str(self.axis)
        s+= "\nangle\t\t= "+str(self.angle)
        return s

    def update_local_transformation(self):
        """
        update local transformation w.r.t element's parent

        .. seealso:: VRML transform calculation http://www.web3d.org/x3d/specifications/vrml/ISO-IEC-14772-VRML97/part1/nodesRef.html#Transform
        """
        if self.jointType in ["free", "freeflyer"]:
            self.localR=euleur2rotation(self.rpy)
            self.localTransformation[0:3,0:3]=self.localR
            self.localTransformation[0:3,3]=numpy.array(self.translation)+\
                numpy.dot(numpy.eye(3)-self.localR,numpy.array(self.center))
        elif ( self.type=="joint" and self.jointType in [ "rotate", "rotation", "revolute"]
               and self.id >= 0):
            self.localR2=axis_name_angle2rot(self.axis,self.angle)
            self.localR=numpy.dot(self.localR1, self.localR2)
            self.localTransformation[0:3,0:3]=self.localR

    def init_local_transformation(self):
        """
        compute local transformation w.r.t for the first time (compute everything)
        """
        self.localTransformation = numpy.eye(4)
        self.globalTransformation = numpy.eye(4)
        self.localR1=axis_angle2rot(self.rotation)
        self.localR = self.localR1
        self.localTransformation[0:3,0:3]=self.localR
        self.update_local_transformation()
        self.localTransformation[0:3,3]=numpy.array(self.translation)+\
            numpy.dot(numpy.eye(3)-self.localR,numpy.array(self.center))
        self.localTransformation[3,3]=1


class Robot(Joint):
    """
    Typically represent a robot/model. First element in the kinematic chain
    """
    def __init__(self):
        Joint.__init__(self)
        self.ndof = 0
        self.type= "Robot"
        self.id= BASE_NODE_ID
        self.joint_dict= {}
        self.joint_names = []
        self.segment_names = []
        self.waist=None
        self.moving_joint_list = []

    def __str__(self):
        s = Joint.__str__(self)
        s += "\nNumber of dof: %d"%self.ndof
        s += "\nNumber meshes: %d"%len(self.mesh_list)
        return s

    def update_config(self, config):
        self.set_angles(config[6:])
        self.waist.translation = config[0:3]
        self.waist.rpy = config[3:6]
        self.init_local_transformation()
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
            self.joint_dict[joint.id] = joint

    def update_moving_joint_list(self):
        self.moving_joint_list = [ j for j in self.joint_list if isinstance(j.id,int)
                                   and j.id != BASE_NODE_ID ]
        self.moving_joint_list.sort(key = lambda x: x.id)
        self.ndof = len(self.moving_joint_list)

    def init(self):
        """ Do the following initializations in order:

         * build joint_list and mesh_list
         * call :func:`kinematics.Joint.init_local_transformation` on itself
         * call init() on all children
        """
        GenericObject.init(self)
        self.update()

        for joint in self.joint_list:
            if joint.jointType in ["free","freeflyer"]:
                self.waist=joint

        self.update_joint_dict()

        self.init_local_transformation()
        self.update_global_transformation()
        self.update_moving_joint_list()

        for mesh in self.mesh_list:
            mesh.app.transparency = 0
            for color in (mesh.app.diffuseColor,mesh.app.ambientColor,
                          mesh.app.specularColor,mesh.app.emissiveColor):
                if color == None:
                    continue
                if type(color) != list:
                    raise Exception("Invalid color for mesh %s"%str(mesh))

                if len(color) != 3:
                    raise Exception("Invalid len for a color in mesh %s"
                                    %str(mesh.name))

        if not self.mesh_list[:]:
            logger.warning("Robot contains 0 mesh.")



class Appearance():
    def __init__(self):
        self.diffuseColor   = None
        self.ambientColor   = None
        self.specularColor  = None
        self.emissiveColor  = None
        self.shininess      = None
        self.transparency   = None
        self.ambientIntensity= 0.0

    def __str__(self):
        s=""
        s+="\ndiffuseColor="+str(self.diffuseColor)
        s+="\nspecularColor="+str(self.specularColor)
        s+="\nemissiveColor="+str(self.emissiveColor)
        s+="\nshininess="+str(self.shininess)
        s+="\ntransparency="+str(self.transparency)
        s+="\nambientIntensity="+str(self.ambientIntensity)
        return s

class Geometry():
    def __init__(self):
        self.coord=[]
        self.idx=[]
        self.norm=[]
        self.tri_idxs  = []
        self.quad_idxs = []
        self.poly_idxs = []
        self.tri_count  = 0
        self.quad_count = 0
        self.normals = self.norm
    def __str__(self):
        s="Geometry:"
        s+="%d points and %d faces"%(len(self.coord)/3,len(self.idx)/4)
        return s
    def scale(self,scale_vec):
        if len(scale_vec) !=3 :
            raise Exception("Expected scale_vec of dim 3, got %s"%str(scale_vec))

        scale_x = scale_vec[0]
        scale_y = scale_vec[1]
        scale_z = scale_vec[2]

        for i in range(len(self.coord)/3):
            self.coord[3*i]   *= scale_x
            self.coord[3*i+1] *= scale_y
            self.coord[3*i+2] *= scale_z


    def compute_normals(self):
        npoints=len(self.coord)/3
        if self.norm==[]:
            normals=[]
            points=[]
            for k in range(npoints):
                normals.append(numpy.array([0.0,0.0,0.0]))
                points.append(numpy.array([self.coord[3*k],self.coord[3*k+1],
                                           self.coord[3*k+2]]))

        poly=[]
        ii=0
        for a_idx in self.idx:
            if a_idx!=-1:
                poly.append(a_idx)
                continue
            num_sides = len(poly)
            # if num_sides not in (3,4):
            #     logger.warning("""n=%d.  Only support tri and quad mesh for
            #                       the moment"""%num_sides)
            #     poly=[]
            #     continue
            # a_idx=-1 and poly is a triangle
            if num_sides == 3:
                self.tri_idxs += poly
            elif num_sides == 4:
                self.quad_idxs += poly
            else:
                self.poly_idxs.append(poly)

            if self.norm == []:
                # update the norm vector
                # update the normals using G. Thurmer, C. A. Wuthrich,
                # "Computing vertex normals from polygonal facets"
                # Journal of Graphics Tools, 3 1998
                ids  = (num_sides)*[None]
                vecs = []
                for i in range(num_sides):
                    vecs.append((num_sides)*[None])
                alphas = (num_sides)*[0]
                for i, iid in enumerate(poly):
                    ids[i] = iid

                for i in range(num_sides):
                    j = i + 1
                    if j == num_sides:
                        j = 0
                    vecs[j][i] = normalized(points[ids[j]] - points[ids[i]])

                try:
                    for i in range(num_sides):
                        if i == 0:
                            alphas[i] = acos(numpy.dot
                                             (vecs[1][0],vecs[0][num_sides-1]))
                        elif i == num_sides - 1:
                            alphas[i] = acos(numpy.dot (
                                vecs[0][num_sides-1],
                                vecs[num_sides-1][num_sides-2]))
                        else:
                            alphas[i] = acos(numpy.dot(vecs[i][i-1],
                                                       vecs[i+1][i]))
                except Exception,error:
                    s = traceback.format_exc()
                    logger.warning("Mesh processing error: %s"%s)

                for i,alpha in enumerate(alphas):
                    if i == 0:
                        normals[ids[i]] += alpha*normalized(
                            numpy.cross(vecs[0][num_sides-1],vecs[1][0]))
                    elif i == num_sides - 1:
                        normal_i = numpy.cross(vecs[num_sides-1][num_sides-2],
                                               vecs[0][num_sides-1])
                        normals[ids[i]] += alpha*normalized(normal_i)
                    else:
                        normal_i = numpy.cross(vecs[i][i-1], vecs[i+1][i])
                        normals[ids[i]] += alpha*normalized(normal_i)
            poly=[]
        if self.norm!=[]:
            self.normals = self.norm
        else:
            for normal in normals:
                normal                =  normalized(normal)
                self.normals          += [normal[0],normal[1],normal[2]]
                # self.norm  += self.norms
