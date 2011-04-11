#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2010
# Authors Duong Dang
import numpy as np
import re
from math import sin,cos
from mathaux import *
from collections import deque
import logging

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("robotviewer.kinematic_chain")
logger.addHandler(NullHandler())

BASE_NODE_ID = -1

class GenericObject(object):
    """
    Base element in the kinematic tree
    """
    def __init__(self):
        self.type="GenericObject"
        self.name=None
        self.jointType=""
        self.translation=[0,0,0]
        self.rotation=[1,0,0,0]
        self.center=[0,0,0]
        self.parent=None
        self.children=[]
        self.localTransformation = None
        self.globalTransformation = None
        self.localR=np.eye(3)
        self.id=None
        self.mesh_list=[]
        self.joint_list= []

    def get_op_point(self, id):
        if not isinstance(self, Robot):
            return self

        if id == None:
            return self.waist
        else:
            return self.moving_joint_list[id]

    def update_kinematics(self, config):
        self.translation = config[0:3]
        self.rotation = config[3:6]
        self.initLocalTransformation()
        self.update()


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
            self.localTransformation[i][3]*=scale[i]

        for child in self.children:
            child.scale(scale)


    def addChild(self,a_child):
        """
        Add a :class:`kinematic_chain.GenericObject` object to element's children list
        """
        self.children.append(a_child)
        a_child.parent = self

    def setParent(self,parent):
        """
        Set a :class:`kinematic_chain.GenericObject` object as element's parent
        """
        self.parent=parent


    def updateLocalTransformation(self):
        """ update local transformation w.r.t element's parent. Nothing to do,
        since nothing varies for a GenericObject

        .. seealso:: VRML transform calculation http://www.web3d.org/x3d/specifications/vrml/ISO-IEC-14772-VRML97/part1/nodesRef.html#Transform

        """
        pass

    def initLocalTransformation(self):
        """
        compute local transformation w.r.t for the first time (compute everything)
        """
        # rotation part
        self.localTransformation = np.eye(4)
        self.localR = axisAngle2rot(self.rotation)
        self.localTransformation[0:3,0:3]=self.localR

        # last column
        self.localTransformation[0:3,3]=np.array(self.translation)+\
            np.dot(np.eye(3)-self.localR,np.array(self.center))

        # last line
        self.localTransformation[3,0:4]=[0,0,0,1]


    def updateGlobalTransformation(self):
        """ update position and orientation in the world coordinates based on
        local transformation and parent's global transformation
        """
        if self.parent == None or self.parent.globalTransformation == None:
            self.globalTransformation = self.localTransformation
            return
        else:
            self.globalTransformation=np.dot\
                (self.parent.globalTransformation,self.localTransformation)
            return


    def init(self):
        """ Do the following initializations in order:

         * call :func:`kinematic_chain.GenericObject.initLocalTransformation` on itself
         * call init() on all children
        """
        self.initLocalTransformation()
        self.updateGlobalTransformation()
        def _union(l1, l2):
            return list(set(l1).union(set(l2)))

        if isinstance(self, Mesh):
            self.mesh_list = [self]
        if isinstance(self, Joint):
            self.joint_list = [self]

        for child in self.children:
            try:
                child.init()
                self.mesh_list  = _union(child.mesh_list,self.mesh_list)
                self.joint_list = _union(child.joint_list,self.joint_list)

            except Exception, error:
                print error, "on object %s"%child.name

    def update(self):
        """ Do the following initializations in order:

         * call :func:`kinematic_chain.GenericObject.updateLocalTransformation` on itself
         * call :func:`kinematic_chain.GenericObject.updateGlobalTransformation` on itself
         * call update() on all children
        """
        self.updateLocalTransformation()
        self.updateGlobalTransformation()
        for child in self.children:
            child.update()

    def getRobot(self):
        """
        Get the highest level parent

        :returns: the Robot
        :rtype: :class:`kinematic_chain.Robot`.
        """
        if self.parent==None:
            return self
        else:
            return self.parent.getRobot()

    def getParentJoint(self):
        """
        Get the highest level parent

        :returns: the Robot
        :rtype: :class:`kinematic_chain.Robot`.
        """
        if not self.parent:
            return None
        elif isinstance(self.parent,Joint):
            return self.parent
        else:
            return self.parent.getParentJoint()



class Mesh(GenericObject):
    def __init__(self):
        GenericObject.__init__(self)
        self.type="mesh"
        self.name=None
        self.localR1=np.eye(3)  # due to offset of coordonee
        self.app=Appearance()
        self.geo=Geometry()

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
        bbox[0] = np.dot(self.globalTransformation,
                         np.array(bbox[0] + [0]))[:3]
        bbox[1] = np.dot(self.globalTransformation,
                         np.array(bbox[1] + [0]))[:3]
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
        self.rpy=[0,0,0]
        self.angle=0
        self.axis=""
        self.localR1=np.eye(3)  # due to cordinate offset
        self.localR2=np.eye(3)  # due to self rotation (revolute joint)
        self.op_point = GenericObject()
        self.addChild(self.op_point)

    def __str__(self):
        s = GenericObject.__str__(self)
        s+= "\naxis\t\t= "+str(self.axis)
        s+= "\nangle\t\t= "+str(self.angle)
        return s

    def updateLocalTransformation(self):
        """
        update local transformation w.r.t element's parent

        .. seealso:: VRML transform calculation http://www.web3d.org/x3d/specifications/vrml/ISO-IEC-14772-VRML97/part1/nodesRef.html#Transform
        """
        if self.jointType in ["free", "freeflyer"]:
            self.localR=euleur2rotation(self.rpy)
            self.localTransformation[0:3,0:3]=self.localR
            self.localTransformation[0:3,3]=np.array(self.translation)+\
                np.dot(np.eye(3)-self.localR,np.array(self.center))
        elif ( self.type=="joint" and self.jointType in [ "rotate", "rotation", "revolute"]
               and self.id >= 0):
            self.localR2=axisNameAngle2rot(self.axis,self.angle)
            self.localR=np.dot(self.localR1, self.localR2)
            self.localTransformation[0:3,0:3]=self.localR

    def initLocalTransformation(self):
        """
        compute local transformation w.r.t for the first time (compute everything)
        """
        self.localTransformation = np.eye(4)
        self.globalTransformation = np.eye(4)
        self.localR1=axisAngle2rot(self.rotation)
        self.localR = self.localR1
        self.localTransformation[0:3,0:3]=self.localR
        self.updateLocalTransformation()
        self.localTransformation[0:3,3]=np.array(self.translation)+\
            np.dot(np.eye(3)-self.localR,np.array(self.center))
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


    def update_kinematics(self, config):
        self.waist.translation = config[0:3]
        self.waist.rotation = config[3:6]
        self.setAngles(config[6:])
        self.initLocalTransformation()
        self.update()


    def setAngles(self,angles):
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

    def printJoints(self):
        """
        Print all joints
        """
        for joint in self.joint_list:
            print "\n====\n",joint

    def getConfig(self):
        vec = self.waist.translation
        vec += self.waist.rpy
        for i in range(len(self.joint_list)):
            if self.joint_dict.has_key(i):
                vec += [self.joint_dict[i].angle]
        return vec

    def waistPos(self,p):
        """
        Set waist position

        :param p: input position
        :type p: 3-double turple
        """
        self.waist.translation=p

    def waistRpy(self,ori):
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
         * call :func:`kinematic_chain.Joint.initLocalTransformation` on itself
         * call init() on all children
        """
        GenericObject.init(self)
        self.update()

        for joint in self.joint_list:
            if joint.jointType in ["free","freeflyer"]:
                self.waist=joint

        self.update_joint_dict()

        self.initLocalTransformation()
        self.updateGlobalTransformation()
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
                    raise Exception("Invalid len for a color in mesh %s"%str(mesh.name))

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
