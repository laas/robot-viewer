#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2010
# Authors Duong Dang
import numpy as np
import re
from math import sin,cos
from mathaux import *
from collections import deque

class GenericObject():
    '''
    Base element in the kinematic tree
    '''
    def __init__(self):
        self.type="GenericObject"
        self.name=None
        self.jointType=""
        self.translation=[0,0,0]
        self.rotation=[1,0,0,0]
        self.center=[0,0,0]
        self.parent=None
        self.children=[]
        self.localTransformation=np.zeros([4,4])
        self.globalTransformation=np.zeros([4,4])
        self.localR=np.eye(3)
        self.id=None

    def __str__(self): 
        s= "%s \t= %s\n"%(self.type,self.name)
        s+="jointType\t= %s\n"%self.jointType
        s+= "id\t\t= "+str(self.id)
        s+= "\ntranslation\t= "+str(self.translation)
        s+= "\nrotation\t= "+str(self.rotation)        
        s+= "\nrpy\t\t= "+str(self.rpy)
        if self.type== "joint":
            s+= "\naxis\t\t= "+str(self.axis)
            s+= "\nangle\t\t= "+str(self.angle)

        s+= "\nPARENT\t\t= "
        if self.parent:
            parent=self.parent
            s+= "%s "%(parent.type) + str(parent.name) +" id="+str(parent.id)+";  "

        s+= "\nCHILDREN (%d)\t= "%len(self.children)
        for child in self.children:
            s+= "%s "%(child.type) + str(child.name) +" id="+str(child.id)+";  "

        s+= "\nT=\n"+str(self.globalTransformation)
        s+= "\nlocalT=\n"+str(self.localTransformation)        
        s+= "\nlocalR1=\n"+str(self.localR1)        
        s+= "\nlocalR2=\n"+str(self.localR2)        

        # count the size of the tree
        count = 0
        pile = deque()
        pile.append(self)            
        while not len(pile) == 0:
            an_element = pile.pop()
            count +=  1
            for child in reversed(an_element.children):
                pile.append(child)                                
        s += "\nThe hierarchy tree has %d elements"%count
        return s


    def addChild(self,a_child):
        '''
        Add a :class:`robo.GenericObject` object to element's children list
        '''
        (self.children).append(a_child)
    
    def setParent(self,parent):
        '''
        Set a :class:`robo.GenericObject` object as element's parent
        '''
        self.parent=parent

 
    def updateLocalTransformation(self):
        ''' update local transformation w.r.t element's parent. Nothing to do,
        since nothing varies for a GenericObject
        
        .. seealso:: VRML transform calculation http://www.web3d.org/x3d/specifications/vrml/ISO-IEC-14772-VRML97/part1/nodesRef.html#Transform
        
        '''
        pass

    def initLocalTransformation(self):
        '''
        compute local transformation w.r.t for the first time (compute everything)
        '''
        # rotation part
        self.localR = rot1(self.rotation)
        self.localTransformation[0:3,0:3]=self.localR

        # last column
        self.localTransformation[0:3,3]=np.array(self.translation)+\
            np.dot(np.eye(3)-self.localR,np.array(self.center))

        # last line
        self.localTransformation[3,0:4]=[0,0,0,1]


    def updateGlobalTransformation(self):        
        ''' update position and orientation in the world coordinates based on
        local transformation and parent's global transformation
        '''
        if self.parent==None:           
            self.globalTransformation=self.localTransformation
        else:
            self.globalTransformation=np.dot\
                (self.parent.globalTransformation,self.localTransformation)

    def init(self):
        ''' Do the following initializations in order:

         * call :func:`robo.GenericObject.initLocalTransformation` on itself
         * call init() on all children
        '''
        self.initLocalTransformation()
        self.updateGlobalTransformation()    
        for child in self.children:
            try:
                child.init()
            except Exception, error:
                print error, "on object %s"%child.name

    def update(self):
        ''' Do the following initializations in order:

         * call :func:`robo.GenericObject.updateLocalTransformation` on itself
         * call :func:`robo.GenericObject.updateGlobalTransformation` on itself
         * call update() on all children
        '''        
        self.updateLocalTransformation()
        self.updateGlobalTransformation()    
        for child in self.children:
            child.update()
    def getBaseNode(self):
        '''
        Get the highest level parent

        :returns: the BaseNode
        :rtype: :class:`robo.BaseNode`.
        '''
        if self.parent==None:
            return self
        else:
            return self.parent.getBaseNode()


#*****************************#
#           JOINT             #
#*****************************#

class Joint(GenericObject):
    '''
    Joint class

    .. todo:: merge this with generic object or offload stuff from generic object to this
    '''
    def __init__(self):
        self.type= "joint"
        self.jointType="rotate"
        self.name=None
        self.id=None
        self.isBaseNode=False
        self.translation=[0,0,0]
        self.center=[0,0,0]
        self.rotation=[1,0,0,0]
        self.rpy=[0,0,0]
        self.parent=None
        self.children=[]
        self.angle=0
        self.axis=""
        self.localTransformation=np.zeros([4,4])
        self.globalTransformation=np.zeros([4,4])
        self.localR=np.eye(3)   # local rotation
        self.localR1=np.eye(3)  # due to cordinate offset
        self.localR2=np.eye(3)  # due to self rotation (revolute joint)

    def updateLocalTransformation(self):
        '''
        update local transformation w.r.t element's parent
        
        .. seealso:: VRML transform calculation http://www.web3d.org/x3d/specifications/vrml/ISO-IEC-14772-VRML97/part1/nodesRef.html#Transform
        '''
        if self.jointType=="free":                
            self.localR=euleur2rotation(self.rpy)        
            self.localTransformation[0:3,0:3]=self.localR    
            self.localTransformation[0:3,3]=np.array(self.translation)+\
                np.dot(np.eye(3)-self.localR,np.array(self.center))
        elif self.type=="joint" and self.jointType=="rotate" and self.id >= 0:
            self.localR2=rot2(self.axis,self.angle)        
            self.localR=np.dot(self.localR1, self.localR2)
            self.localTransformation[0:3,0:3]=self.localR

    def initLocalTransformation(self):
        '''
        compute local transformation w.r.t for the first time (compute everything)
        '''  
        self.localR1=rot1(self.rotation)
        self.localR = self.localR1
        self.localTransformation[0:3,0:3]=self.localR
        self.updateLocalTransformation()
        self.localTransformation[0:3,3]=np.array(self.translation)+\
            np.dot(np.eye(3)-self.localR,np.array(self.center))
        self.localTransformation[3,3]=1

#*****************************#
#         BASE NODE           #
#*****************************#

            
class BaseNode(Joint):
    '''
    Typically represent a robot/model. First element in the kinematic chain
    '''
    def __init__(self):
        self.type= "BaseNode"
        self.jointType=""
        self.name=None
        self.id=-999
        self.translation=[0,0,0]
        self.center=[0,0,0]
        self.rotation=[1,0,0,0]
        self.children=[]
        self.parent=None
        self.localTransformation=np.zeros([4,4])
        self.globalTransformation=np.zeros([4,4])
        self.joint_list=list()
        self.joint_dict=dict()
        self.rpy=[0,0,0]
        self.waist=None
        self.mesh_list=[]


    def jointAngles(self,angles):
        '''
        Set joint angles

        :param angles: input joint angles
        :type angles: list of double
        '''
        if len(angles) != 40:
            raise Exception("wrong angles size need 40 but have %d"%len(angles))
        for i in range(len(angles)):
            angle=angles[i]
            (self.joint_dict[i]).angle=angle
#            print "update joint id =%d, angle=%f"%(i,angle)

    def printJoints(self):
        '''
        Print all joints
        '''
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
        '''
        Set waist position

        :param p: input position
        :type p: 3-double turple
        '''
        self.waist.translation=p

    def waistRpy(self,ori):
        '''
        Set waist orientation

        :param ori: input raw, pitch, yaw angles
        :type ori: 3-double turple
        '''
        self.waist.rpy=ori

    def update_joint_dict(self):
        self.joint_dict = dict()
        for joint in self.joint_list:
            self.joint_dict[joint.id] = joint
    
    def init(self):
        ''' Do the following initializations in order:

         * build joint_list and mesh_list
         * call :func:`robo.Joint.initLocalTransformation` on itself
         * call init() on all children
        '''
        self.update()
        self.joint_list=[]
        self.mesh_list=[]
        pile=deque()
        pile.append(self)

        ## loop through the tree to create a list of joints 
        while not len(pile)==0:
            an_element=pile.pop()
            if an_element.type=="joint":
                self.joint_list.append(an_element)
            elif an_element.type=="mesh":
                self.mesh_list.append(an_element)
            for child in reversed(an_element.children):
                pile.append(child)
                
        for joint in self.joint_list:
            if joint.jointType=="free":
                joint.getBaseNode().waist=joint

        self.update_joint_dict()

        self.initLocalTransformation()
        self.updateGlobalTransformation()    
        for child in self.children:
            try:
                child.init()
            except Exception, error:
                print error, "on object %s"%child.name
