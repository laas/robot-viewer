#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang
import numpy as np
import re
from math import sin,cos


from pyglet.gl import *
from nutshell import *
import pyglet

class genericObject():
    def __init__(self,translation=[0,0,0],rotation=[1,0,0,0]):
        self.type="genericObject"
        self.name=None
        self.jointType=""
        self.translation=translation
        self.rotation=rotation
        self.parent=None
        self.children=[]
        self.rpy=[0,0,0]
        self.localTransformation=np.zeros([4,4])
        self.globalTransformation=np.zeros([4,4])
        self.localR=np.eye(3)   # local rotation
        self.localR1=np.eye(3)  # due to offset of coordonee
        self.localR2=np.eye(3)  # due to self rotation (revolute joint)
        self.id=None
    def draw_skeleton(self):
        pass

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
            s+= "%s "%(parent.type) + str(self.name) +" id="+str(parent.id)+";  "

        s+= "\nCHILDREN (%d)\t= "%len(self.children)
        for child in self.children:
            s+= "%s "%(child.type) + str(self.name) +" id="+str(child.id)+";  "

        s+= "\nT=\n"+str(self.globalTransformation)
        s+= "\nlocalT=\n"+str(self.localTransformation)        
        return s


    def addChild(self,a_child):
        (self.children).append(a_child)
    
    def setParent(self,parent):
        self.parent=parent

    def updateLocalTransformation(self):
        if self.type in ["joint","baseNode"]:
            if self.jointType=="free":                
                self.localR=euleur2rotation(self.rpy)        
                self.localTransformation[0:3,0:3]=self.localR    
                self.localTransformation[0:3,3]=self.translation
            elif self.type=="joint" and self.jointType=="rotate"\
                    and self.id and self.id > 0:
                self.localR2=rot2(self.axis,self.angle)        
                self.localR=np.dot(self.localR1,self.localR2)
                self.localTransformation[0:3,0:3]=self.localR
    
    def initLocalTransformation(self):

        if self.jointType=="free":                
            self.localR=euleur2rotation(self.rpy)            
        elif self.jointType=="rotate":                
            self.localR1=rot1(self.rotation)            
            self.localR2=rot2(self.axis,self.angle)        
            self.localR=np.dot(self.localR1,self.localR2)
        else:
            self.localR=rot1(self.rotation)


        self.localTransformation[0:3,0:3]=self.localR
        self.localTransformation[0:3,3]=self.translation
        self.localTransformation[3,0:4]=[0,0,0,1]


    def updateGlobalTransformation(self):        
        if self.parent==None:           
            self.globalTransformation=self.localTransformation
        else:
            self.globalTransformation=np.dot(self.parent.globalTransformation,\
                self.localTransformation)


    def init(self):
        if self.type=="baseNode":
            self.update()
            from collections import deque
            pile=deque()
            pile.append(self)
        
            while not len(pile)==0:
                an_element=pile.pop()

                if an_element.type=="joint":
                    self.joint_list.append(an_element)
                elif an_element.type=="mesh":
                    self.mesh_list.append(an_element)

                for child in reversed(an_element.children):
                    pile.append(child)
                    if child.id!=None:
                        self.joint_dict[child.id]=child

            for joint in self.joint_list:
                if joint.jointType=="free":
                    joint.getBaseNode().waist=joint

        self.initLocalTransformation()
        self.updateGlobalTransformation()    

        for child in self.children:
            try:
                child.init()
            except Exception, error:
                print error, "on object %s"%child.name


    def update(self):
        self.updateLocalTransformation()
        self.updateGlobalTransformation()    
        for child in self.children:
            child.update()


    def getBaseNode(self):
        if self.parent==None:
            return self
        else:
            return self.parent.getBaseNode()


# ******************************
#                              #
#                              # 
# ******************************
class joint(genericObject):
    def __init__(self,id=None,translation=[0,0,0],rotation=[1,0,0,0],axis= ""):
        self.type= "joint"
        self.jointType=""
        self.name=None
        self.id=id
        self.isBaseNode=False
        self.translation=translation
        self.rotation=rotation
        self.rpy=[0,0,0]
        self.parent=None
        self.children=[]
        self.angle=0
        self.axis=axis
        self.localTransformation=np.zeros([4,4])
        self.globalTransformation=np.zeros([4,4])
        self.localR=np.eye(3)   # local rotation
        self.localR1=np.eye(3)  # due to offset of coordonee
        self.localR2=np.eye(3)  # due to self rotation (revolute joint)
        
    def draw_skeleton(self):
        # draw_skeleton a sphere at each mobile joint
        if self.jointType in ["free","rotate"]:
            pos=self.globalTransformation[0:3,3]
            glPushMatrix()
            glTranslatef(pos[0], pos[1], pos[2])
            sphere = gluNewQuadric() 
            gluSphere(sphere,0.01,10,10)
            glPopMatrix()

            if self.jointType=="rotate":
                parent=self.parent
                parent_pos=parent.globalTransformation[0:3,3]
                draw_link(pos,parent_pos)
        for child in self.children:
            child.draw_skeleton()
            
class baseNode(joint):
    def __init__(self,translation=[0,0,0],rotation=[1,0,0,0],children=[]):
        self.type= "baseNode"
        self.jointType=""
        self.name=None
        self.id=-999
        self.translation=translation
        self.rotation=rotation
        self.children=children
        self.parent=None
        self.localTransformation=np.zeros([4,4])
        self.globalTransformation=np.zeros([4,4])
        self.joint_list=[]
        self.joint_dict=dict()
        self.rpy=[0,0,0]
        self.waist=None
        self.mesh_list=[]
    def jointAngles(self,angles):
        if len(angles)<40:
            raise Exception("wrong angles size need 39 but have %d"%len(angles))
        for i in range(len(angles)):
            angle=angles[i]
            (self.joint_dict[i]).angle=angle
    

    def printJoints(self):
        for joint in self.joint_list:
            print "\n====\n",joint

    def waistPos(self,p):
        self.waist.translation=p
    def waistRpy(self,p):
        self.waist.rpy=p

