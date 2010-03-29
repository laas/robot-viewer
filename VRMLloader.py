#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang


class Leaf():
    def __init__(self):
        self.children=[]
        self.parent=None
        self.data=None
        self.struct=()
        self.begin_idx=-1
        self.end_idx=-1
        self.distro=None
        self.depth=0
        self.value=None
    def __str__(self):
        s=""
        for i in range(self.depth):
            s+=" "
        s+="Leaf %s ,gen=%d, value=%s"%(self.distro,self.depth,str(self.value))
        if len(self.children)>0:
            s+=",%d children"%len(self.children)
        s+="\n"
        for child in self.children:
            s+=child.__str__()
        return s
    
    def addChild(self,a_child):
        self.children.append(a_child)
        
    def setParent(self,a_parent):
        self.parent=a_parent

    def getData(self):
        if self.parent==None:
            return self.data
        else:
            return self.parent.getData()

    def fullString(self):
        data_string=self.getData()
        return data_string[self.begin_idx:self.end_idx]
    def updateValue(self):
        data_string=self.getData()
        if self.distro in ["Node","rootItem","Field","Attr"]: 
            self.value=list([self.begin_idx,self.end_idx])
        elif self.distro in ["name","nodegi","SFString","CHARNODBLQUOTE"]:
            self.value=data_string[self.begin_idx:self.end_idx]
        elif self.distro=="SFNumber":
            self.value=float(data_string[self.begin_idx:self.end_idx])
        else:
            self.value="Nothing"
        
        for child in self.children:
            child.updateValue()


    def parse(self,a_struct,a_data=None):
#        print "\n=====\nParsing:\n", a_struct
        self.struct=a_struct
        self.data=a_data
        self.distro=a_struct[0]
        self.begin_idx=a_struct[1]
        self.end_idx=a_struct[2]
        if a_struct[3]:
            for substruct in a_struct[3]:
                child_leaf=Leaf()
                child_leaf.depth=self.depth+1
                child_leaf.parse(substruct)
                child_leaf.setParent(self)
                self.addChild(child_leaf)
      
def parseVRML(struct,data):
    l=Leaf()
    l.parse(struct,data)
    l.updateValue()
    return l



'''
- function nodeparse(Node):
      case node_name:
          Robot: object=BaseNode()
          Joint: object=Joint()
          Generic: object=Generic()
          Inline:  object=mesh
      for attr in artributes:
          case attr_name
          rotation/translation/axis...: set object.kinematic_attr
          children:
             for childNode in childNodes:
                 child_object=nodeparse(childNode)
                 object.add_child(child_object)                 

  return object
'''
import robo
from meshLoader import *
vrmlFilePath="./"


def getObjectList(rootItemNode):
    objectList=list()
    object=None
    if rootItemNode.distro=="rootItem":
        for leaf in rootItemNode.children:            
            if leaf.distro != "Node":
#                print "getObjectList() not a Node, skipping,"+\
#                       "distro=%s"%leaf.distro
                continue
            
            childLeaves=leaf.children[:] # create a copy, donot use list2=list1 as they will point to the same data
            
            if childLeaves[0].distro=="name":
                nameleaf=childLeaves.pop(0)
            if not childLeaves[0].distro=="nodegi":
                raise Exception("A node must have a type name (Joint, Segment, Camera...)")

            typeleaf=childLeaves[0]
            objectType=typeleaf.value

            if  objectType not in ["Humanoid", "Joint"]:
#                print "getObjectList() not an interesting object, skipping"+\
#                     "objectType=%s"%objectType
                continue

            object=getObject(leaf)

            if object:
                objectList.append(object)
    else:
        raise Exception("expecting a rootNode")
    return objectList
    

def getObject(nodeLeaf,isRoot=True):
    leaf=nodeLeaf
    childLeaves=leaf.children[:]

    if leaf.distro != "Node":
        raise Exception("Expectin a node")    
#           A note must start with either
#              DEF something something {..
#     or       something {

    objectName=None
    if childLeaves[0].distro=="name":
        nameleaf=childLeaves.pop(0)
        objectName=nameleaf.value        

    if not childLeaves[0].distro=="nodegi":
        raise Exception("Expecting a nodegi but distro=%s Parsing:\n %s"%(childLeaves[0].distro,childLeaves[0]))

    typeleaf=childLeaves.pop(0)
    objectType=typeleaf.value

    if  isRoot and (objectType not in ["Humanoid", "Joint"]):
        raise Exception("Expecting a Humanoid or a Joint but objectType=%s. \n Parsing:\n %s"%( objectType,leaf.fullString() ))

    if objectType in ["Joint","joint"]:
        object=robo.Joint()
    elif objectType=="Humanoid":
        object=robo.BaseNode()
    else:
#        print "creating generic object, objecType=%s"%objectType
        object=robo.GenericObject()


    for a_leaf in childLeaves:
        if a_leaf.distro!="Attr":
            raise Exception ("Expecting an attribute")

        if len(a_leaf.children) !=2:
            raise Exception ("Atribute must have 2 children. But lhere we have %d"%(len(a_leaf.children)))

        fieldName=a_leaf.children[0].value
        fieldLeaf=a_leaf.children[1]
        
        if fieldName in ["children","humanoidBody"]:
            for childNode in fieldLeaf.children:
                childObject=getObject(childNode,False)
                childObject.setParent(object)
                object.addChild(childObject)
        elif fieldName=="jointId":
            if len(fieldLeaf.children)!=1:
                raise Exception("Invalid id")
            object.id=int(fieldLeaf.children[0].value)

        elif fieldName=="translation":
            if len(fieldLeaf.children)!=3:
                raise Exception("Invalid translation")
            object.translation=[fieldLeaf.children[0].value,\
                             fieldLeaf.children[1].value,\
                             fieldLeaf.children[2].value]

        elif fieldName=="rotation":
            if len(fieldLeaf.children)!=4:
                raise Exception("Invalid rotation")
            object.rotation=[fieldLeaf.children[0].value,\
                             fieldLeaf.children[1].value,\
                             fieldLeaf.children[2].value,\
                             fieldLeaf.children[3].value]

        elif fieldName=="jointAxis":
            if len(fieldLeaf.children)!=1:
                raise Exception("Invalid jointAxis")
            object.axis=fieldLeaf.children[0].children[0].value

        elif fieldName in ["jointType"]:
            if len(fieldLeaf.children)!=1:
                raise Exception("Invalid jointType")
            object.jointType=fieldLeaf.children[0].children[0].value

        elif fieldName in ["url"]:
            global vrmlFilePath
            global loadMeshBool
            if loadMeshBool:
                meshFile=fieldLeaf.children[0].children[0].value
                amesh=meshLoader(vrmlFilePath+meshFile)
                amesh.setParent(object)
                object.addChild(amesh)

    object.name= objectName   
    
    object.init()
    if object.id==-666:
        print object.children[0]
    return object

import re
loadMeshBool=True 

def VRMLloader(filename,loadMesh=True):
    global loadMeshBool # this is ugly
    loadMeshBool=loadMesh

    global vrmlFilePath
    from vrml_grammar import VRMLPARSERDEF,buildVRMLParser
    data = open(filename).read()

    vrmlFilePath=re.sub(r"[\w_\d]+\.wrl","",filename)
#    print "roddot=",vrmlFilePath

    parser = buildVRMLParser()
    success, tags, next = parser.parse( data)
    if success!=1:
        raise Exception("Invalid vrml file")

    robots=[]
    for tag in tags:
        VRMLtree=parseVRML(tag,data)        
        robots+=getObjectList(VRMLtree)
    
    if len(robots)!=1:
        errorMsg="Only one robot allowed in the scene. Your scene has %d robot(s)" %len(robots)
        for robot in robots:
            errorMsg+="\n"+robot.__str__()
        raise Exception(errorMsg)
    return robots[0]

import sys
def main():
    robot=VRMLloader(sys.argv[1])
    print robot
    print "it has %d meshes"%len(robot.mesh_list)
if __name__=="__main__":main()
