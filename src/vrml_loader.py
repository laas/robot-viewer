#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2010
# Authors Duong Dang

import sys,re,pprint
import robo
from collections import deque
from meshLoader import *
from vrml_grammar import VRMLPARSERDEF,buildVRMLParser

class Leaf():
    '''
    Presentation of an element in a VRML tree
    '''
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
        elif self.distro in ["name","def","nodegi","SFString","CHARNODBLQUOTE"]:
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

    def nextSibling(self):
        found=False
        if self.parent:
            for child in self.parent.children:
                if found:
                    return child
                if child == self:
                    found = True
        return None

    def nextSiblings(self):
        alist=[]
        tmp = self.nextSibling()
        while tmp:
            alist.append(tmp)
            tmp=tmp.nextSibling()
        return alist

    def getBfList(self):
        """Get breath first list of all decendants

        Arguments:
        - `self`:
        """
        alist=[]
        pile = deque()
        pile.append(self)
        while not len(pile) == 0:
            an_element = pile.popleft()
            alist.append(an_element)
            for child in reversed(an_element.children):
                pile.append(child)
        return alist

    def getDfList(self):
        """Get depth first list of all decendants

        Arguments:
        - `self`:
        """
        alist=[]
        pile = deque()
        pile.append(self)
        while not len(pile) == 0:
            an_element = pile.pop()
            alist.append(an_element)
            for child in reversed(an_element.children):
                pile.append(child)
        return alist



    def getLeavesWithDistro(self,a_distro):
        """
        get in a (sub)tree all leaves with given distro
        Arguments:
        - `a_distro`:
        """
        a_list=[]
        for a_leaf in self.getBfList():
            if a_leaf.distro==a_distro:
                a_list.append(a_leaf)
        return a_list


def parseVRML(struct,data):
    ''' Parse the resulting structure from simpleparse (using grammar in
    src/vrml_grammar.py) to the Tree

    :param struct: hash from simple parse
    :param data: full structure
    :type data: string
    :returns: resulting VRML tree
    :rtype: :class:`load.Leaf`
    '''
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


def getObjectList(rootItemNode):
    '''
    Parse a VRML tree to a list of object

    :param rootItemNode: root item of a VRMLtree
    :type rootItemNode: :class:`load.Leaf`
    :rtype: list
    :returns: list of :class:`robo.GenericObject`  objects
    '''
    objectList=list()
    object=None
    if rootItemNode.distro=="rootItem":
        for leaf in rootItemNode.children:
            if leaf.distro != "Node":
#                print "getObjectList() not a Node, skipping,"+\
#                       "distro=%s"%leaf.distro
                continue

            childLeaves=leaf.children[:]
            # create a copy, donot use list2=list1 as they will point to the
            # same data

            if childLeaves[0].distro=="def":
                nameleaf=childLeaves.pop(0)
            if not childLeaves[0].distro=="nodegi":
                raise Exception("A node must have a type name "+\
                                    "(Joint, Segment, Camera...)")

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
    '''
    Return an object from a leaf of distro 'Node'

    :param nodeLeaf: a leaf of distro 'Node'
    :type nodeLeaf: :class:`load.Leaf`
    :rtype: :class:`robo.GenericObject`
    :returns: resulting object or None if nodeLeaf is not a Node
    '''
    leaf=nodeLeaf
    childLeaves=leaf.children[:]

    if leaf.distro != "Node":
        raise Exception("Expecting a node")
#           A note must start with either
#              DEF something something {..
#     or       something {

    objectName=None
    if childLeaves[0].distro=="def":
        nameleaf=childLeaves.pop(0)
        objectName=nameleaf.children[0].value

    if not childLeaves[0].distro=="nodegi":
        raise Exception("Expecting a nodegi but distro=%s Parsing:\n %s"\
                            %(childLeaves[0].distro,childLeaves[0]))

    typeleaf=childLeaves.pop(0)
    objectType=typeleaf.value

    if  isRoot and (objectType not in ["Humanoid", "Joint"]):
        errorMsg="Expecting a Humanoid or a Joint but objectType="+\
            "%s. \n Parsing:\n %s"%( objectType,leaf.fullString() )
        raise Exception(errorMsg)

    if objectType in ["Joint","joint"]:
        object=robo.Joint()
    elif objectType=="Humanoid":
        object=robo.BaseNode()

    elif objectType in ["Shape"]:
        object=Mesh()
        shape=Shape()
        shape.loadVRMLleaf(leaf)
        object.shapes.append(shape)
    else:
        object=robo.GenericObject()

    for a_leaf in childLeaves:
        if a_leaf.distro!="Attr":
            raise Exception ("Expecting an attribute")

        if len(a_leaf.children) !=2:
            raise Exception("Atribute must have 2 children not %d"\
                                 %(len(a_leaf.children)))

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
                raise Exception("Invalid translation: %s"\
                                    %fieldLeaf.fullString())
            object.translation=[fieldLeaf.children[0].value,\
                             fieldLeaf.children[1].value,\
                             fieldLeaf.children[2].value]

        elif fieldName=="center":
            if len(fieldLeaf.children)!=3:
                raise Exception("Invalid center: %s"\
                                    %fieldLeaf.fullString())
            object.center=[fieldLeaf.children[0].value,\
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


    object.name= objectName

    object.init()
    if object.id==-666:
        print object.children[0]
    return object


def VRML_read_string(filename,loadMesh,verbose,defSupport):
    vrmlFilePath=re.sub(r"[\w_\d]+\.wrl","",filename)

    data = open(filename).read()

    # replace in place Inline { url "" } directives
    if loadMesh:
        matches = re.finditer(r'Inline\s+\{\s+url\s+\"([^\"]+)\"\s+\}'\
                                  ,data)
        for match in matches:
            directive = match.group(0)
            url = match.group(1)
            tmp = open(vrmlFilePath+"/"+url).read()
            data=data.replace(directive,tmp)

    if defSupport:
        # process DEF, USE, IS etc. macro
        parser = buildVRMLParser()
        success, tags, next = parser.parse( data)
        if success!=1:
            raise Exception("Invalid vrml file")

        replace_list=[]

        for tag in tags:
            VRMLtree=parseVRML(tag,data)
            defLeaves=VRMLtree.getLeavesWithDistro("def")
            for a_leaf in defLeaves:
                def_name=a_leaf.children[0].value
                def_type=a_leaf.nextSibling().fullString()
                parent_node=a_leaf.parent
                def_part=re.compile(r"\s*DEF\s+%s\s*"%def_name)
                # strip this part from parent_string
                body_string=def_part.sub("",parent_node.fullString())
                replace_list.append((def_name,body_string))

        for a_pair in replace_list:
            def_name=a_pair[0]
            substi_string=a_pair[1]

            # cleanup this!!  replace DEF somthing to DEF__something to
            # differentiate with other instance of somthing, than replace something
            # by the substitute string, replace DEF__something back to original
            data=re.sub(r"DEF\s*%s"%def_name,"DEF___%s"%def_name,data)
            data=data.replace(" %s "%def_name," %s "%substi_string)
            data=data.replace("DEF___%s"%def_name,"DEF %s"%def_name)

    return data


def load(filename,loadMesh=True,verbose=False):
    '''
    Get the robot from a VRML file

    :param filename: input file
    :param loadMesh: flag indicating if the meshes will be loaded
    :type loadMesh: bool
    :param verbose: verbose flag
    :type verbose: bool
    :return: the robot if any, raise Error otherwise
    :rtype: :class:`robo.BaseNode`
    '''
    data = VRML_read_string(filename,loadMesh,verbose,False)

    parser = buildVRMLParser()
    success, tags, next = parser.parse( data)
    if success!=1:
        raise Exception("Invalid vrml file")

    robots=[]

    for tag in tags:
        VRMLtree=parseVRML(tag,data)
        robots+=getObjectList(VRMLtree)

    if len(robots)!=1:
        errorMsg="Only one robot allowed in the scene."+\
            " Your scene has %d robot(s)" %len(robots)
        for robot in robots:
            errorMsg+="\n"+robot.__str__()
        raise Exception(errorMsg)
    return robots[0]


def main():
    robot=load(sys.argv[1],True,False)

    print robot
    print "it has %d meshes"%len(robot.mesh_list)
    for mesh in robot.mesh_list:
        print mesh

if __name__=="__main__":main()
