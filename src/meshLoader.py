#! /usr/bin/env python

from vrml_loader import *
from robo import GenericObject
import numpy as np
import re,warnings
from collections import deque
'''
Asumption:
The mesh is of the form
Transform {
translation
rotation
children [
Shape {
Material
Geometry
}
]
}
'''

class Mesh(GenericObject):
    def __init__(self):
        GenericObject.__init__(self)
        self.type="mesh"
        self.name=None
        self.localR1=np.eye(3)  # due to offset of coordonee
        self.shapes=[]

    def __str__(self):
        s=""
        s+="\nrotation="+str(self.rotation)
        s+="\ntranslation="+str(self.translation)
        for shape in self.shapes:
            s+=shape.__str__()
            s+="\n"
        return s



class Appearance():
    def __init__(self):
        self.diffuseColor   =None
        self.specularColor  =None
        self.emissiveColor  =None
        self.shininess      =None
        self.transparency   =None
        self.ambientIntensity=0.0
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
        return
# helper function
def getLeavesNextTo(tree,leafValue):
    pile = deque()
    pile.append(tree)
    while not len(pile) == 0:
        an_element = pile.pop()
        if an_element.value == leafValue :
            return an_element.nextSiblings()
        for child in reversed(an_element.children):
            pile.append(child)
    return []

def getLeafNextTo(tree,leafValue):
    pile = deque()
    pile.append(tree)
    while not len(pile) == 0:
        an_element = pile.pop()
        if an_element.value == leafValue :
            return an_element.nextSibling()
        for child in reversed(an_element.children):
            pile.append(child)
    return None


class Shape():
    def __init__(self):
        self.app=Appearance()
        self.geo=Geometry()
    def __str__(self):
        s=""
        s=self.app.__str__()
        s+="\n"
        s+=self.geo.__str__()
        return s


    ## this function needs a throughout cleanup !!!!
    def loadVRMLleaf(self,aleaf):
        if aleaf.distro != "Node":
            raise Exception("Expecting a node")
        childLeaves=aleaf.children[:]

        if not childLeaves[0].distro=="nodegi":
            raise Exception("Expecting a nodegi but distro=%s Parsing:\n %s"\
                                %(childLeaves[0].distro,childLeaves[0]))

        typeleaf=childLeaves.pop(0)
        objectType=typeleaf.value
        if  (objectType not in ["Shape"]):
            raise Exception("Expecting a Shape but objectType=%s. \n Parsing:\n %s"\
                                %( objectType,aleaf.fullString() ))

        for a_leaf in childLeaves:
            if a_leaf.distro!="Attr":
                raise Exception ("Expecting an attribute")

            if len(a_leaf.children) !=2:
                raise Exception ("Atribute must have 2 children. But lhere we have %d"\
                                     %(len(a_leaf.children)))

            fieldName=a_leaf.children[0].value
            fieldLeaf=a_leaf.children[1]

            if fieldName=="appearance":
                # search the tree for Material leaf
                matLeaves = getLeavesNextTo(fieldLeaf,"Material")

                for matLeaf in matLeaves:
                    matfieldName=matLeaf.children[0].value
                    matfieldLeaf=matLeaf.children[1]

                    if matfieldName=="diffuseColor":
                        self.app.diffuseColor=[matfieldLeaf.children[0].value,\
                                                   matfieldLeaf.children[1].value, \
                                                   matfieldLeaf.children[2].value]

                    elif matfieldName=="specularColor":
                        self.app.specularColor=\
                            [matfieldLeaf.children[0].value,\
                                 matfieldLeaf.children[1].value,\
                                 matfieldLeaf.children[2].value]


                    elif matfieldName=="emissiveColor":
                        self.app.emissiveColor=\
                            [matfieldLeaf.children[0].value,\
                                 matfieldLeaf.children[1].value,\
                                 matfieldLeaf.children[2].value]

                    elif matfieldName=="shininess":
                        self.app.shininess=\
                            matfieldLeaf.children[0].value

                    elif matfieldName=="transparency":
                        self.app.transparency=\
                            matfieldLeaf.children[0].value

                    elif matfieldName=="ambientIntensity":
                        self.app.ambientIntensity=\
                            matfieldLeaf.children[0].value

            elif fieldName=="geometry":
                pointLeaves = getLeafNextTo(fieldLeaf,"point").children
                for pointLeaf in pointLeaves:
                    self.geo.coord.append(pointLeaf.value)

                idxLeaves = getLeafNextTo(fieldLeaf,"coordIndex").children
                for idxLeaf in idxLeaves:
                    self.geo.idx.append(int(idxLeaf.value))

        if self.app.specularColor==None and self.app.diffuseColor!=None:
            self.app.specularColor==self.app.diffuseColor

        if self.app.emissiveColor==None and self.app.diffuseColor!=None:
            self.app.emissiveColor==self.app.diffuseColor


def OBJmeshLoader(filename):
    amesh=Mesh()
    ## doesn't support color, normals ... yet
    # no group so just one shape
    ashape=Shape()
    ashape.app.diffuseColor=[1,1,1]
    ashape.app.specularColor=[1,1,1]
    ashape.app.emissiveColor=[0,0,0]
    ashape.app.shininess=1.0

    lines=open(filename).readlines()
    for line in lines:
        words=line.split()
        if not words[:]:
            continue
        if re.match(r"\s*v\s*$",words[0]):
            if len(words)!=4:
                raise Exception("invalid vertex: line=%s,words[0]=%s"\
                                    %(line,words[0]))
            else:
                for word in words[1:]:
                    p=float(word)
                    ashape.geo.coord.append(p)


        elif re.match(r"\s*f\s*",words[0]):
            if len(words)<2:
                raise Exception("Invalid face")
            else:
                for word in words[1:]:
                    m=re.search(r"^(\d+)\/",word)
                    if m:
                        idx=int(m.group(1))-1
                        ashape.geo.idx.append(idx)
                ashape.geo.idx.append(-1)

    amesh.shapes.append(ashape)
    if amesh.shapes!=[]:
        return amesh
    else:
        raise Exception("Invalid mesh")

def VRMLmeshLoader(filename):
    data = open(filename).read()
    from vrml_grammar import VRMLPARSERDEF,buildVRMLParser
    parser = buildVRMLParser()
    success, tags, next = parser.parse(data)
    if success!=1:
        raise Exception("Invalid vrml file")
    print tags
    tag = (tags[0])[0]
    amesh = Mesh()
    VRMLtree=parseVRML(tag,data)
    amesh.shapes.append(Shape().loadVRMLleaf(VRMLtree))
    return amesh

def meshLoader(filename):
    if re.search(r"\.wrl$",filename):
        return VRMLmeshLoader(filename)

    elif re.search(r"\.obj$",filename):
        return OBJmeshLoader(filename)

    return None

def main():
    import sys
    print meshLoader(sys.argv[1])

if __name__=="__main__":main()
