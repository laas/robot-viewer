#! /usr/bin/env python

from VRMLloader import *
from robo import genericObject
import numpy as np
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

class appearance():
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

class geometry():
    def __init__(self):
        self.coord=[]
        self.idx=[]
        self.norm=[]
    def __str__(self):
        s="Geometry:"
        s+="%d points and %d faces"%(len(self.coord)/3,len(self.idx)/4)
        return s
                                     

class Shape():
    def __init__(self):
        self.app=appearance()
        self.geo=geometry()
    def __str__(self):
        s=""
        s=self.app.__str__()
        s+="\n"
        s+=self.geo.__str__()
        return s
    
    def loadVRMLleaf(self,aleaf):
        if aleaf.distro != "Node":
            raise Exception("Expecting a node")    
        childLeaves=aleaf.children[:]

        if not childLeaves[0].distro=="nodegi":
            raise Exception("Expecting a nodegi but distro=%s Parsing:\n %s"%(childLeaves[0].distro,childLeaves[0]))

        typeleaf=childLeaves.pop(0)
        objectType=typeleaf.value
        if  (objectType not in ["Shape"]):
            raise Exception("Expecting a Shape but objectType=%s. \n Parsing:\n %s"%( objectType,aleaf.fullString() ))

        for aaa_leaf in childLeaves:
            if aaa_leaf.distro!="Attr":
                raise Exception ("Expecting an attribute")

            if len(aaa_leaf.children) !=2:
                raise Exception ("Atribute must have 2 children. But lhere we have %d"%(len(aaa_leaf.children)))

            fffieldName=aaa_leaf.children[0].value
            fffieldLeaf=aaa_leaf.children[1]

            if fffieldName=="appearance":
                matLeaves=[]
                matLeaves=fffieldLeaf.children[0].children[1]\
                    .children[1].children[0].children[:]
                matLeaves.pop(0) # should check the name but im lazy

                for matLeaf in matLeaves:
                    matfieldName=matLeaf.children[0].value
                    matfieldLeaf=matLeaf.children[1]

                    if matfieldName=="diffuseColor":
                        self.app.diffuseColor=[matfieldLeaf.children[0].value, matfieldLeaf.children[1].value, matfieldLeaf.children[2].value]
                                                
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

            elif fffieldName=="geometry":
                geoLeaves=fffieldLeaf.children[0].children
                for geoLeaf in geoLeaves:
                    if geoLeaf.distro=="Attr":
                        if geoLeaf.children[0].value=="coord":
                            pointLeaf=geoLeaf.children[1]\
                                .children[0].children[1].\
                                children[1]
                            for numLeaf in pointLeaf.children:
                                self.geo.coord.append(numLeaf.value)
                        elif geoLeaf.children[0].value=="coordIndex":
                            indexLeaves=geoLeaf.children[1].children
                            for indexLeaf in indexLeaves:
                                self.geo.idx.append(int(indexLeaf.value))
                                                                  
class mesh(genericObject):
    def __init__(self,translation=[0,0,0],rotation=[1,0,0,0]):
        self.type="mesh"
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
        self.shapes=[]
        self.id=None

    def __str__(self):
        s=""
        for shape in self.shapes:
            s+=shape.__str__()
            s+="\n"
        return s

def meshLoader(filename):
    from vrml_grammar import VRMLPARSERDEF,buildVRMLParser
    data=open(filename,'r').read()
    parser = buildVRMLParser()
    success, tags, next = parser.parse( data)
    if len(tags)>1:
        raise Exception("%s failed. 1 tag is enough"%filename)
    tag=tags[0]
    
    tree=parseVRML(tag,data)
    if len(tree.children)>1:
        raise Exception("%s failed. 1 child is enough"%filename)

    resultingMesh=getMesh(tree.children[0])
    resultingMesh.name=filename
    return resultingMesh
    

def getMesh(nodeLeaf):
    leaf=nodeLeaf
    childLeaves=leaf.children[:]
    if leaf.distro != "Node":
        raise Exception("Expecting a node")    
#           A note must start with either
#              DEF something something {..
#     or       something {

    if not childLeaves[0].distro=="nodegi":
        raise Exception("Expecting a nodegi but distro=%s Parsing:\n %s"%(childLeaves[0].distro,childLeaves[0]))

    typeleaf=childLeaves.pop(0)
    objectType=typeleaf.value

    if  (objectType not in ["Transform"]):
        raise Exception("Expecting a Tranform but objectType=%s. \n Parsing:\n %s"%( objectType,leaf.fullString() ))

    new_mesh=mesh()
    for a_leaf in childLeaves:
        if a_leaf.distro!="Attr":
            raise Exception ("Expecting an attribute")

        if len(a_leaf.children) !=2:
            raise Exception ("Atribute must have 2 children. But lhere we have %d"%(len(a_leaf.children)))

        fieldName=a_leaf.children[0].value
        fieldLeaf=a_leaf.children[1]
        
        if fieldName in ["children"]:
            for childNode in fieldLeaf.children:
                a_shape=Shape()
                a_shape.loadVRMLleaf(childNode)
                new_mesh.shapes.append(a_shape)

        elif fieldName=="translation":
            if len(fieldLeaf.children)!=3:
                raise Exception("Invalid translation")
            new_mesh.translation=[fieldLeaf.children[0].value,\
                             fieldLeaf.children[1].value,\
                             fieldLeaf.children[2].value]

        elif fieldName=="rotation":
            if len(fieldLeaf.children)!=4:
                raise Exception("Invalid rotation")
            new_mesh.rotation=[fieldLeaf.children[0].value,\
                             fieldLeaf.children[1].value,\
                             fieldLeaf.children[2].value,\
                             fieldLeaf.children[3].value]
                    
    return new_mesh

def main():
    import sys
    print meshLoader(sys.argv[1])

if __name__=="__main__":main()
