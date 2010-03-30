#! /usr/bin/env python

from VRMLloader import *
from robo import GenericObject
import numpy as np
import re,warnings
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
        return s
                                     

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
                                                                  
class Mesh(GenericObject):
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
        s+="\nrotation="+str(self.rotation)
        s+="\ntranslation="+str(self.translation)
        for shape in self.shapes:
            s+=shape.__str__()
            s+="\n"
        return s

def VRMLmeshLoader(filename):
    from vrml_grammar import VRMLPARSERDEF,buildVRMLParser
    data=open(filename,'r').read()
    parser = buildVRMLParser()
    success, tags, next = parser.parse( data)
    if len(tags)>1:
        warnings.warn("%s failed. Expected 1 tag, has %d tags"%(filename,len(tags)))

    for tag in tags:
        tree=parseVRML(tag,data)
        if len(tree.children)>1:
            raise Exception("%s failed. 1 child is enough"%filename)

        resultingMesh=None
        try:
            resultingMesh=getVRMLMesh(tree.children[0])
        except Exception,error:
            print "caught error %s"%error

            
        if resultingMesh:
            resultingMesh.name=filename
            resultingMesh
    

def getVRMLMesh(nodeLeaf):
    leaf=nodeLeaf
    childLeaves=leaf.children[:]
    if leaf.distro != "Node":
        raise Exception("Expecting a node")    
#           A note must start with either
#              DEF something something {..
#     or       something {

#    if childLeaves[0].distro=="name":
#        nameleaf=childLeaves.pop(0)
#        objectName=nameleaf.value        
#        print "caught DEF :%s"%objectName

    if not childLeaves[0].distro == "nodegi":
        raise Exception("Expecting a nodegi but distro=%s Parsing:\n %s"%(childLeaves[0].distro,childLeaves[0]))

    typeleaf=childLeaves.pop(0)
    objectType=typeleaf.value

    if  (objectType not in ["Transform"]):
        raise Exception("Expecting a Tranform but objectType=%s. \n Parsing:\n %s"%( objectType,leaf.fullString() ))

    new_mesh=Mesh()
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

    if new_mesh.shapes!=[]:
        return new_mesh
    else:
        raise Exception("Invalid mesh")
                    
 
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
                raise Exception("invalid vertex: line=%s,words[0]=%s"%(line,words[0]))
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
