#! /usr/bin/env python

from robo import GenericObject
import numpy as np
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

