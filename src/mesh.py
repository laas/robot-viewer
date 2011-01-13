#! /usr/bin/env python

from robo import GenericObject
import numpy as np

class Mesh(GenericObject):
    def __init__(self):
        GenericObject.__init__(self)
        self.type="mesh"
        self.name=None
        self.localR1=np.eye(3)  # due to offset of coordonee
        self.app=Appearance()
        self.geo=Geometry()

    def scale(self, scale_vec):
        self.geo.scale(scale_vec)

    def __str__(self):
        s=""
        s+="\nrotation="+str(self.rotation)
        s+="\ntranslation="+str(self.translation)
        s+="Apparence: %s\n"%str(self.app)
        s+="\n"
        s+="Geometry: %s\n"%str(self.geo)
        return s

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

