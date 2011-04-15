from __future__ import division
import kinematic_chain
from math import sqrt,sin,cos,atan2
import numpy

def norm(a):
    return sqrt(numpy.dot(a,a))

def normalized(v):
    return v/norm(v)


class Camera(object):
    def __init__(self, p=[3.5,0,1.0],l=[0,0,0.7], u=[0,0,1]):
        self.position=numpy.array(p)
        self.lookat=numpy.array(l)
        self.up=numpy.array(u)
        self._old_cam_up = None;
        self._cam_ray = None
        self._cam_distance = None
        self._cam_right = None
        self._cam_up = None

    def __str__(self):
        s = object.__str__(self)
        s += "position = {0}\nlookat = {1}\nup={2}".format(self.position, self.lookat, self.up)
        return s

    def computeUnitVectors(self):

        self._cam_ray   = self.position - self.lookat
        self._cam_distance = norm(self._cam_ray)
        self._cam_right = normalized(numpy.cross(self._cam_ray,self.up))
        self._cam_up    = normalized(numpy.cross(self._cam_right,self._cam_ray))

    def rotate(self,dx,dy):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        factor = 0.01
        dup    = dy*factor
        dright = dx*factor
        self.computeUnitVectors()
        if self._old_cam_up != None:
            dot_prod = numpy.dot(self._cam_up,self._old_cam_up)
            if dot_prod < 0:
                self.up *= -1
                self._cam_right = normalized(numpy.cross(self._cam_ray,self.up))
                self._cam_up    = normalized(numpy.cross(self._cam_right,self._cam_ray))
            self._old_cam_up = self._cam_up


        self._cam_ray += dup*self._cam_up + dright*self._cam_right
        self._cam_ray = normalized(self._cam_ray)
        self.position = self.lookat + self._cam_ray*self._cam_distance
        return



    def moveSideway(self,dx,dy):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        factor = 0.01
        dup    = dy*factor
        dright = dx*factor
        self.computeUnitVectors()

        self._cam_ray -= dup*self._cam_up + dright*self._cam_right
        self._cam_ray = normalized(self._cam_ray)
        self.lookat = self.position - self._cam_ray*self._cam_distance


    def moveBackForth(self,dy):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        self.computeUnitVectors()
        factor = 0.02*0.1
        cam_distance = norm(self.position - self.lookat)
        if cam_distance > 0.1 or dy > 0:
            self.position += dy*factor*self._cam_ray
