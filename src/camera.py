from __future__ import division
import kinematic_chain
from math import sqrt,sin,cos,atan2
import numpy
from mathaux import *
import display_element
def norm(a):
    return sqrt(numpy.dot(a,a))

def normalized(v):
    return v/norm(v)


class Camera(kinematic_chain.GenericObject):
    '''
    World camera and robot cameras. Use OpenHRP convention
    http://openrtp.info/openhrp3/en/create_model.html#VISIONSENSOR
    '''
    def __init__(self):
        kinematic_chain.GenericObject.__init__(self)

        self.front_clip_distance = 0.01
        self.back_clip_distance = 100
        self.width = 320
        self.height = 240

        self.translation = [3.5,0,1]
        self.focal = 3.5
        self.localR = numpy.array([ [ 0 , 0 , 1],
                                    [ 1 , 0 , 0],
                                    [ 0 , 1 , 0],
                                    ]
                                  )
        self.rotation = rot2AxisAngle(self.localR)
        self.moved = True
        self.lookat = None
        self.init()
        # print self.globalTransformation

    def render(self):
        pass

    def update(self):
        kinematic_chain.GenericObject.update(self)
        lookatRel = numpy.eye(4)
        lookatRel[2,3] = - self.focal
        lookatT = numpy.dot(self.globalTransformation,
                            lookatRel)
        self.lookat = lookatT[:3,3]

    def cam_up(self):
        return self.globalTransformation[:3,1]

    def cam_position(self):
        return self.globalTransformation[:3,3]

    def cam_ray(self):
        return self.globalTransformation[:3,2]

    def cam_right(self):
        return - self.globalTransformation[:3,0]


    def __str__(self):
        s = object.__str__(self)
        s += "position = {0}\nlookat = {1}\nup={2}".format(self.cam_position(),
                                                           self.lookat, self.cam_up())
        return s

    def rotate(self,dx,dy):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        factor = 0.002
        dup    = dy*factor
        dright = dx*factor

        # print self.localTransformation
        # print trans
        new_z = self.cam_ray()
        new_z -= dup*self.cam_up() + dright*self.cam_right()
        new_z = normalized(new_z)
        new_pos = self.lookat + new_z*self.focal

        # trans = new_pos - self.cam_position()
        new_x = - normalized(numpy.cross(new_z, numpy.array([0,0,1])))
        new_y = numpy.cross(new_z, new_x)
        new_R = numpy.eye(3)

        new_R[:,0] =  new_x
        new_R[:,1] =  new_y
        new_R[:,2] =  new_z

        rpy = rot2rpy(new_R[:3,:3])
        config = self.get_config()
        config[0] = new_pos[0]
        config[1] = new_pos[1]
        config[2] = new_pos[2]
        #config[3] = rpy[0]
        #config[4] = rpy[1]
        #config[5] = rpy[2]

        self.translation  = new_pos
        self.localTransformation[:3,3] = new_pos
        self.localTransformation[:3,:3] = new_R

        #self.update()
        #print self.globalTransformation
        #print self.cam_position()
        #print self.lookat
        self.update()
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

        d = self.cam_right()*dright + self.cam_up()*dup
        for i in range(3):
            self.translation[i] += d[i]
        self.globalTransformation[:3,3] =  self.translation
        self.update()


    def moveBackForth(self,dy):
        """Move camera but keep orientation

        Arguments:
        - `self`:
        - `dx`:
        - `dy`:
        """
        if self.focal < 0.01 and dy <=0:
            return
        factor = 0.02*0.1
        d = self.cam_ray()*dy*factor
        self.focal += dy*factor
        for i in range(3):
            self.translation[i] += d[i]
        self.globalTransformation[:3,3] =  self.translation
        self.update()
