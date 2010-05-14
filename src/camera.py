from __future__ import division
from math import sin, cos


import robo
from math import sqrt,sin,cos,atan2
import numpy as np

def norm(a):
    return sqrt(np.dot(a,a))

def normalized(v):
    return v/norm(v)


class Camera(object):
    def __init__(self, position=[2.0,0,1.0],lookat=[0,0,1.0], up=[0,0,1]):
        self.position=np.array(position)
        self.lookat=np.array(lookat)
        self.up=np.array(up)
        self.baseSpeed=1 # m/s
        self.goalPosition=np.array(position)
        self.goalLookat=np.array(lookat)
        self.distance=2.0

    
    def adjust(self,robot,dt):
        if not robot.waist:
            return
        robotpos=robot.waist.translation
        self.lookat[2]=self.position[2]

        self.goalLookat[2]=self.position[2]

        vec=(self.position-robotpos)[0:2]
        realDistance=norm(vec)

        pos2goal=(self.goalPosition-self.position)
        ddd=norm(pos2goal)
        if ddd > 0.2:
            pos2goalDir=pos2goal/ddd
            self.position+=pos2goalDir*self.baseSpeed*dt
            # print "Camera: adjusting position to",self.position


        lookat2goal=(self.goalLookat-self.lookat)
        ddd=norm(lookat2goal)
        if ddd > 0.2:
            lookat2goalDir=lookat2goal/ddd
            self.lookat+=lookat2goalDir*self.baseSpeed*0.1*dt
            # print "Camera: ddd=%f  adjusting lookat position to"\
            #    %ddd,self.lookat,self.goalLookat


        vec=(self.position-robotpos)
        realDistance=norm(vec)
        if realDistance > 1.5*self.distance \
                or realDistance < 0.66 * self.distance:
            goalVec=vec*self.distance/realDistance
            self.goalPosition=robotpos+goalVec
            # print "Camera: adjusting goalPosition=",\
            #    self.goalPosition

        lookatDiff=norm((self.goalLookat-robotpos)[0:2])
        if lookatDiff > 0.2:
            self.goalLookat[0]=robotpos[0]
            self.goalLookat[1]=robotpos[1]
                        
    def moveZ(self,dz):
        self.position[2]+=dz
        self.lookat[2]+=dz
        self.goalPosition=self.position

                        
    def moveR(self,dr):
        pos2look=(self.position-self.lookat)
        ddd=norm(pos2look)
        if ddd > 0:
            pos2lookDir=pos2look/ddd
            self.position+=pos2lookDir*dr
            self.goalPosition=self.position
            self.distance=norm(self.position-self.lookat)
            # print "Camera: adjusting position to",self.position

    
    def rotateZ(self,d_al):
        x=self.position[0]-self.lookat[0]
        y=self.position[1]-self.lookat[1]
        r=sqrt(x*x+y*y)
        alpha=atan2(x,y)
        newalpha=alpha+d_al
        self.position[0]=self.lookat[0]+r*sin(newalpha)
        self.position[1]=self.lookat[1]+r*cos(newalpha)

        self.goalPosition=self.position
        


        
