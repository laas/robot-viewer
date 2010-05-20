#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2010
# Authors Duong Dang

# auxilaries functions that don't belong to any class


import numpy as np
import re
from math import sin,cos,sqrt,acos,pi


def norm(a):
    return sqrt(np.dot(a,a))

def normalized(v):
    return v/norm(v)

def rot2AngleAxis(M):
    v=np.array( [M[2][1]-M[1][2], M[0][2]-M[2][0], M[1][0]-M[0][1] ])
    normv=sqrt(np.dot(v,v))
    if normv == 0.0:
        return (0.0,1.0,0.0,0.0)
    v=v/normv
    angle=acos( min ( (M[0][0]+M[1][1]+M[2][2]-1)/2 ,1) )*180/pi
    return [angle,v[0],v[1],v[2]]

def euleur2AngleAxis(rpy):
    R = euleur2rotation(rpy)
    return rot2AngleAxis(R)
    

def draw_link(p1,p2,size=0.01):
    p=p2-p1
    height=np.sqrt(np.dot(p,p)) 
    glPushMatrix()
    glBegin(GL_LINES)    
    glVertex3f(p1[0],p1[1],p1[2])
    glVertex3f(p2[0],p2[1],p2[2])
    glEnd()
    glPopMatrix()

def rot2(axis,a):
    if re.search(r"[zZ]",axis):
        return np.array([(cos(a), -sin(a),      0),\
                        (sin(a),  cos(a),      0),\
                        (0     ,   0    ,      1)])

    if re.search(r"[xX]",axis):
        return np.array([(1     ,     0  ,      0),\
                        (0     ,  cos(a),-sin(a)),\
                        (0     ,  sin(a), cos(a))])

    if re.search(r"[yY]",axis):
        return np.array([(cos(a) ,   0    , sin(a)),\
                        (0      ,   1    ,      0),\
                        (-sin(a),   0    , cos(a))])
    return np.eye(3)

def rot1(array):
    if array[0]==1:
        axis="X"
    elif array[1]==1:
        axis="Y"
    elif array[2]==1:
        axis="Z"
    else :
        axis=""
    angle = array[3]
    return rot2(axis,angle)

def euleur2rotation(rpy):
    tx=rpy[0]
    ty=rpy[1]
    tz=rpy[2]
    cx=cos(tx);sx=sin(tx)
    cy=cos(ty);sy=sin(ty)
    cz=cos(tz);sz=sin(tz)
    R=np.zeros((3,3))

    R[0][0] =cy*cz                                                     
    R[1][0] =cy*sz                                                     
    R[2][0] =-sy  

    R[0][1] =cz*sx*sy-cx*sz                                            
    R[1][1] =cx*cz+sx*sy*sz                                            
    R[2][1] =cy*sx                                                     
                                                                            
    R[0][2] =cx*cz*sy+sz*sx                                            
    R[1][2] =-cz*sx+cx*sy*sz                                           
    R[2][2] =cx*cy 
    return R
    
