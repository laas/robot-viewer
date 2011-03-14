#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2010
# Authors Duong Dang

# auxilaries functions that don't belong to any class


import numpy as np
import re
from math import sin,cos,sqrt,acos,pi,atan2

def norm(a):
    return sqrt(np.dot(a,a))

def normalized(v):
    return np.array([e/norm(v) for e in v])

def rot2AngleAxis(M):
    """
    Convert Transformation to angle, axis reprensentation (angle in degree)
    """
    axisAngle = rot2AxisAngle(M)
    return [axisAngle[3]*180/pi, axisAngle[0],axisAngle[1],axisAngle[2]]


# http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/index.htm
def rot2AxisAngle(M):
    """
    Convert Transformation to axis, angle reprensentation (angle in radian)
    """
    epsilon = 1e-6
    angle=acos( min ( (M[0][0]+M[1][1]+M[2][2]-1)/2 ,1))
    if abs(angle) < epsilon:
        v = [1,0,0]
    else:
        v=np.array( [M[2][1]-M[1][2], M[0][2]-M[2][0], M[1][0]-M[0][1] ])
        normv=sqrt(np.dot(v,v))
        if normv < 1e-6 or abs(pi-angle) < epsilon:
            if M[0][0]+1 > epsilon:
                v[0] = sqrt((M[0][0] + 1)/2)
                v[1] = M[0][1]/2/v[0]
                v[2] = M[0][2]/2/v[0]
            elif M[1][1]+1 > epsilon:
                v[1] = sqrt((M[1][1] + 1)/2)
                v[0] = M[1][0]/2/v[1]
                v[2] = M[1][2]/2/v[1]
            elif M[2][2]+1 > epsilon:
                v[2] = sqrt((M[2][2] + 1)/2)
                v[0] = M[2][0]/2/v[2]
                v[1] = M[2][1]/2/v[2]
            else:
                raise Exception("mathaux:rot2AngleAxis:Uncaught corner case for: %s"
                                %str(M))
            return [v[0],v[1],v[2],angle]
        v=v/normv
    return [v[0],v[1],v[2],angle]

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

def axisNameAngle2rot(axis,a):
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

# http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm
def axisAngle2rot(array):
    array = np.array(array)
    array[0:3] = normalized(array[0:3])

    x = array[0]
    y = array[1]
    z = array[2]
    angle = array[3]

    if angle == 0:
        return np.eye(3)

    c = cos(angle)
    s = sin(angle)
    t = 1 - c

    return np.array([[t*x*x + c  , t*x*y - z*s , t*x*z + y*s],
                     [t*x*y + z*s, t*y*y + c   , t*y*z - x*s],
                     [t*x*z - y*s, t*y*z + x*s , t*z*z +c]
                     ]
                    )

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

def rot2rpy(R):
    alpha = atan2(R[1][0],R[0][0])
    beta = atan2(-R[2][0],sqrt(R[2][1]**2 + R[2][2]**2   ))
    gamma = atan2(R[2][1],R[2][2])
    return [alpha, beta, gamma]
