#! /usr/bin/env python

__author__ = "Duong Dang"
__version__ = "0.1"

import logging, sys, os
import ml_parser

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("sketcher")
logger.addHandler(NullHandler())
logger.setLevel(logging.DEBUG)
import transformations as tf
import math, numpy, numpy.linalg
prologue="""
def ee 0.000001

def O (0,0,0) % origo
def J [0,1,0] % rotation.jointAxis
def dx 2
def dy dx
def dz dx
def axes {
    % draw the axes
    def ax (dx,0,0)
    def ay (0,dy,0)
    def az (0,0,dz)
    line[arrows=->,line width=.4pt, draw=red](O)(ax)
    line[arrows=->,line width=.4pt, draw=green](O)(ay)
    line[arrows=->,line width=.4pt, draw=blue](O)(az)
    % annote axes
    special |\path #1 node[left] {$z$}
                   #2 node[below] {$x$}
                   #3 node[below] {$y$};|(az)(ax)(ay)
}



def jR .2
def jR_inner 0.05
def jH 1.0
def joint  {
    def n 20
    sweep[fill=blue!20, cull=false, draw=gray]{n<>, rotate(360/n,[0,1,0])}
        line[draw=gray,fill opacity=0.8,cull=false,fill=blue!20]
            (jR,-jH/2)(jR,jH/2)
    sweep[fill=darkgray, cull=false]{n<>, rotate(360/n,[0,1,0])}
        line(jR_inner,-jH/2-ee)(jR_inner,jH/2+ee)
}

def viewpoint (2.5, 1, 1)
def lookat (0, 0, 0)
% Set upvector along the z.jointAxis
def upvector [0,0,1]


"""

epilogue="""
global { language tikz }
"""



def sketch(robot):
    res = ""
    res += "def robot {\n"
    oldT = numpy.eye(4)
    for j in robot.moving_joint_list[:20]:
        T = j.globalTransformation
        T1 = numpy.eye(4)
        if j.jointAxis in ['y','Y']:
            T1 = tf.rotation_matrix(math.pi/2, [0,0,1])
        if j.jointAxis in ['z','Z']:
            T1 = tf.rotation_matrix(math.pi/2, [0,1,0])
        T = numpy.dot(T,T1)
        relT = numpy.dot(numpy.linalg.inv(oldT),T)
        angle, direc, point = tf.rotation_from_matrix(relT)
        print T
        p = relT[:3,3]
        p = [w*10 for w in p]
        res += """put {rotate (%f,(%f, %f, %f),[%f, %f, %f]) then translate ([%f, %f, %f])} {joint}
"""%(angle*180/math.pi,0,0,0, direc[0], direc[1], direc[2], p[0],p[1],p[2])
        oldT = T

    res += "}\n"
    return res

def main():
    import optparse
    parser = optparse.OptionParser(
        usage='\n\t%prog [options]',
        version='%%prog %s' % __version__)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")
    (options, args) = parser.parse_args(sys.argv[1:])

    result = prologue

    p = ml_parser.parse(args[0])
    robot = p[0]
    robot.init()
    result += sketch(robot)


    result += """
put{ view((viewpoint),(lookat), [upvector])} {
    {robot}{axes}
}
"""
    result += epilogue
    f = open("test.sk",'w')
    f.write( result)
    f.close()

if __name__ == '__main__':
    main()

