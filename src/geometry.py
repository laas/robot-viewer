# Copyright (c) 2010-2011, Duong Dang <mailto:dang.duong@gmail.com>
# This file is part of robot-viewer.

# robot-viewer is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# robot-viewer is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with robot-viewer.  If not, see <http://www.gnu.org/licenses/>.
#! /usr/bin/env python

import numpy
import numpy.linalg
import re
from math import sin, cos, isnan, pi, acos
from collections import deque
import logging, uuid
import __builtin__
import traceback
import vrml.parser
import vrml.standard_nodes as nodes
import OpenGL
# OpenGL.FORWARD_COMPATIBLE_ONLY = True
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL.ARB.vertex_buffer_object import *
from abc import ABCMeta, abstractmethod
import numpy
import numpy.linalg
import pprint
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

SHOW_NORMALS = False
logger = logging.getLogger("robotviewer.shape")
logger.addHandler(NullHandler())


# description of the ccw, solid, and creaseAngle fields are ignored for all nodes
class Geometry(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def render(self, scale):
        return

    def init(self):
        return

class Box(nodes.Box, Geometry):
    def render(self, scale = 3*[1.0]):
        sizex = self.size[0]*scale[0]
        sizey = self.size[1]*scale[1]
        sizez = self.size[2]*scale[2]
        count = 0
        v = []
        for x in [1, -1]:
            for y in [1, -1]:
                for z in [1, -1]:
                    v.append([x*sizex,y*sizey,z*sizez])
        v0 = v[0]
        v1 = v[1]
        v2 = v[2]
        v3 = v[3]
        v4 = v[4]
        v5 = v[5]
        v6 = v[6]
        v7 = v[7]
        glBegin(GL_QUADS)
        glNormal3f(1,0,0)
        glVertex3fv(v0)    # front face
        glVertex3fv(v2)
        glVertex3fv(v3)
        glVertex3fv(v1)

        glNormal3f(0,1,0)
        glVertex3fv(v0)    # right face
        glVertex3fv(v1)
        glVertex3fv(v5)
        glVertex3fv(v4)

        glNormal3f(0,0,1)
        glVertex3fv(v0)    # up face
        glVertex3fv(v4)
        glVertex3fv(v6)
        glVertex3fv(v2)

        glNormal3f(-1,0,0)
        glVertex3fv(v4)    # back face
        glVertex3fv(v5)
        glVertex3fv(v7)
        glVertex3fv(v6)

        glNormal3f(0,-1,0)
        glVertex3fv(v2)    # left face
        glVertex3fv(v6)
        glVertex3fv(v7)
        glVertex3fv(v3)

        glNormal3f(0,0,-1)
        glVertex3fv(v1)    # down face
        glVertex3fv(v3)
        glVertex3fv(v7)
        glVertex3fv(v5)
        glEnd()

class Cone(nodes.Cone, Geometry):
    quad = gluNewQuadric()
    def render(self, scale = 3*[1.]):
        # print self.bottomRadius, self.height, self.side, self.bottom
        glRotatef(-90, 1, 0, 0)
        glTranslatef(0, 0, -self.height*scale[2]/2)
        if self.side:
            gluCylinder(self.quad, self.bottomRadius*scale[0],
                        0.0, self.height*scale[2], 20,20)
        if self.bottom:
            glRotatef(180, 1., 0., 0.)
            gluDisk(self.quad, 0., self.bottomRadius*scale[0], 20, 20)

class Cylinder(nodes.Cylinder, Geometry):
    quad = gluNewQuadric()
    def render(self, scale = 3*[1.]):
        glRotatef(-90, 1, 0, 0)
        glTranslatef(0, 0, -self.height*scale[2]/2)

        if self.side:
            gluCylinder(self.quad, self.radius*scale[0], self.radius*scale[0],
                        self.height*scale[2], 20,20)
        if self.bottom:
            glPushMatrix()
            glRotatef(180, 1., 0., 0.)
            gluDisk(self.quad, 0., self.radius*scale[0], 20, 20)
            glPopMatrix()

        if self.top:
            glPushMatrix()
            glTranslatef(0, 0, self.height*scale[2])
            gluDisk(self.quad, 0., self.radius*scale[0], 20, 20)
            glPopMatrix()


class Sphere(nodes.Sphere, Geometry):
    quad = gluNewQuadric()
    def render(self, scale = 3*[1.]):
        gluSphere(self.quad, self.radius*scale[0], 20, 20)


class PointSet(nodes.PointSet, Geometry):
    quad = gluNewQuadric()
    def render(self, scale = 3*[1.]):
        points = zip(self.coord.point[::3],
                     self.coord.point[1::3],
                     self.coord.point[2::3],
                     )
        colors = zip(self.color.color[::3],
                     self.color.color[1::3],
                     self.color.color[2::3],
                     )
        for i, p in enumerate(points):
            c = colors[i]
            glColor3fv(c)
            glPushMatrix()
            glTranslatef(*p)
            gluSphere(self.quad, 0.01, 10, 10)
            glPopMatrix()

class IndexedLineSet(nodes.IndexedLineSet, Geometry):
    def render(self, scale = 3*[1.]):
        points = zip(self.coord.point[::3],
                     self.coord.point[1::3],
                     self.coord.point[2::3],
                     )
        colors = zip(self.color.color[::3],
                     self.color.color[1::3],
                     self.color.color[2::3],
                     )
        lines = []
        line = []
        for idx in self.coordIndex:
            if idx == -1:
                lines.append(line)
                line = []
            line.append(idx)
        for i, line in enumerate(lines):
            glColor(colors[self.colorIndex[i]])
            glBegin(GL_LINE_STRIP)
            for j in line:
                glVertex3fv( points[j] )
            glEnd()
        return

class ElevationGrid(nodes.ElevationGrid, Geometry):
    def render(self, scale = 3*[1.]):
        Xs = [self.xSpacing*i*scale[0] for i in range(self.xDimension)]
        Zs = [self.zSpacing*i*scale[2] for i in range(self.zDimension)]
        for i in range(self.xDimension - 1):
            for j in range(self.zDimension -1):
                A = Xs[i],   self.height[i   + (j)  *self.zDimension], Zs[j]
                B = Xs[i],   self.height[i   + (j+1)*self.zDimension], Zs[j+1]
                C = Xs[i+1], self.height[i+1 + (j+1)*self.zDimension], Zs[j+1]
                D = Xs[i+1], self.height[i+1 + (j)  *self.zDimension], Zs[j]

                count = i + j*(self.zDimension-1)
                color =  self.color.color[3*count:3*count+3]

                glColor3f(color[0], color[1], color[2])
                glBegin(GL_QUADS)
                glNormal3f(0,1,0)
                glVertex3f(A[0], A[1], A[2])
                glVertex3f(B[0], B[1], B[2])
                glVertex3f(C[0], C[1], C[2])
                glVertex3f(D[0], D[1], D[2])
                glEnd()

class Extrusion(nodes.Extrusion, Geometry):
    def render(self, scale = 3*[1.]):
        """
        Start with the cross section as specified, in the XZ plane.  Scale it
        about (0, 0, 0) by the value for scale given for the current joint.
        Apply a rotation so that when the cross section is placed at its proper
        location on the spine it will be oriented properly. Essentially, this
        means that the cross section's Y axis (up vector coming out of the
        cross section) is rotated to align with an approximate tangent to the
        spine curve.

        For all points other than the first or last: The tangent for spine[i]
        is found by normalizing the vector defined by (spine[i+1] -
        spine[i-1]).

        If the spine curve is closed: The first and last points need to have
        the same tangent. This tangent is found as above, but using the points
        spine[0] for spine[i], spine[1] for spine[i+1] and spine[n-2] for
        spine[i-1], where spine[n-2] is the next to last point on the
        curve. The last point in the curve, spine[n-1], is the same as the
        first, spine[0].

        If the spine curve is not closed: The tangent used for the first point
        is just the direction from spine[0] to spine[1], and the tangent used
        for the last is the direction from spine[n-2] to spine[n-1].

        In the simple case where the spine curve is flat in the XY plane, these
        rotations are all just rotations about the Z axis. In the more general
        case where the spine curve is any 3D curve, you need to find the
        destinations for all 3 of the local X, Y, and Z axes so you can
        completely specify the rotation. The Z axis is found by taking the
        cross product of:

        (spine[i-1] - spine[i]) and (spine[i+1] - spine[i]).

        If the three points are collinear then this value is zero, so take the
        value from the previous point. Once you have the Z axis (from the cross
        product) and the Y axis (from the approximate tangent), calculate the X
        axis as the cross product of the Y and Z axes.  Given the plane
        computed in step 3, apply the orientation to the cross-section relative
        to this new plane. Rotate it counter-clockwise about the axis and by
        the angle specified in the orientation field at that joint.  Finally,
        the cross section is translated to the location of the spine point.
        """

        spines = zip(self.spine[::3],self.spine[1::3],self.spine[2::3] )
        spines = [numpy.array(s) for s in spines]

        closed_spine = True
        for i in range(3):
            if spines[0][i] != spines[-1][i]:
                closed_spine = False

        cs_vts = []
        for i, v in enumerate(zip(self.crossSection[::2],
                                 self.crossSection[1::2])):
            cs_vts.append(numpy.array([v[0]*self.scale[0], #x
                                       0 ,
                                       v[1]*self.scale[1]]))    #z (crosssection: xz plane)

        print cs_vts
        # orientation of local coordinate axes at spine points
        Ys = [None for i in range(len(spines))]
        Zs = [None for i in range(len(spines))]
        Xs = [None for i in range(len(spines))]


        for i in range(len(spines)):
            last_t = None
            next_t = None

            if i == 0:
                next_t = spines[1] - spines[0]
                if closed_spine:
                    last_t = spines[-1] - spines[0]

            elif i == len(spines) - 1:
                last_t = spines[i] - spines[i-1]
                if closed_spine:
                    next_t = spines[0] - spines[-1]
            else:
                last_t = spines[i] - spines[i-1]
                next_t = spines[i+1] - spines[i]

            if last_t == None:
                Ys[i] = next_t
            elif next_t == None:
                Ys[i] = last_t
            else:
                Ys[i] = (next_t + last_t)/2
                Zs[i] = numpy.cross(next_t, last_t)
                if numpy.linalg.norm(Zs[i]) < 1e-6:
                    Zs[i] == None


        for i, Z in enumerate(Zs):
            if Z == None:
                if Zs[i-1] != None:
                    Zs[i] = Zs[i-1]
                elif Zs[i+1] != None:
                    Zs[i] = Zs[i+1]
        if Zs[0] == None:
            Zs = [numpy.array([0,0,1]) for i in range(len(Zs))]

        for i in range(len(Xs)):
            Xs[i] = numpy.cross(Ys[i], Zs[i])
            Xs[i] = Xs[i]/numpy.linalg.norm(Xs[i])
            Ys[i] = Ys[i]/numpy.linalg.norm(Ys[i])
            Zs[i] = Zs[i]/numpy.linalg.norm(Zs[i])

        ## list of 3d cross section vertices
        cs_3ds = []
        for i in range(len(spines)):
            R = numpy.identity(3)
            R[:3,0] = Xs[i]
            R[:3,1] = Ys[i]
            R[:3,2] = Zs[i]
            t = spines[i]
            cs_3d = [ numpy.dot(R, p) + t for p in cs_vts]
            cs_3ds.append(cs_3d)

        glBegin(GL_QUADS)
        for i in range(len(cs_3ds)-1):
            cs0 = cs_3ds[i]
            cs1 = cs_3ds[i+1]
            for j in range(len(cs0)):
                j1 = j + 1
                if j1 == len(cs0):
                    j1 = 0

                n = -numpy.cross(cs1[j] - cs0[j], cs0[j1] - cs0[j])
                glNormal3fv(n)
                glVertex3fv(cs0[j1]*scale[0])
                glVertex3fv(cs1[j1]*scale[0])
                glVertex3fv(cs1[j]*scale[0])
                glVertex3fv(cs0[j]*scale[0])

                glNormal3fv(-n)
                glVertex3fv(cs0[j1]*scale[0])
                glVertex3fv(cs0[j]*scale[0])
                glVertex3fv(cs1[j]*scale[0])
                glVertex3fv(cs1[j1]*scale[0])

        glEnd()

        if self.beginCap:
            glBegin(GL_POLYGON)
            cs = cs_3ds[0]
            n = numpy.cross(cs[2] - cs[1], cs[0] - cs[1])
            glNormal3fv(n)
            for v in cs:
                glVertex3fv(v*scale[0])
            glEnd()
            glBegin(GL_POLYGON)
            glNormal3fv(-n)
            for v in reversed(cs):
                glVertex3fv(v*scale[0])
            glEnd()


        if self.endCap:
            glBegin(GL_POLYGON)
            cs = cs_3ds[-1]
            n = numpy.cross(cs[2] - cs[1], cs[0] - cs[1])
            glNormal3fv(n)
            for v in cs:
                glVertex3fv(v*scale[0])
            glEnd()
            glBegin(GL_POLYGON)
            glNormal3fv(-n)
            for v in reversed(cs):
                glVertex3fv(v*scale[0])
            glEnd()



        # ignore self.orientation, this should be determined automatically with
        # spines
        print "spines: ", spines


class Text(nodes.Text, Geometry):
    def render(self, scale = 3*[1.]):
        logger.info("Trying to render '{0}'".format(self.string))
        logger.fatal("Text rendering has not been implented")

class IndexedFaceSet(nodes.IndexedFaceSet, Geometry):
    def __init__(self):
        nodes.IndexedFaceSet.__init__(self)
        self.coord = None
        self.coordIndex = []
        self.tri_idxs  = []
        self.quad_idxs = []
        self.poly_idxs = []
        self.tri_count  = 0
        self.quad_count = 0
        self.normal = None

    def __str__(self):
        s="Geometry:"
        s+="%d points and %d faces"%(len(self.coord.point)/3,len(self.coordIndex)/4)
        return s

    # def scale(self,scale_vec):
    #     if len(scale_vec) !=3 :
    #         raise Exception("Expected scale_vec of dim 3, got %s"%str(scale_vec))

    #     scale_x = scale_vec[0]
    #     scale_y = scale_vec[1]
    #     scale_z = scale_vec[2]

    #     for i in range(len(self.coord.point)/3):
    #         self.coord.point[3*i]   *= scale_x
    #         self.coord.point[3*i+1] *= scale_y
    #         self.coord.point[3*i+2] *= scale_z

    def init(self):
        self.compute_normals()

    def render(self, scale = 3*[1.]):
        logger.debug("Generating glList for {0}".format(self))

        if not (self.tri_idxs[:] and self.normal
            and self.normal.vector):
            self.compute_normals()

        glBegin(GL_TRIANGLES)
        for i in self.tri_idxs:
            n = [ self.normal.vector[i][0],
                  self.normal.vector[i][1],
                  self.normal.vector[i][2],
                  ]
            v =  [ self.coord.point[3*i],
                   self.coord.point[3*i+1],
                   self.coord.point[3*i+2],
                   ]

            v[0] *= scale[0]
            v[1] *= scale[1]
            v[2] *= scale[2]

            logger.debug("object  {0}: {1} {2}".format(id(self), v, n))
            glNormal3f( n[0], n[1], n[2])
            glVertex3f( v[0], v[1], v[2])
        glEnd()

        if SHOW_NORMALS:
            glBegin(GL_LINES)
            for i in self.tri_idxs:
                n = [ self.normal.vector[i][0],
                      self.normal.vector[i][1],
                      self.normal.vector[i][2],
                      ]

                v =  [ self.coord.point[3*i],
                       self.coord.point[3*i+1],
                       self.coord.point[3*i+2],
                       ]

                glVertex3f(v[0], v[1], v[2])
                glVertex3f(v[0] + 0.01*n[0],
                           v[1] + 0.01*n[1],
                           v[2] + 0.01*n[2],
                           )
            glEnd()


    def compute_normals(self):
        if self.normal == None:
            self.normal = nodes.Normal()

        if self.normal.vector[:]:
            return
        logger.debug("Computing normals in {0}. len(self.coordIndex)={1}, self.normal= {2}, self.normal.vector = {3}"
                     .format(self, len(self.coordIndex), self.normal, repr(self.normal.vector)[:100]))
        npoints=len(self.coord.point)/3

        if self.normal.vector == []:
            normals=[]
            points=[]
            for k in range(npoints):
                normals.append(numpy.array([0.0,0.0,0.0]))
                points.append(numpy.array([self.coord.point[3*k],self.coord.point[3*k+1],
                                           self.coord.point[3*k+2]]))

        poly=[]
        ii=0
        for a_idx in self.coordIndex:
            if a_idx!=-1:
                poly.append(a_idx)
                continue
            num_sides = len(poly)
            # if num_sides not in (3,4):
            #     logger.warning("""n=%d.  Only support tri and quad shape for
            #                       the moment"""%num_sides)
            #     poly=[]
            #     continue
            # a_idx=-1 and poly is a triangle
            if num_sides == 3:
                self.tri_idxs += poly
            elif num_sides == 4:
                self.quad_idxs += poly
            else:
                self.poly_idxs.append(poly)
            if self.normal.vector == []:
                # update the norm vector
                # update the normals using G. Thurmer, C. A. Wuthrich,
                # "Computing vertex normals from polygonal facets"
                # Journal of Graphics Tools, 3 1998
                ids  = (num_sides)*[None]
                vecs = []
                for i in range(num_sides):
                    vecs.append((num_sides)*[None])
                alphas = (num_sides)*[0]
                for i, iid in enumerate(poly):
                    ids[i] = iid

                vertices = []
                for i in range(num_sides):
                    vertices.append(points[ids[i]])
                    j = i + 1
                    if j == num_sides:
                        j = 0
                    vecs[j][i] = (points[ids[j]] - points[ids[i]])
                    vecs[j][i] /= numpy.linalg.norm(vecs[j][i])


                try:
                    for i in range(num_sides):
                        if i == 0:
                            vec1 = vecs[1][0]
                            vec2 = vecs[0][num_sides-1]
                        elif i == num_sides - 1:
                            vec1 =   vecs[0][num_sides-1]
                            vec2 =   vecs[num_sides-1][num_sides-2]
                        else:
                            vec1 = vecs[i][i-1]
                            vec2 = vecs[i+1][i]

                        if abs(numpy.dot(vec1, vec2) + 1) < 1e-6:
                            alphas[i] = pi
                        elif abs(numpy.dot(vec1, vec2) - 1) < 1e-6:
                            alphas[i] = 0
                        else:
                            try:
                                alphas[i] = acos(numpy.dot(vec1, vec2))
                            except ValueError:
                                print "Math domain error: acos({0})".format(numpy.dot(vec1, vec2))


                except Exception,error:
                    s = traceback.format_exc()
                    logger.warning("Shape processing error: %s"%s)
                for i,alpha in enumerate(alphas):
                    if i == 0:
                        normal_i = numpy.cross(vecs[0][num_sides-1],vecs[1][0])
                    elif i == num_sides - 1:
                        normal_i = numpy.cross(vecs[num_sides-1][num_sides-2],
                                               vecs[0][num_sides-1])
                    else:
                        normal_i = numpy.cross(vecs[i][i-1], vecs[i+1][i])
                    if isnan(alphas[i]) or abs(alpha - pi) < 1e-6:
                        # print "alpha is NaN for", vertices
                        continue
                    normals[ids[i]] += alpha*normal_i/numpy.linalg.norm(normal_i)
                    #if isnan(normals[ids[i]][0]):
                    #    print "produced invalid normal", alpha, normal_i, vertices
            poly=[]

        self.normal.vector = normals


