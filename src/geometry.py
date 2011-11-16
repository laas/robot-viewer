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

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

SHOW_NORMALS = False
logger = logging.getLogger("robotviewer.shape")
logger.addHandler(NullHandler())

class ElevationGrid(nodes.ElevationGrid):
    def init(self):
        pass


    def render(self, scale):
        Xs = [self.xSpacing*i*scale[0] for i in range(self.xDimension)]
        Zs = [self.zSpacing*i*scale[2] for i in range(self.zDimension)]
        for i in range(self.xDimension - 1):
            for j in range(self.zDimension -1):
                A = Xs[i], 0, Zs[j]
                B = Xs[i], 0, Zs[j+1]
                C = Xs[i+1], 0, Zs[j+1]
                D = Xs[i+1], 0, Zs[j]

                count = i*(self.xDimension-1) + j
                color =  self.color.color[3*count:3*count+3]
                glColor3f(color[0], color[1], color[2])
                glBegin(GL_QUADS)
                glNormal3f(0,1,0)
                glVertex3f(A[0], A[1], A[2])
                glNormal3f(0,1,0)
                glVertex3f(B[0], B[1], B[2])
                glNormal3f(0,1,0)
                glVertex3f(C[0], C[1], C[2])
                glNormal3f(0,1,0)
                glVertex3f(D[0], D[1], D[2])
                glEnd()


class IndexedFaceSet(nodes.IndexedFaceSet):
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

    def render(self, scale):
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
