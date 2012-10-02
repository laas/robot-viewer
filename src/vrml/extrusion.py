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


try:
    from OpenGL.GL import *
    from OpenGL.GLUT import *
    from OpenGL.GLU import *
except:
    print "did not import OpenGL"

import numpy
import parser
import standard_nodes as nodes

from abstract_geometry import Geometry

import logging
class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger = logging.getLogger("robotviewer.vrml.box")
logger.addHandler(NullHandler())


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
