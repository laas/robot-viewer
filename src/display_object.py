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
# OpenGL.FORWARD_COMPATIBLE_ONLY = True
try:
    import OpenGL
    from OpenGL.GL import *
    from OpenGL.GLUT import *
    from OpenGL.GLU import *
    from OpenGL.GL.ARB.vertex_buffer_object import *
except:
    print "Did not import OpenGL"
import numpy, time
from safeeval import safe_eval
import traceback
import numpy.linalg, numpy
import transformations as tf
import math
import mathaux
import vrml.standard_nodes as nodes
from vbo import Vbo
shaders = {}

import logging, os, sys
logger = logging.getLogger("robotviewer.display_object")

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

#logger.addHandler(NullHandler())

USE_VBO = False
MODERN_SHADER = True


def ifenabled(f):
    def new_f(cls, *args, **kwargs):
        if cls.enabled:
            return f(cls, *args, **kwargs)
        else:
            return
    return new_f


class DisplayObject(object):
    def __init__(self):
        self.app_gl_list = {}
        self.geo_gl_list = {}
        self.enabled = True
        self.vbos = {}
        self.children = []
        self.appearance = None
        self.geometry   = None
        self.globalTransformation = numpy.eye(4)
        self.exclude_cameras = None
        self.stamp = None
    # def __del__(self):
    #     for child in self.children:
    #         del child
    #     object.__del__(self)

    @property
    def shape_list(self):
        pass

    @property
    def joint_list(self):
        pass

    @property
    def shape_list(self):
        pass

    @property
    def bone_list(self):
        pass

    @property
    def type(self):
        return self.__class__.__name__

    def render(self):
        self.non_recursive_render()
        for child in self.children:
            child.render()

    @ifenabled
    def non_recursive_render(self):
        if not (self.appearance or self.geometry):
            return
        glColor3f(0., 0., 0.)
        win = glutGetWindow()

        if not self.app_gl_list.get(win):
            self.app_gl_list[win] = self.generate_app_gl_list()

        if not self.geo_gl_list.get(win):
            self.geo_gl_list[win] = self.generate_geo_gl_list()

        glPushMatrix()

        location = self.globalTransformation[:3,3]
        glTranslatef(*location)

        ag = mathaux.rot2AxisAngle(self.globalTransformation)
        glRotated(ag[3]*180./math.pi,*(ag[:3]))

        if not MODERN_SHADER:
            glCallList(self.app_gl_list[win])


        elif self.appearance and self.appearance.material:
            shaders[glutGetWindow()].uMaterialSpecularColor = self.appearance.material.specularColor + [1.]
            shaders[glutGetWindow()].uMaterialEmissiveColor = self.appearance.material.emissiveColor + [1.]
            shaders[glutGetWindow()].uMaterialDiffuseColor = self.appearance.material.diffuseColor + [1.]
            shaders[glutGetWindow()].uMaterialShininess = self.appearance.material.shininess

            shaders[glutGetWindow()].uMaterialAmbientIntensity = self.appearance.material.ambientIntensity



        if not USE_VBO:
            geo_list = self.geo_gl_list.get(win)
            if geo_list:
                glCallList(geo_list)
            glPopMatrix()
            return

        if not self.vbos.get(win):
            self.vbos[win] = Vbo(self)

        vbo = self.vbos[win]


        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_VERTEX_ARRAY);
        # print
        # self.vbo.ver_vboId,self.vbo.nor_vboId,
        # self.vbo.idx_vboId
        # before draw, specify vertex and index arrays with their offsets
        # Use VBO
        try:
            glBindBufferARB(GL_ARRAY_BUFFER_ARB, vbo.ver_vboId);
            glVertexPointer( 3, GL_FLOAT, 0, None );

            glBindBufferARB(GL_ARRAY_BUFFER_ARB, vbo.nor_vboId);
            glNormalPointer(GL_FLOAT, 0,None);

            glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,
                            vbo.tri_idx_vboId);
            glDrawElements(GL_TRIANGLES, vbo.tri_count,
                           GL_UNSIGNED_SHORT, None);

            glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,
                            vbo.quad_idx_vboId);
            glDrawElements(GL_QUADS, vbo.quad_count,
                           GL_UNSIGNED_SHORT, None);
            for i, vboId in enumerate(vbo.poly_idx_vboIds):
                glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, vboId);
                glDrawElements(GL_POLYGON, len(vbo._poly_idxs),
                               GL_UNSIGNED_SHORT, None);
        except:
            logger.exception("Error while drawing parent %s"%obj.aname)
        glPopMatrix()
        glFlush()
        # end drawing the bot
        glDisableClientState(GL_VERTEX_ARRAY);  # disable vertex arrays
        glDisableClientState(GL_NORMAL_ARRAY);


    def generate_app_gl_list(self):
        new_list = glGenLists(1)
        logger.debug("Generated new gllist for a shape {0}".format(new_list))

        app = self.appearance
        if not app:
            app = nodes.Appearance()
        glNewList(new_list, GL_COMPILE)
        # if not app.transparency:
        #     app.transparency = 0
        # elif type(app.transparency) == list:
        #     app.transparency = app.transparency[0]
        #print app.material.diffuseColor
        if not app.material:
            app.material = nodes.Material()
        ambientColor = [app.material.ambientIntensity*app.material.diffuseColor[i]
                        for i in range(3)]
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,
                     app.material.diffuseColor + [1.])
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientColor + [1.])
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, app.material.shininess)
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, app.material.emissiveColor + [1.] )
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, app.material.specularColor + [1.] )

        glEndList();
        return new_list

    def generate_geo_gl_list(self):
        if not self.geometry:
            return
        new_list = glGenLists(1)
        glNewList(new_list, GL_COMPILE)
        self.geometry.render(self.cumul_scale())
        glEndList()
        return new_list

