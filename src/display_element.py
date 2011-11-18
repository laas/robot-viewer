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
import OpenGL
# OpenGL.FORWARD_COMPATIBLE_ONLY = True
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL.ARB.vertex_buffer_object import *
import numpy, time
import kinematics
from safeeval import safe_eval
from kinematics import Robot, GenericObject
import traceback
import vrml.standard_nodes as nodes
import numpy.linalg, numpy
import transformations as tf
import math
import mathaux

shaders = {}

import logging, os, sys
logger = logging.getLogger("robotviewer.display_element")

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

#logger.addHandler(NullHandler())

SHOW_NORMALS = False
USE_VBO = False
MODERN_SHADER = False

def ifenabled(meth):
    def new_meth(cls, *args, **kwargs):
        if cls.enabled:
            return meth(cls, *args, **kwargs)
        else:
            return
    return new_meth


class GlPrimitive(GenericObject):
    """
    """
    def __init__(self, gl_list_ids = None, vbos = None,
                 shape = None, script = None, parent = None):
        """

        Arguments:
        - `glList`:
        - `VBO`:
        """
        GenericObject.__init__(self)
        self.gl_list_ids = {}
        self.vbos = {}
        self.enabled = True
        self.script = script
        self.shape = shape
        if parent:
            parent.add_child(self)
            setattr(parent, "gl_primitive", self)

        if gl_list_ids:
            self.gl_list_ids = gl_list_ids
        if vbos:
            self.vbos = vbos

    def set_transparency(self, transparency):
        pass

    def generate_gl_list(self):
        if self.shape:
            return self.generate_gl_list_shape()
        if self.script:
            return self.generate_gl_list_script()

    def generate_gl_list_script(self):
        new_list = glGenLists(1)
        logger.debug("Generated new gllist for a script {0}".format(new_list))
        glNewList(new_list, GL_COMPILE);
        safe_eval(self.script, globals())
        glEndList();
        return new_list

    def generate_gl_list_shape(self):
        new_list = glGenLists(1)
        logger.debug("Generated new gllist for a shape {0}".format(new_list))

        app = self.shape.appearance
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

        if not USE_VBO:
            scale = self.cumul_scale()
            self.shape.geometry.render(scale)

        glEndList();
        return new_list

    @ifenabled
    def render(self):
        glColor3f(0., 0., 0.)

        win = glutGetWindow()
        if not self.gl_list_ids.get(win):
            self.gl_list_ids[win] = self.generate_gl_list()

        glPushMatrix()

        glTranslatef(*self.globalTransformation[:3,3])

        # angle, direction, point = tf.rotation_from_matrix(self.globalTransformation)
        # glRotated(angle*180./math.pi, direction[0], direction[1], direction[2])
        ag = mathaux.rot2AxisAngle(self.globalTransformation)
        glRotated(ag[3]*180./math.pi,*(ag[:3]))

        # logger.debug("Caling glList {0} at ({1}, {2})".format(self.gl_list_ids[win],
        #                                                       p, agax))
        if self.shape:
            app = self.shape.appearance
            if not app:
                app = nodes.Appearance()
            if not app.material:
                app.material = nodes.Material()

            if MODERN_SHADER:
                    shaders[glutGetWindow()].uMaterialSpecularColor = app.material.specularColor
                    shaders[glutGetWindow()].uMaterialEmissiveColor = app.material.emissiveColor
                    shaders[glutGetWindow()].uMaterialDiffuseColor = app.material.diffuseColor
                    shaders[glutGetWindow()].uMaterialShininess = app.material.shininess

            glCallList(self.gl_list_ids[win])


        if (not USE_VBO) or (not self.shape) :
            glPopMatrix()
            return
        if not self.vbos.get(win):
            self.vbos[win] = Vbo(self.shape)

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

class DisplayObject(object):
    """
    """

    def __init__(self, obj):
        self.obj = obj
        self.pending_update = False
        self.config = None
        self.enabled = True
        for shape in self.shape_list:
            shape.add_child( GlPrimitive (shape = shape, parent = shape) )
        self.pending_display_change = False

    def __getattr__(self, attr):
        return getattr(self.obj, attr)

    def __str__(self):
        return "FIX-ME"

    @ifenabled
    def render(self):
        """Render element and its children in the scene

        Arguments:
        - `self`:
        """
        if self.pending_update:
            self.pending_update = False
            self.obj.update_config(self.config)

        for m in self.shape_list:
            if not m.gl_primitive:
                continue
            m.gl_primitive.render()

    # def set_transparency(self, transparency):
    #     for shape in self.shape_list:
    #         shape.appearance.material.transparency = transparency
    #         shape.gl_primitive.shape.appearance.material.transparency = transparency
            # shape.gl_primitive.generate_gl_list()


    def get_config(self):
        return self.config


    def update_config(self, config):
        self.pending_update = True
        self.config = config


class JointGlPrimitve(GlPrimitive):
    def __init__(self, *arg, **kwargs):
        GlPrimitive.__init__(self, *arg, **kwargs )
        self.skeleton_size = 2

    def generate_gl_list(self):
        new_list = glGenLists(1)
        glNewList(new_list, GL_COMPILE);
        draw_joint(self.parent, size = self.skeleton_size)
        draw_link(self.parent, size = self.skeleton_size)
        glEndList();
        return new_list

class DisplayRobot(DisplayObject):
    def __init__(self, *args, **kwargs):
        DisplayObject.__init__(self, *args, **kwargs)
        self.skeleton_size = 2
        for joint in self.joint_list:
            primitive = JointGlPrimitve(parent = joint)


    @ifenabled
    def render(self, shape_flag = True,  skeleton_flag = False):
        if self.pending_update:
            self.pending_update = False
            self.obj.update_config(self.config)

        if shape_flag:
            DisplayObject.render(self)

        if skeleton_flag or not self.shape_list[:]:
            self.render_skeleton()

    def render_skeleton(self):
        for j in self.joint_list:
            j.gl_primitive.render()

        # for j in self.joint_list:
        #     parent = j.get_parent_joint()
        #     if (not parent) or j.jointType  in ["free", "freeFlyer"]:
        #         continue
        #     parent_pos = parent.globalTransformation[:3,3]
        #     child_pos = j.globalTransformation[:3,3]
        #     draw_cylinder(child_pos, parent_pos, self.skeleton_size)

    def set_skeleton_size(self, size):
        self.skeleton_size = size
        for j in self.joint_list:
            j.gl_primitive.skeleton_size = size
            for winid, list_id in j.gl_primitive.gl_list_ids.items():
                glDeleteLists(list_id, 1)
            j.gl_primitive.gl_list_ids = {}


class Vbo(object):
    """
    """

    def __init__(self, shape):
        """

        Arguments:
        - `shape`:
        """
        self.ver_vboId  = -1
        self.nor_vboId  = -1
        self.tri_idx_vboId  = -1
        self.quad_idx_vboId  = -1
        self.poly_idx_vboIds  = []

        self.verts = shape.geometry.coord.point

        if not shape.geometry.normal.vector[:] or not shape.geometry.tri_idxs[:]:
            shape.geometry.compute_normals()

        self.normal = shape.geometry.normal.vector
        self.tri_idxs  = shape.geometry.tri_idxs
        self.quad_idxs = shape.geometry.quad_idxs
        self.poly_idxs = shape.geometry.poly_idxs
        self.tri_count  = shape.geometry.tri_count
        self.quad_count = shape.geometry.tri_count

        logger.debug("Computing normals")

        logger.debug("Loading to GPUs")
        self.load_gpu(shape)

    def __del__(self):
        for vboid in [self.tri_idx_vboId, self.quad_idx_vboId] + self.poly_idx_vboIds:
            glDeleteBuffersARB(1, vboid)
        object.__del__(self)


    def load_gpu(self, shape):
        logger.debug("Creating VBO for shape %s"%shape.name)
        self.tri_count = len(self.tri_idxs)
        self.quad_count = len(self.quad_idxs)
        self.ver_vboId = int(glGenBuffersARB(1))
        logger.debug("Populating VBO for vertices: vboID %d"%self.ver_vboId)
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,self.ver_vboId );
        glBufferDataARB( GL_ARRAY_BUFFER_ARB,
                             numpy.array (self.verts, dtype=numpy.float32),
                             GL_STATIC_DRAW_ARB );
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,0 );
        logger.debug("Generated VBO for vertices: vboID %d"%self.ver_vboId)

        self.nor_vboId = int(glGenBuffersARB(1))
        logger.debug("Populating VBO for normals: vboID %d"%self.nor_vboId)
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,self.nor_vboId );
        glBufferDataARB( GL_ARRAY_BUFFER_ARB,
                             numpy.array (self.normal, dtype=numpy.float32),
                             GL_STATIC_DRAW_ARB );
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,0 );
        logger.debug("Generated VBO for normals: vboID %d"%self.nor_vboId)

        self.tri_idx_vboId = int(glGenBuffersARB(1))
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,self.tri_idx_vboId );
        glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB,
                             numpy.array (self.tri_idxs, dtype=numpy.uint16),
                             GL_STATIC_DRAW_ARB );
        logger.debug("Generated VBO for triangle indices: vboID %d"%
                     self.tri_idx_vboId)
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,0 );
        logger.debug("Finished creating VBO for shape %s"%shape.name)

        self.quad_idx_vboId = int(glGenBuffersARB(1))
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,self.quad_idx_vboId );
        glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB,
                             numpy.array (self.quad_idxs, dtype=numpy.uint16),
                             GL_STATIC_DRAW_ARB );
        logger.debug("Generated VBO for quadangle indices: vboID %d"%
                     self.quad_idx_vboId)
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,0 );

        for i,poly in enumerate(self.poly_idxs):
            poly_idx_vboId = int(glGenBuffersARB(1))
            self.poly_idx_vboIds.append(poly_idx_vboId)
            glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, poly_idx_vboId );
            glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB,
                             numpy.array (poly, dtype=numpy.uint16),
                             GL_STATIC_DRAW_ARB );
            logger.debug("Generated VBO for poly indices: vboID %d"%
                         poly_idx_vboId)
            glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,0 );

        logger.debug("Finished creating VBO for shape %s"%shape.name)


    def __str__(self):
        """
        """
        s="[Vbo instance:\n"
        s+="ver_vboId\t=%d\n"%self.ver_vboId
        s+="nor_vboId\t=%d\n"%self.nor_vboId
        s+="tri_idx_vboId\t=%d\n"%self.tri_idx_vboId
        s+="quad_idx_vboId\t=%d\n"%self.quad_idx_vboId

        s+="len (_verts)\t=%d\n"%(len(self.verts))
        s+="len (_norms)\t=%d\n"%(len(self.normal))
        s+="len (_idxs)\t=%d\n"%(len(self.tri_idxs))
        s+="]"
        return s


def draw_joint(joint, size = 1):
    for (key, value) in [ (GL_SPECULAR, [1,1,1,1]),
                          (GL_EMISSION, [0.5,0,0,1]),
                          (GL_AMBIENT_AND_DIFFUSE, [0.5,0,0,1]),
                          (GL_SHININESS, 5),
                      ]:
        glMaterialfv(GL_FRONT_AND_BACK, key, value)
    r = 0.01*size
    h = r/2
    glPushMatrix()
    if joint.jointType in ["free", "freeflyer"]:
        sphere = gluNewQuadric()
        gluSphere(sphere,0.01*size,10,10)
        glPopMatrix()
        gluDeleteQuadric(sphere)
        return

    if joint.jointAxis in ("X","x"):
        glRotated(90,0,1,0)
    elif joint.jointAxis in ("Y","y"):
        glRotated(90,1,0,0)
    glTranslated(0.0,0.0,-h/2)
    qua = gluNewQuadric()
    gluCylinder(qua,r,r,h,10,5)
    glTranslated(0.0,0.0,h)
    gluDisk(qua,0,r,10,5)
    glTranslated(0.0,0.0,-h)
    glRotated(180,1,0,0)
    gluDisk(qua,0,r,10,5)
    gluDeleteQuadric(qua)
    glPopMatrix()

def draw_link(joint, size = 1):
    children = joint.get_children_joints()
    for child in children:
        localT = kinematics.find_relative_transformation( joint , child )
        child_pos = localT[:3,3]
        #draw_cylinder([0.,0.,0.], child_pos, size)
        #    print "{0}\t{1}{2}".format(joint.name, child.name, child_pos)

        color = [0,1,0,1]
        for (key, value) in [ (GL_SPECULAR, [1,1,1,1]),
                          (GL_EMISSION, color),
                          (GL_AMBIENT_AND_DIFFUSE, color),
                          (GL_SHININESS, 5),
                          ]:
            glMaterialfv(GL_FRONT_AND_BACK, key, value)
        glBegin(GL_LINES)
        glVertex3f(0.,0.,0.)
        glVertex3f(child_pos[0], child_pos[1], child_pos[2])
        glEnd()

def draw_cylinder(p1, p2, size=1, color = [0,1,0,1]):
    for (key, value) in [ (GL_SPECULAR, [1,1,1,1]),
                          (GL_EMISSION, color),
                          (GL_AMBIENT_AND_DIFFUSE, color),
                          (GL_SHININESS, 5),
                          ]:
        glMaterialfv(GL_FRONT_AND_BACK, key, value)

    r = 0.01*size/4
    p = p2-p1
    h = numpy.linalg.norm(p)
    n_p = p/h
    z_axis = numpy.array([0,0,1])
    axis = numpy.cross(z_axis, n_p)
    angle = acos(numpy.dot(z_axis, n_p))*180/pi
    glPushMatrix()
    glTranslatef(p1[0], p1[1], p1[2])
    glRotated(angle, axis[0], axis[1], axis[2])
    qua = gluNewQuadric()
    gluCylinder(qua,r,r,h,10,5)

    glTranslated(0.0,0.0,h)
    gluDisk(qua,0,r,10,5)

    glTranslated(0.0,0.0,-h)
    glRotated(180,1,0,0)
    gluDisk(qua,0,r,10,5)

    glPopMatrix()
    gluDeleteQuadric(qua)
