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
OpenGL.FORWARD_COMPATIBLE_ONLY = True
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL.ARB.vertex_buffer_object import *
import numpy, time
import kinematics
from mathaux import *
from safeeval import safe_eval
from kinematics import Robot, GenericObject
import traceback

shaders = {}
MODERN_SHADER = False

import logging, os, sys
logger = logging.getLogger("robotviewer.display_element")

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger.addHandler(NullHandler())

SHOW_NORMALS = False
USE_VBO = False

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
                 mesh = None, script = None, parent = None):
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
        self.mesh = mesh
        if parent:
            parent.add_child(self)
            setattr(parent, "gl_primitive", self)

        if gl_list_ids:
            self.gl_list_ids = gl_list_ids
        if vbos:
            self.vbos = vbos
        self.init()

    def set_transparency(self, transparency):
        pass

    def generate_gl_list(self):
        if self.mesh:
            return self.generate_gl_list_mesh()
        if self.script:
            return self.generate_gl_list_script()

    def generate_gl_list_script(self):
        new_list = glGenLists(1)
        glNewList(new_list, GL_COMPILE);
        safe_eval(self.script, globals())
        glEndList();
        return new_list

    def generate_gl_list_mesh(self):
        new_list = glGenLists(1)
        app = self.mesh.app
        glNewList(new_list, GL_COMPILE);
        if not app.transparency:
            app.transparency = 0
        elif type(app.transparency) == list:
            app.transparency = app.transparency[0]

        if not app.specularColor:
            app.specularColor = [1, 1, 1]


        if self.mesh.parent == None:
            for key in ['emissiveColor', 'emissiveColor','ambientColor']:
                if not app.__dict__[key]:
                    app.__dict__[key] = [0.2, 0.2, 0.2]
            if not app.shininess:
                app.shininess = 5


        for (key, value) in [ (GL_SPECULAR,app.specularColor),
                              (GL_EMISSION,app.emissiveColor ),
                              (GL_AMBIENT_AND_DIFFUSE,app.diffuseColor ),
                              (GL_AMBIENT,app.ambientColor ),
                              (GL_SHININESS,app.shininess),
                              # (GL_TRANSPARENCY,app.transparency)
                              ]:
            if value:
                try:
                    if key != GL_SHININESS:
                        glMaterialfv(GL_FRONT_AND_BACK, key, value +
                                     [1-app.transparency])
                    else:
                        glMaterialfv(GL_FRONT_AND_BACK, key, value)
                except:
                    logger.exception("Failed to set material key={0}, value={1}".
                                     format(key,value))
            else:
                joint_name = "None"
                if self.mesh.get_parent_joint():
                    joint_name = self.mesh.get_parent_joint().name
                logger.debug("Self.Mesh %s of joint %s: Missing %s in material"
                               %(self.mesh.name, joint_name, key.name))
        if not USE_VBO:
            geo = self.mesh.geo
            if not geo.normals[:] or not geo.tri_idxs[:]:
                geo.compute_normals()
            glBegin(GL_TRIANGLES)
            for i in geo.tri_idxs:
                glNormal3f( geo.normals[3*i], geo.normals[3*i+1], geo.normals[3*i+2])
                glVertex3f( geo.coord[3*i], geo.coord[3*i+1], geo.coord[3*i+2])
            glEnd()

            if SHOW_NORMALS:
                glBegin(GL_LINES)
                for i in geo.tri_idxs:
                    n = [geo.normals[3*i], geo.normals[3*i+1], geo.normals[3*i+2]]
                    v = [geo.coord[3*i], geo.coord[3*i+1], geo.coord[3*i+2]]
                    glVertex3f(v[0], v[1], v[2])
                    glVertex3f(v[0] + 0.01*n[0],
                               v[1] + 0.01*n[1],
                               v[2] + 0.01*n[2],
                               )
                glEnd()

        glEndList();
        return new_list

    @ifenabled
    def render(self):
        win = glutGetWindow()
        if not self.gl_list_ids.get(win):
            self.gl_list_ids[win] = self.generate_gl_list()

        glPushMatrix()

        Tmatrix = self.globalTransformation
        R=Tmatrix[0:3,0:3]
        p=Tmatrix[0:3,3]
        agax=rot2AngleAxis(R)
        glTranslatef(p[0],p[1],p[2])
        glRotated(agax[0],agax[1],agax[2],agax[3])

        if MODERN_SHADER:
            if self.mesh:
                app = self.mesh.app
                #print app.specularColor, app.emissiveColor, app.diffuseColor, app.ambientColor
                if app.specularColor:
                    shaders[glutGetWindow()].uMaterialSpecularColor = app.specularColor

                if app.emissiveColor:
                    shaders[glutGetWindow()].uMaterialEmissiveColor = app.emissiveColor

                if app.diffuseColor:
                    shaders[glutGetWindow()].uMaterialDiffuseColor = app.diffuseColor

                if app.ambientColor:
                    shaders[glutGetWindow()].uMaterialAmbientColor = app.ambientColor
            glCallList(self.gl_list_ids[win])

        else:
            glCallList(self.gl_list_ids[win])

        if (not USE_VBO) or (not self.mesh) :
            glPopMatrix()
            return
        if not self.vbos.get(win):
            self.vbos[win] = Vbo(self.mesh)

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
        for mesh in self.mesh_list:
            mesh.add_child( GlPrimitive (mesh = mesh, parent = mesh) )
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

        for m in self.mesh_list:
            if not m.gl_primitive:
                continue
            m.gl_primitive.render()


    def set_transparency(self, transparency):
        for mesh in self.mesh_list:
            mesh.app.transparency = transparency
            mesh.gl_primitive.mesh.app.transparency = transparency
            # mesh.gl_primitive.generate_gl_list()


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
    def render(self, mesh_flag = True,  skeleton_flag = False):
        if self.pending_update:
            self.pending_update = False
            self.obj.update_config(self.config)

        if mesh_flag:
            DisplayObject.render(self)

        if skeleton_flag or not self.mesh_list[:]:
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

    def __init__(self, mesh):
        """

        Arguments:
        - `mesh`:
        """
        self.ver_vboId  = -1
        self.nor_vboId  = -1
        self.tri_idx_vboId  = -1
        self.quad_idx_vboId  = -1
        self.poly_idx_vboIds  = []

        self.verts = mesh.geo.coord
        if not mesh.geo.normals[:] or not mesh.geo.tri_idxs[:]:
            mesh.geo.compute_normals()

        self.normals = mesh.geo.normals
        self.tri_idxs  = mesh.geo.tri_idxs
        self.quad_idxs = mesh.geo.quad_idxs
        self.poly_idxs = mesh.geo.poly_idxs
        self.tri_count  = mesh.geo.tri_count
        self.quad_count = mesh.geo.tri_count

        logger.debug("Computing normals")

        logger.debug("Loading to GPUs")
        self.load_gpu(mesh)

    def __del__(self):
        for vboid in [self.tri_idx_vboId, self.quad_idx_vboId] + self.poly_idx_vboIds:
            glDeleteBuffersARB(1, vboid)
        object.__del__(self)


    def load_gpu(self, mesh):
        logger.debug("Creating VBO for mesh %s"%mesh.name)
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
                             numpy.array (self.normals, dtype=numpy.float32),
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
        logger.debug("Finished creating VBO for mesh %s"%mesh.name)

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

        logger.debug("Finished creating VBO for mesh %s"%mesh.name)


    def __str__(self):
        """
        """
        s="[Vbo instance:\n"
        s+="ver_vboId\t=%d\n"%self.ver_vboId
        s+="nor_vboId\t=%d\n"%self.nor_vboId
        s+="tri_idx_vboId\t=%d\n"%self.tri_idx_vboId
        s+="quad_idx_vboId\t=%d\n"%self.quad_idx_vboId

        s+="len (_verts)\t=%d\n"%(len(self.verts))
        s+="len (_norms)\t=%d\n"%(len(self.normals))
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

    if joint.axis in ("X","x"):
        glRotated(90,0,1,0)
    elif joint.axis in ("Y","y"):
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
    n_p = normalized(p)
    h = norm(p)
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
