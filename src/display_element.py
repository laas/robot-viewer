import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL.ARB.vertex_buffer_object import *
import numpy, time
import kinematic_chain
from mathaux import *
from safeeval import safe_eval
from kinematic_chain import Robot, GenericObject
import traceback

import logging, os, sys
logger = logging.getLogger("robotviewer.display_element")

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger.addHandler(NullHandler())


glList_joint_sphere_mat = {}
glList_link_mat = {}

def ifenabled(meth):
    def new_meth(cls, *kargs, **kwargs):
        if cls.enabled:
            return meth(cls, *kargs, **kwargs)
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
            parent.addChild(self)
        # global obj_primitives
        if mesh:
            setattr(mesh, "gl_primitive", self)
            mesh.addChild(self)
        if gl_list_ids:
            self.gl_list_ids = gl_list_ids
        if vbos:
            self.vbos = vbos
        self.init()

    def set_transparency(self, transparency):
        pass

    def generate_gl_list(self):
        if self.mesh:
            self.generate_gl_list_mesh()
        if self.script:
            self.generate_gl_list_script()

    def generate_gl_list_script(self):
        win = glutGetWindow()
        self.gl_list_ids[win] = glGenLists(1)
        glNewList(self.gl_list_ids[win], GL_COMPILE);
        safe_eval(self.script, globals())
        glEndList();


    def generate_gl_list_mesh(self):
        win = glutGetWindow()
        if not self.gl_list_ids.get(win):
            self.gl_list_ids[win] = glGenLists(1)
        else:
            glDeleteLists(self.gl_list_ids[win], 1)
        app = self.mesh.app
        glNewList(self.gl_list_ids[win], GL_COMPILE);
        if not app.transparency:
            app.transparency = 0
        elif type(app.transparency) == list:
            app.transparency = app.transparency[0]

        for (key, value) in [ (GL_SPECULAR,app.specularColor),
                              (GL_EMISSION,app.emissiveColor ),
                              (GL_AMBIENT_AND_DIFFUSE,app.diffuseColor ),
                              (GL_AMBIENT,app.ambientColor ),
                              (GL_SHININESS,app.shininess),
                              #(GL_TRANSPARENCY,app.transparency)
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
                if self.mesh.getParentJoint():
                    joint_name = self.mesh.getParentJoint().name
                logger.debug("Self.Mesh %s of joint %s: Missing %s in material"
                               %(self.mesh.name, joint_name, key.name))
        glEndList();

    @ifenabled
    def render(self):
        win = glutGetWindow()
        if not self.gl_list_ids.get(win):
            self.generate_gl_list()

        glPushMatrix()

        Tmatrix = self.globalTransformation
        R=Tmatrix[0:3,0:3]
        p=Tmatrix[0:3,3]
        agax=rot2AngleAxis(R)
        glTranslatef(p[0],p[1],p[2])
        glRotated(agax[0],agax[1],agax[2],agax[3])
        glCallList(self.gl_list_ids[win])

        if not self.mesh:
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
        for mesh in self.mesh_list:
            mesh.addChild( GlPrimitive (mesh = mesh) )

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
            mesh.transparency = transparency
            mesh.gl_primitive.generate_gl_list(mesh)


    def getConfig(self):
        return self.config


    def update_config(self, config):
        self.pending_update = True
        self.config = config


class DisplayRobot(DisplayObject):
    @ifenabled
    def render(self, mesh_flag, skeleton_flag, skeleton_size):
        if mesh_flag:
            DisplayObject.render(self)
        if skeleton_flag or not self.mesh_list[:]:
            render_skeleton(self, skeleton_size)


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

        self.verts = []
        self.norms = []
        self.tri_idxs  = []
        self.quad_idxs = []
        self.poly_idxs = []
        self.tri_count  = 0
        self.quad_count = 0

        logger.debug("Computing normals")
        self.computeNormals(mesh)

        logger.debug("Loading to GPUs")
        self.loadGPU(mesh)

    def __del__(self):
        for vboid in [self.tri_idx_vboId, self.quad_idx_vboId] + self.poly_idx_vboIds:
            glDeleteBuffersARB(1, vboid)
        object.__del__(self)


    def computeNormals(self, mesh):
        # TODO glList for colors
        # copy vertex and normals from mesh
        self.verts = mesh.geo.coord
        coord=self.verts
        idx = mesh.geo.idx
        npoints=len(coord)/3

        if mesh.geo.norm==[]:
            normals=[]
            points=[]
            for k in range(npoints):
                normals.append(numpy.array([0.0,0.0,0.0]))
                points.append(numpy.array([coord[3*k],coord[3*k+1],
                                           coord[3*k+2]]))

        poly=[]
        ii=0
        for a_idx in idx:
            if a_idx!=-1:
                poly.append(a_idx)
                continue
            # idx=-1
            num_sides = len(poly)
            # if num_sides not in (3,4):
            #     logger.warning("""n=%d.  Only support tri and quad mesh for
            #                       the moment"""%num_sides)
            #     poly=[]
            #     continue
            # idx=-1 and poly is a triangle
            if num_sides == 3:
                self.tri_idxs += poly
            elif num_sides == 4:
                self.quad_idxs += poly
            else:
                self.poly_idxs.append(poly)

            if mesh.geo.norm == []:
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

                for i in range(num_sides):
                    j = i + 1
                    if j == num_sides:
                        j = 0
                    vecs[j][i] = normalized(points[ids[j]] - points[ids[i]])

                try:
                    for i in range(num_sides):
                        if i == 0:
                            alphas[i] = acos(numpy.dot
                                             (vecs[1][0],vecs[0][num_sides-1]))
                        elif i == num_sides - 1:
                            alphas[i] = acos(numpy.dot (
                                vecs[0][num_sides-1],
                                vecs[num_sides-1][num_sides-2]))
                        else:
                            alphas[i] = acos(numpy.dot(vecs[i][i-1],
                                                       vecs[i+1][i]))
                except Exception,error:
                    s = traceback.format_exc()
                    logger.warning("Mesh processing error: %s"%s)

                for i,alpha in enumerate(alphas):
                    if i == 0:
                        normals[ids[i]] += alpha*normalized(
                            numpy.cross(vecs[0][num_sides-1],vecs[1][0]))
                    elif i == num_sides - 1:
                        normal_i = numpy.cross(vecs[num_sides-1][num_sides-2],
                                               vecs[0][num_sides-1])
                        normals[ids[i]] += alpha*normalized(normal_i)
                    else:
                        normal_i = numpy.cross(vecs[i][i-1], vecs[i+1][i])
                        normals[ids[i]] += alpha*normalized(normal_i)
            poly=[]
        if mesh.geo.norm!=[]:
            self.norms = mesh.geo.norm
        else:
            for normal in normals:
                normal                =  normalized(normal)
                self.norms          += [normal[0],normal[1],normal[2]]
                # mesh.geo.norm  += self.norms



    def loadGPU(self, mesh):
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
                             numpy.array (self.norms, dtype=numpy.float32),
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
        s+="len (_norms)\t=%d\n"%(len(self.norms))
        s+="len (_idxs)\t=%d\n"%(len(self.tri_idxs))
        s+="]"
        return s



def render_skeleton(robot, size):
    for joint in robot.joint_list:
        draw_joint(joint, size)
        if joint.parent and joint.jointType in ["rotate","revolute",
                                                "prismatic","rotation",
                                                "translation"]:
            pos=joint.globalTransformation[0:3,3]
            parent=joint.parent
            parent_pos=parent.globalTransformation[0:3,3]
            draw_link(pos,parent_pos,size)


def draw_link(p1,p2,size=1):
    global glList_link_mat
    win = glutGetWindow()
    if not glList_link_mat.get(win):
        glList_link_mat[win] = glGenLists(1)
        glNewList(glList_link_mat[win], GL_COMPILE);
        for (key, value) in [ (GL_SPECULAR, [1,1,1,1]),
                              (GL_EMISSION, [0,1,0,1]),
                              (GL_AMBIENT_AND_DIFFUSE, [0,1,0,1]),
                              (GL_SHININESS, 5),
                              ]:
            glMaterialfv(GL_FRONT_AND_BACK, key, value)
        glEndList()

    r = 0.01*size/4
    glCallList(glList_link_mat[win])
    p = p2-p1
    n_p = normalized(p)
    h = norm(p)

    glPushMatrix()
    glTranslatef(p1[0], p1[1], p1[2])
    z_axis = numpy.array([0,0,1])
    axis = numpy.cross(z_axis, n_p)
    angle = acos(numpy.dot(z_axis, n_p))*180/pi

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


def draw_joint(joint, size = 1):
    global glList_joint_sphere_mat
    win = glutGetWindow()
    if not glList_joint_sphere_mat.get(win):
        glList_joint_sphere_mat[win] = glGenLists(1)
        glNewList(glList_joint_sphere_mat[win], GL_COMPILE);
        for (key, value) in [ (GL_SPECULAR, [1,1,1,1]),
                      (GL_EMISSION, [0.5,0,0,1]),
                      (GL_AMBIENT_AND_DIFFUSE, [0.5,0,0,1]),
                      (GL_SHININESS, 5),
                      ]:
            glMaterialfv(GL_FRONT_AND_BACK, key, value)
        glEndList()
    r = 0.01*size
    h = r/2
    pos=joint.globalTransformation[0:3,3]
    glCallList(glList_joint_sphere_mat[win])

    if joint.jointType in ["free", "freeflyer"]:
        glPushMatrix()
        glTranslatef(pos[0], pos[1], pos[2])
        angleAxis = rot2AngleAxis(joint.globalTransformation[0:3][0:3])
        glRotated(angleAxis[0],angleAxis[1],angleAxis[2],angleAxis[3])
        sphere = gluNewQuadric()
        gluSphere(sphere,0.01*size,10,10)
        glPopMatrix()
        gluDeleteQuadric(sphere)

    else:
        glPushMatrix()
        glTranslatef(pos[0], pos[1], pos[2])
        angleAxis = rot2AngleAxis(joint.globalTransformation[0:3][0:3])
        glRotated(angleAxis[0],angleAxis[1],angleAxis[2],angleAxis[3])
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
        glPopMatrix()
        gluDeleteQuadric(qua)

