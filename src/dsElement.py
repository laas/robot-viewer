import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL.ARB.vertex_buffer_object import *
import numpy, time
import robo,robotLoader
from mathaux import *
from safeeval import safe_eval
class DsElement(object):
    """
    """

    def __init__(self, xyz = [0,0,0], rpy = [0,0,0], enabled = False):
        """

        Arguments:
        - `xyz`: Position in space
        - `rpy`: rpy in space
        """
        self._xyz = xyz
        self._rpy = rpy
        self._enabled = enabled

    def __str__(self):
        return "FIX-ME"

    def render(self):
        """Render element and its children in the scene

        Arguments:
        - `self`:
        """
        raise NotImplementedError, "Implement me"


    def updateConfig(self,conf):
        """Update element configuration

        Arguments:
        - `self`:
        - `xyz`:
        - `rpy`:
        """
        self._xyz=conf[0:3]
        self._rpy=conf[3:6]

    def getConfig(self):
        return self._xyz + self._rpy


    def enable(self):
        """
        Arguments:
        - `self`:
        """
        self._enabled = True

    def disable(self):
        """

        Arguments:
        - `self`:
        """
        self._enabled = False


    def isEnabled(self):
        return self._enabled


class DsRobot(DsElement):
    """Display Element for a humanoid robot
    """
    def __init__(self, robot = None, xyz = [0,0,0], rpy = [0,0,0], enabled= False):
        """

        Arguments:
        - `xyz`: Position in space
        - `rpy`: rpy in space
        """
        self._xyz = xyz
        self._rpy = rpy
        self._enabled = enabled
        self._q = []
        self._robot = robot
        self._shapeVBOlist=[]
        self._kinematics_update_t = 0
        self._config_update_t = 0
        for amesh in robot.mesh_list:
            for ashape in amesh.shapes:
                shapeVBO=ShapeVBO(ashape)
                shapeVBO._mesh=amesh
                self._shapeVBOlist.append(shapeVBO)


    def __str__(self):
        s = "  type\t: Robot\n"
        s += "  config\t: %s\n"%str(self._robot.getConfig())
        return s

    def updateConfig(self,conf):
        """Update element configuration

        Arguments:
        - `self`:
        - `xyz`:
        - `rpy`:
        """
        self._config_update_t = time.time()
        self._xyz = conf[0:3]
        self._rpy = conf[3:6]
        self._q   = conf[6:]

    def getConfig(self):
        return self._xyz + self._rpy + self._q

    def render(self):
        """

        Arguments:
        - `self`:
        """
        if not self._enabled:
            return

        if self._kinematics_update_t < self._config_update_t:
            self._kinematics_update_t = time.time()
            self._robot.waistPos(self._xyz)
            self._robot.waistRpy(self._rpy)
            self._robot.setAngles(self._q)
            self._robot.update()
            # print "Waist: \n", self._robot.waist.globalTransformation, "\n\n"
            # for i in range(6):
                # print "R(%d): \n"%i, self._robot.joint_dict[i], "\n"
                # print "J(%d): \n"%i, self._robot.joint_dict[i].globalTransformation, "\n"
                # print "L(%d): \n"%i, self._robot.joint_dict[i+6], "\n"
                # print "L(%d): \n"%i, self._robot.joint_dict[i+6].globalTransformation, "\n\n"
                # print "%d %f"%(i,self._robot.joint_dict[i].angle)
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_VERTEX_ARRAY);
        for avbo in self._shapeVBOlist:
            amesh = avbo._mesh
            Tmatrix=amesh.globalTransformation
            R=Tmatrix[0:3,0:3]
            p=Tmatrix[0:3,3]
            agax=rot2AngleAxis(R)

            glPushMatrix()
            glTranslatef(p[0],p[1],p[2])
            glRotated(agax[0],agax[1],agax[2],agax[3])
            glCallList(avbo.glList_idx)

            # print avbo.ver_vboId,avbo.nor_vboId,avbo.idx_vboId
            # before draw, specify vertex and index arrays with their offsets

            # Use VBO
            glBindBufferARB(GL_ARRAY_BUFFER_ARB, avbo.ver_vboId);
            glVertexPointer( 3, GL_FLOAT, 0, None );

            glBindBufferARB(GL_ARRAY_BUFFER_ARB, avbo.nor_vboId);
            glNormalPointer(GL_FLOAT, 0,None);

            glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, avbo.idx_vboId);

            glDrawElements(GL_TRIANGLES, avbo.count, GL_UNSIGNED_SHORT, None);

            glPopMatrix()
            glFlush()
        # end drawing the bot
        glDisableClientState(GL_VERTEX_ARRAY);  # disable vertex arrays
        glDisableClientState(GL_NORMAL_ARRAY);


class DsScript(DsElement):
    """
    """
    def __init__(self, script = "" , xyz = [0,0,0], rpy = [0,0,0], enabled = False):
        """
        Arguments:
        - `xyz`: Position in space
        - `rpy`: rpy in space
        """
        self._xyz = xyz
        self._rpy = rpy
        self._enabled = enabled
        self._script = script
        self._glList_idx = -1
        self._glList_idx = glGenLists(1)
        glNewList(self._glList_idx, GL_COMPILE);
        safe_eval(self._script,globals())
        glEndList();

    def __str__(self):
        s = "  type\t: script\n"
        s += "  config\t: %s\n"%str(self._xyz+self._rpy)
        return s

    def render(self):
        """

        Arguments:
        - `self`:
        """
        if not self._enabled:
            return
        glPushMatrix()
        glTranslatef(self._xyz[0],self._xyz[1],self._xyz[2])
        agax = euleur2AngleAxis(self._rpy)
        glRotated(agax[0],agax[1],agax[2],agax[3])
        glCallList(self._glList_idx)
        glPopMatrix()


class ShapeVBO(object):
    """
    """

    def __init__(self, shape):
        """

        Arguments:
        - `mesh`:
        """
        self._shape  = shape
        self._mesh = None

        self.ver_vboId  = -1
        self.nor_vboId  = -1
        self.idx_vboId  = -1
        self.glList_idx = -1
        self._verts = []
        self._norms = []
        self._idxs  = []
        self.count  = 0

        # TODO glList for colors
        # copy vertex and normals from shape
        self._verts = self._shape.geo.coord
        coord=self._verts
        idx = self._shape.geo.idx
        npoints=len(coord)/3

        if self._shape.geo.norm==[]:
            normals=[]
            points=[]
            for k in range(npoints):
                normals.append(numpy.array([0.0,0.0,0.0]))
                points.append(numpy.array([coord[3*k],coord[3*k+1],coord[3*k+2]]))

        poly=[]
        ii=0
        for a_idx in idx:
            if a_idx!=-1:
                poly.append(a_idx)
                continue

            # idx=-1
            if len(poly)!=3:
                warnings.warn("""oops not a triangle, n=%d.
                                  Only support triangle mesh for the moment"""%len(poly))
                poly=[]
                continue
            # idx=-1 and poly is a triangle
            self._idxs += poly

            if self._shape.geo.norm==[]:
                # update the norm vector
                # update the normals using G. Thurmer, C. A. Wuthrich,
                # "Computing vertex normals from polygonal facets"
                # Journal of Graphics Tools, 3 1998
                id0=poly[0];id1=poly[1];id2=poly[2]
                p10=normalized(points[id1]-points[id0])
                p21=normalized(points[id2]-points[id1])
                p02=normalized(points[id0]-points[id2])
                alpha0=alpha1=alpha2=0
                try:
                    alpha0=acos(numpy.dot(p10,p02))
                    alpha1=acos(numpy.dot(p21,p10))
                    alpha2=acos(numpy.dot(p02,p21))
                except Exception,error:
                    warnings.warn("Mesh processing error: %s"%error)
                normals[id0]+=alpha0*normalized(numpy.cross(p02,p10))
                normals[id1]+=alpha1*normalized(numpy.cross(p10,p21))
                normals[id2]+=alpha2*normalized(numpy.cross(p21,p02))
            poly=[]

        if self._shape.geo.norm!=[]:
            self._norms=self._shape.geo.norm
        else:
            for normal in normals:
                normal                =  normalized(normal)
                self._norms          += [normal[0],normal[1],normal[2]]
                self._shape.geo.norm += [normal[0],normal[1],normal[2]]

        self.count = len(self._idxs)
        self.ver_vboId = int(glGenBuffersARB(1))
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,self.ver_vboId );
        glBufferDataARB( GL_ARRAY_BUFFER_ARB, \
                             numpy.array (self._verts, dtype=numpy.float32),\
                             GL_STATIC_DRAW_ARB );
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,0 );

        self.nor_vboId = int(glGenBuffersARB(1))
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,self.nor_vboId );
        glBufferDataARB( GL_ARRAY_BUFFER_ARB, \
                             numpy.array (self._norms, dtype=numpy.float32),\
                             GL_STATIC_DRAW_ARB );
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,0 );

        self.idx_vboId = int(glGenBuffersARB(1))
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,self.idx_vboId );
        glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB, \
                             numpy.array (self._idxs, dtype=numpy.uint16),\
                             GL_STATIC_DRAW_ARB );
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,0 );

        self.glList_idx = glGenLists(1)
        app=self._shape.app
        glNewList(self.glList_idx, GL_COMPILE);
        if app.specularColor:
            glMaterialfv(GL_FRONT, GL_SPECULAR,app.specularColor)
        if app.emissiveColor:
            glMaterialfv(GL_FRONT, GL_EMISSION,app.emissiveColor )
        if app.diffuseColor:
            glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,app.diffuseColor )
        if app.shininess:
            glMaterialfv(GL_FRONT, GL_SHININESS,app.shininess)

        glEndList();


    def __str__(self):
        """
        """
        s="[ShapeVBO instance:\n"
        s+="ver_vboId\t=%d\n"%self.ver_vboId
        s+="nor_vboId\t=%d\n"%self.nor_vboId
        s+="idx_vboId\t=%d\n"%self.idx_vboId

        s+="len (_verts)\t=%d\n"%(len(self._verts))
        s+="len (_norms)\t=%d\n"%(len(self._norms))
        s+="len (_idxs)\t=%d\n"%(len(self._idxs))
        s+="]"
        return s
