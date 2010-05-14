#! /usr/bin/env python
# inspired from nehe45

import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy
from OpenGL.GL.ARB.vertex_buffer_object import *
import time				
import sys
import robo,robotLoader
from nutshell import *
from camera import Camera,norm,normalized

ver_vboId=-1
nor_vboId=-1
col_vboId=-1
camera=Camera()
# *********************** Globals *********************** 

# Some api in the chain is translating the keystrokes to this octal string
# so instead of saying: ESCAPE = 27, we use the following.
ESCAPE = '\033'

# Number of the glut window.
window = 0

# PyOpenGL doesn't yet have the ARB for vertex_buffer_objects
NO_VBOS = True

g_fVBOSupported = False;
# // ARB_vertex_buffer_object supported?
g_nFPS = 0
g_nFrames = 0;
# // FPS and FPS Counter
g_dwLastFPS = 0;
# // Last FPS Check Time	

g_prev_draw_time = 0.0
# The main drawing function. 
robot=None
shapeVBOlist=[]

def IsExtensionSupported (TargetExtension):
    """ Accesses the rendering context to see if it supports an extension.
    Note, that this test only tells you if the OpenGL library
    supports the extension. The PyOpenGL system might not actually
    support the extension.
    """
    Extensions = glGetString (GL_EXTENSIONS)
    # python 2.3
    # if (not TargetExtension in Extensions):
    #	gl_supports_extension = False
    #	print "OpenGL does not support '%s'" % (TargetExtension)
    #	return False
    
    # python 2.2
    Extensions = Extensions.split ()
    found_extension = False
    for extension in Extensions:
        if extension == TargetExtension:
            found_extension = True
            break;
    if (found_extension == False):
        gl_supports_extension = False
        print "OpenGL rendering context does not support '%s'" % (TargetExtension)
        return False

    gl_supports_extension = True

    # Now determine if Python supports the extension
    # Exentsion names are in the form GL_<group>_<extension_name>
    # e.g.  GL_EXT_fog_coord 
    # Python divides extension into modules
    # g_fVBOSupported = IsExtensionSupported ("GL_ARB_vertex_buffer_object")
    # from OpenGL.GL.EXT.fog_coord import *
    if (TargetExtension [:3] != "GL_"):
        # Doesn't appear to following extension naming convention.
        # Don't have a means to find a module for this exentsion type.
        return False

    # extension name after GL_
    afterGL = TargetExtension [3:]
    try:
        group_name_end = afterGL.index ("_")
    except:
        # Doesn't appear to following extension naming convention.
        # Don't have a means to find a module for this exentsion type.
        return False

    group_name = afterGL [:group_name_end]
    extension_name = afterGL [len (group_name) + 1:]
    extension_module_name = "OpenGL.GL.ARB.%s" % (extension_name)

    import traceback
    try:
        __import__ (extension_module_name)
        print "PyOpenGL supports '%s'" % (TargetExtension)
    except:
        traceback.print_exc()
        print 'Failed to import', extension_module_name
        print "OpenGL rendering context supports '%s'" % (TargetExtension),
        return False

    return True


def DrawGLScene():
    global g_dwLastFPS, g_nFPS, g_nFrames, g_pMesh,\
        g_fVBOSupported, g_flYRot, g_prev_draw_time
    global ver_vboId,nor_vboId,col_vboId

    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
    # Clear Screen And Depth Buffer
    glLoadIdentity ();
    # # Reset The Modelview Matrix        
    # # Get FPS
    # milliseconds = win32api.GetTickCount() 
    milliseconds = time.clock () * 1000.0
    if (milliseconds - g_dwLastFPS >= 1000):
        # # When A Second Has Passed...
        # g_dwLastFPS = win32api.GetTickCount();
        # # Update Our Time Variable
        g_dwLastFPS = time.clock () * 1000.0
        g_nFPS = g_nFrames;
        # # Save The FPS
        g_nFrames = 0;	
        # # Reset The FPS Counter
        
        # # Build The Title String
        szTitle = "OpenGL Backend - %d FPS"%g_nFPS 
        if ( g_fVBOSupported ):	
            # # Include A Notice About VBOs
            szTitle = szTitle + ", Using VBOs";
        else:
            szTitle = szTitle + ", Not Using VBOs";

        glutSetWindowTitle ( szTitle );	
    
    g_nFrames += 1 
    # # Increment Our FPS Counter
    rot = (milliseconds - g_prev_draw_time) / 1000.0 * 25
    g_prev_draw_time = milliseconds
    robot.update()
    p=camera.position
    f=camera.lookat
    u=camera.up
    gluLookAt(p[0],p[1],p[2],f[0],f[1],f[2],u[0],u[1],u[2])

    # start drawing the bot


    for avbo in shapeVBOlist:
        amesh = avbo._mesh
        Tmatrix=amesh.globalTransformation
        R=Tmatrix[0:3,0:3]
        p=Tmatrix[0:3,3]
        agax=rot2AngleAxis(R)        

        glPushMatrix()
        glTranslatef(p[0],p[1],p[2])
        glRotated(agax[0],agax[1],agax[2],agax[3])
        glCallList(avbo.glList_idx)
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_INDEX_ARRAY);
        print avbo.ver_vboId,avbo.nor_vboId,avbo.idx_vboId
        # before draw, specify vertex and index arrays with their offsets
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, avbo.ver_vboId);
        glVertexPointer( 3, GL_FLOAT, 0, None );

        glBindBufferARB(GL_ARRAY_BUFFER_ARB, avbo.nor_vboId);
        glNormalPointer(GL_FLOAT, 0,None);

        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, avbo.idx_vboId);
        glColorPointer(3,GL_FLOAT, 0, None);
        
        glDrawElements(GL_TRIANGLES, avbo.count, GL_UNSIGNED_SHORT, None);
        glDisableClientState(GL_VERTEX_ARRAY);  # disable vertex arrays
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_INDEX_ARRAY);
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
        glPopMatrix()

    # end drawing the bot

    #         # # Flush The GL Rendering Pipeline
    glutSwapBuffers()

    return True



# The function called whenever a key is pressed. Note the use of Python tuples
# to pass in: (key, x, y)
def keyPressed(*args):
    global window

	# If escape is pressed, kill everything.
    if args[0] == ESCAPE:
        sys.exit ()
    return



# # Any GL Init Code & User Initialiazation Goes Here
def InitGL(Width, Height):
    global ver_vboId,nor_vboId,col_vboId
    # We call this right after our OpenGL window is created.
    global g_pMesh,g_fVBOSupported,vertices,normals,colors

    # # Check for VBOs Supported
    g_fVBOSupported = IsExtensionSupported ("GL_ARB_vertex_buffer_object")
    if (g_fVBOSupported):
        # # Get Pointers To The GL Functions
        # In python, calling Init for the extension functions will
        # fill in the function pointers (really function objects)
        # so that we call the Extension.
        
        if (not glInitVertexBufferObjectARB()):
            print "Help!  No GL_ARB_vertex_buffer_object"
            sys.exit(1)
            return False
    else:
        print "Need VBO, sorry!"

    # Setup GL States
    glClearColor (0.0, 0.0, 0.0, 0.5);
        # # Black Background
    glClearDepth (1.0);	
        # # Depth Buffer Setup
    glDepthFunc (GL_LEQUAL);
        # # The Type Of Depth Testing
    glEnable (GL_DEPTH_TEST);
    # # Enable Depth Testing
    glShadeModel (GL_SMOOTH);
    # # Select Smooth Shading
    glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	
        # # Set Perspective Calculations To Most Accurate
    glEnable(GL_TEXTURE_2D);	
        # # Enable Texture Mapping
    glColor4f (1.0, 6.0, 6.0, 1.0)
    
    glClearColor(0.,0.,0.,1.)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    
    lightZeroPosition = [-3.0,3.0,3.0,1.0]
    lightZeroColor = [1.0,1.0,1.0,1.0] #green tinged
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)

    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.0)
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.0)
    glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.08)

    glEnable(GL_LIGHT0)
        
    return True;		

# The function called when our window is resized (which shouldn't happen if you
# enable fullscreen, below)
def ReSizeGLScene(Width, Height):
    if Height == 0:		
        # Prevent A Divide By Zero If The Window Is Too Small 
        Height = 1

    glViewport(0, 0, Width, Height)
    
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    # # field of view, aspect ratio, near and far
    # This will squash and stretch our objects as the window is resized.
    gluPerspective(45.0, float(Width)/float(Height), 1, 1000.0)
    
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()



# *********************** Mesh wrapper *******************

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
                normals.append(np.array([0.0,0.0,0.0]))
                points.append(np.array([coord[3*k],coord[3*k+1],coord[3*k+2]]))

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
                    alpha0=acos(np.dot(p10,p02))
                    alpha1=acos(np.dot(p21,p10))
                    alpha2=acos(np.dot(p02,p21))
                except Exception,error:
                    warnings.warn("Mesh processing error: %s"%error)
                normals[id0]+=alpha0*normalized(np.cross(p02,p10))
                normals[id1]+=alpha1*normalized(np.cross(p10,p21))
                normals[id2]+=alpha2*normalized(np.cross(p21,p02))
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

        self.nor_vboId = int(glGenBuffersARB(1))
        glBindBufferARB( GL_ARRAY_BUFFER_ARB,self.nor_vboId );
        glBufferDataARB( GL_ARRAY_BUFFER_ARB, \
                             numpy.array (self._norms, dtype=numpy.float32),\
                             GL_STATIC_DRAW_ARB );
        
        self.idx_vboId = int(glGenBuffersARB(1))
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB,self.idx_vboId );
        glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB, \
                             numpy.array (self._idxs, dtype=numpy.uint16),\
                             GL_STATIC_DRAW_ARB );

        glBindBufferARB( GL_ARRAY_BUFFER_ARB,0 ); 
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
                            
def main():
    """Main function
    """
    global robot
    global window
    global shapeVBOlist

    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
    glutInitWindowSize(640, 480)
    glutInitWindowPosition(0, 0)
    window = glutCreateWindow("Robot-viewer Backend")
    glutDisplayFunc(DrawGLScene)
    glutIdleFunc(DrawGLScene)
    glutReshapeFunc(ReSizeGLScene)
    glutKeyboardFunc(keyPressed)
    glutSpecialFunc(keyPressed)
    InitGL(640, 480)

    robot=robotLoader.robotLoader(os.environ['HOME']+'/licenses/'+\
                                      'HRP2JRL/model/HRP2JRLmain.wrl',True)    
    i=1
    for mesh in robot.mesh_list:
        for shape in mesh.shapes:
            shapeVBO=ShapeVBO(shape)
            shapeVBO._mesh=mesh
            shapeVBOlist.append(shapeVBO)

    
    glutMainLoop()


if __name__ == '__main__':
    main()

