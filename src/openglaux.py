import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL.ARB.vertex_buffer_object import *
import time
import sys				
from camera import Camera
from mathaux import *
import numpy
import traceback

# Some api in the chain is translating the keystrokes to this octal string
# so instead of saying: ESCAPE = 27, we use the following.
ESCAPE = '\033'
old_cam_up = None;
    
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



class GlWindow(object):
    """
    """
    
    def __init__(self, width, height, title):
        """
        
        Arguments:
        - `width`:
        - `height`:
        """
        self._width = width
        self._height = height
        self._fps = 0
        self._g_dwLastFPS = 0
        self._g_nFrames = 0	
        self._title = title
        self._VBOSupported = False
        self._g_dwLastFPS = 0

        self.camera=Camera()
        self._mouseButton = None
        self._oldMousePos = [ 0, 0 ]

        # # Check for VBOs Supported
        self._VBOSupported = IsExtensionSupported ("GL_ARB_vertex_buffer_object")

        # ------------------------------
        #         init window
        # ------------------------------


        glutReshapeFunc(ReSizeGLScene)

        if (self._VBOSupported):
        # # Get Pointers To The GL Functions
        # In python, calling Init for the extension functions will
        # fill in the function pointers (really function objects)
        # so that we call the Extension.        
            if (not glInitVertexBufferObjectARB()):
                raise "Help!  No GL_ARB_vertex_buffer_object"

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
        glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.03)

        glEnable(GL_LIGHT0)        
        self.bindEvents()

    def bindEvents(self):

        def keyPressedFunc(*args):
            # If escape is pressed, kill everything.
            if args[0] == ESCAPE : # exit when ESCAPE is pressed
                sys.exit ()
            return

        def mouseButtonFunc( button, mode, x, y ):
            """Callback function (mouse button pressed or released).

            The current and old mouse positions are stored in
            a	global renderParam and a global list respectively"""

            if mode == GLUT_DOWN:
                    self._mouseButton = button
            else:
                    self._mouseButton = None
            self._oldMousePos[0], self._oldMousePos[1] = x, y
            glutPostRedisplay( )
        
        def mouseMotionFunc( x, y ):            
            """Callback function (mouse moved while button is pressed).

            The current and old mouse positions are stored in
            a	global renderParam and a global list respectively.
            The global translation vector is updated according to
            the movement of the mouse pointer."""
            factor = 0.01
            global old_cam_up
            deltaX = x - self._oldMousePos[ 0 ]
            deltaY = y - self._oldMousePos[ 1 ]

            cam_ray   = self.camera.position - self.camera.lookat
            cam_distance = norm(cam_ray)
            cam_right = normalized(numpy.cross(cam_ray,self.camera.up))
            cam_up    = normalized(numpy.cross(cam_right,cam_ray))

            if old_cam_up != None:
                dot_prod = numpy.dot(cam_up,old_cam_up)
                if dot_prod < 0:
                    self.camera.up *= -1
                    cam_right = normalized(numpy.cross(cam_ray,self.camera.up))
                    cam_up    = normalized(numpy.cross(cam_right,cam_ray))
            old_cam_up = cam_up

            if ( glutGetModifiers() == GLUT_ACTIVE_SHIFT and\
                   self._mouseButton == GLUT_LEFT_BUTTON  ):
                if cam_distance > 0.1 or deltaY > 0:
                    self.camera.position += deltaY*factor*cam_ray*0.02
                                    
            elif self._mouseButton == GLUT_LEFT_BUTTON:
                dup    = deltaY*factor
                dright = deltaX*factor
                
                cam_ray += dup*cam_up + dright*cam_right
                cam_ray = normalized(cam_ray)
                self.camera.position = self.camera.lookat + cam_ray*cam_distance
                self._oldMousePos[0], self._oldMousePos[1] = x, y


            elif self._mouseButton == GLUT_RIGHT_BUTTON:
                dup    = deltaY*factor
                dright = deltaX*factor

                cam_ray -= dup*cam_up + dright*cam_right
                cam_ray = normalized(cam_ray)
                self.camera.lookat = self.camera.position - cam_ray*cam_distance
                self._oldMousePos[0], self._oldMousePos[1] = x, y

            glutPostRedisplay( )
        
        glutMouseFunc( mouseButtonFunc )
        glutMotionFunc( mouseMotionFunc )
        glutSpecialFunc(keyPressedFunc)
        glutKeyboardFunc(keyPressedFunc)

        
    def updateFPS(self):
        """       
        Arguments:
        - `self`:
        """
        milliseconds = time.clock () * 1000.0
        if (milliseconds - self._g_dwLastFPS >= 1000):
            # # When A Second Has Passed...
            # g_dwLastFPS = win32api.GetTickCount();
            # # Update Our Time Variable
            self._g_dwLastFPS = time.clock () * 1000.0
            self._fps = self._g_nFrames;
            # # Save The FPS
            self._g_nFrames = 0;	
            # # Reset The FPS Counter

        szTitle = "%s %s fps"%(self._title,self._fps)

        if ( self._VBOSupported ):	
            # # Include A Notice About VBOs
            szTitle = szTitle;
        else:
            szTitle = szTitle;

        glutSetWindowTitle ( szTitle );
