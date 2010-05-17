#! /usr/bin/env python
# prototype with pyopengl
import os,warnings
import imp
import re
from math import acos,pi
import robotLoader
import numpy as np

import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import Numeric

import Image
# PIL
import sys
# import win32api				
# GetTickCount
import time				

# Note Yet Supported
from OpenGL.GL.ARB.vertex_buffer_object import *
# http://oss.sgi.com/projects/ogl-sample/registry/ARB/vertex_buffer_object.txt



from camera import *

class Application(object):
     """Main class of robot-viewer, contains OpenGL window, robot's kinematics
     structures, GUI, corbaserver server
     """
     def __init__(self):
         self.robot=None
         self.state=None


         ## redering stuff
         self.RobotKinematicsFile=None
         self.numMeshes=0
         self.meshes=[]         

     def loadRobot(self,RobotKinematicsFile=None):
         self.state="LOADING"
         import pickle
         if RobotKinematicsFile:
             self.robot=robotLoader.robotLoader(RobotKinematicsFile,True)
         else:
             if not os.path.exists('/tmp/lastRobot.pickle'):
                 warnings.warn("""No robot found. If this is your first time, 
                                  use -w RobotKinematics_File.wrl to load a robot.""")

                 tmp=imp.find_module('robotviewer')
                 path=tmp[1]
                 path=re.sub(r"/robotviewer$","/",path)
                 if os.path.exists(path+'/data/nancy.wrl'):
                      print ("Loading default model (Nancy) in %s"\
                                  %(path+'/data/nancy.wrl') )
                      self.robot=robotLoader.robotLoader(\
                                                        path+'/data/nancy.wrl',True)
                 
                 else:
                      self.state="STOP"
                      return
             else:
                  print "loading the last loaded robot \n"
                  self.robot=pickle.load(open('/tmp/lastRobot.pickle','r'))
             print "loaded ",self.robot.name


         self.state="STOP"
         self.meshes=self.robot.mesh_list 
         self.numMeshes=len(self.meshes)
         for i in range(self.numMeshes):
             mesh=self.meshes[i]
             shapes=mesh.shapes
             for j in range(len(shapes)):
                 ashape=shapes[j]
                 coord=ashape.geo.coord
                 npoints=len(coord)/3
                 tri_list=[]
                 idx=ashape.geo.idx
 #                print "idx=",idx
                 ii=0
                 polyline=[]

                 if ashape.geo.norm==[]:
                     normals=[]                
                     points=[]
                     for k in range(npoints):
                         normals.append(np.array([0.0,0.0,0.0]))
                         points.append(np.array([coord[3*k],coord[3*k+1],coord[3*k+2]]))

                 while ii < len(idx):
                     if idx[ii]!=-1:
                         polyline.append(idx[ii])

                     else:
                         if len(polyline)!=3:
                             warnings.warn("oops not a triangle, n=%d. The program only support triangle mesh for the moment"\
                                               %len(polyline))
                         else:
                             if ashape.geo.norm==[]:                                
                                 # update the normals using G. Thurmer,
                                 # C. A. Wuthrich, "Computing vertex normals
                                 # from polygonal facets" Journal of Graphics
                                 # Tools, 3 1998
                                 id0=polyline[0];id1=polyline[1];id2=polyline[2]
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
                             tri_list+=polyline
                         polyline=[]
                     ii+=1
                 if ashape.geo.norm==[]:
                     norm_array=[]
                     for normal in normals:
                         normal=normalized(normal)
                         norm_array+=[normal[0],normal[1],normal[2]]
                     ashape.geo.norm=norm_array


         f=open('/tmp/lastRobot.pickle','w')
         pickle.dump(self.robot,f)
         f.close()
# The function called when our window is resized (which shouldn't happen if you
# enable fullscreen, below)
def ReSizeGLScene(Width, Height):
	if Height == 0:
                # Prevent A Divide By Zero If The Window Is Too Small 
		Height = 1

	glViewport(0, 0, Width, Height)
        # Reset The Current Viewport And Perspective Transformation
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()
	# // field of view, aspect ratio, near and far
	# This will squash and stretch our objects as the window is resized.
	gluPerspective(45.0, float(Width)/float(Height), 1, 1000.0)

	glMatrixMode(GL_MODELVIEW)
	glLoadIdentity()


# The function called whenever a key is pressed. Note the use of Python tuples
# to pass in: (key, x, y)
def keyPressed(*args):
	global window

	# If escape is pressed, kill everything.
	if args[0] == ESCAPE:
		sys.exit ()
	return

def DrawGLScene():
	global g_dwLastFPS, g_nFPS, g_nFrames, g_pMesh, g_fVBOSupported, g_flYRot, g_prev_draw_time

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        # // Clear Screen And Depth Buffer
	glLoadIdentity ();
        # // Reset The Modelview Matrix

	# // Get FPS
	# milliseconds = win32api.GetTickCount() 
	milliseconds = time.clock () * 1000.0
	if (milliseconds - g_dwLastFPS >= 1000):
                # // When A Second Has Passed...
		# g_dwLastFPS = win32api.GetTickCount();
                # // Update Our Time Variable
		g_dwLastFPS = time.clock () * 1000.0
		g_nFPS = g_nFrames;										
                # // Save The FPS
		g_nFrames = 0;
                # // Reset The FPS Counter

		# // Build The Title String
		szTitle = "Lesson 45: NeHe & Paul Frazee's VBO Tut - %d Triangles, %d FPS"\
                    % (g_pMesh.m_nVertexCount / 3, g_nFPS );
		if ( g_fVBOSupported ):
                        # // Include A Notice About VBOs
			szTitle = szTitle + ", Using VBOs";
		else:
			szTitle = szTitle + ", Not Using VBOs";
                        
		glutSetWindowTitle ( szTitle );							
                # // Set The Title

	g_nFrames += 1
        # // Increment Our FPS Counter
	rot = (milliseconds - g_prev_draw_time) / 1000.0 * 25
	g_prev_draw_time = milliseconds
	g_flYRot += rot 		
        # // Consistantly Rotate The Scenery

	# // Move The Camera
	glTranslatef( 0.0, -220.0, 0.0 );							
        # // Move Above The Terrain
	glRotatef( 10.0, 1.0, 0.0, 0.0 );	
        # // Look Down Slightly
	glRotatef( g_flYRot, 0.0, 1.0, 0.0 );
        # // Rotate The Camera
	# // Enable Pointers
	glEnableClientState( GL_VERTEX_ARRAY );
        # // Enable Vertex Arrays
	glEnableClientState( GL_TEXTURE_COORD_ARRAY );
        # // Enable Texture Coord Arrays


	# // Set Pointers To Our Data
	if( g_fVBOSupported ):
		glBindBufferARB( GL_ARRAY_BUFFER_ARB, g_pMesh.m_nVBOVertices );
		glVertexPointer( 3, GL_FLOAT, 0, None );				
                # // Set The Vertex Pointer To The Vertex Buffer
		glBindBufferARB( GL_ARRAY_BUFFER_ARB, g_pMesh.m_nVBOTexCoords );
		glTexCoordPointer( 2, GL_FLOAT, 0, None );
                # // Set The TexCoord Pointer To The TexCoord Buffer
	else:
		# You can use the pythonism glVertexPointerf (), which will
		# convert the numarray into the needed memory for
		# VertexPointer. This has two drawbacks however: 1) This does
		# not work in Python 2.2 with PyOpenGL 2.0.0.44 2) In Python
		# 2.3 with PyOpenGL 2.0.1.07 this is very slow.  See the
		# PyOpenGL documentation. Section "PyOpenGL for OpenGL
		# Programmers" for details regarding glXPointer API.  Also see
		# OpenGLContext Working with Numeric Python glVertexPointerf (
		# g_pMesh.m_pVertices ); # // Set The Vertex Pointer To Our
		# Vertex Data glTexCoordPointerf ( g_pMesh.m_pTexCoords ); # //
		# Set The Vertex Pointer To Our TexCoord Data
		#
		#
		# The faster approach is to make use of an opaque "string" that
		# represents the the data (vertex array and tex coordinates in
		# this case).
		glVertexPointer( 3, GL_FLOAT, 0, g_pMesh.m_pVertices_as_string);  	
                # // Set The Vertex Pointer To Our Vertex Data
		glTexCoordPointer( 2, GL_FLOAT, 0, g_pMesh.m_pTexCoords_as_string); 	
                # // Set The Vertex Pointer To Our TexCoord Data


	# // Render
	glDrawArrays( GL_TRIANGLES, 0, g_pMesh.m_nVertexCount );
        # // Draw All Of The Triangles At Once
	# // Disable Pointers
	glDisableClientState( GL_VERTEX_ARRAY );
        # // Disable Vertex Arrays
	glDisableClientState( GL_TEXTURE_COORD_ARRAY );
        # // Disable Texture Coord Arrays
	glutSwapBuffers()												
	 # // Flush The GL Rendering Pipeline
	return True
# // Any GL Init Code & User Initialiazation Goes Here
def InitGL(Width, Height):
        # We call this right after our OpenGL window is created.
	global g_pMesh

	# // TUTORIAL
	# // Load The Mesh Data
	g_pMesh = CMesh ()
	if (not g_pMesh.LoadHeightmap ("Terrain.bmp",
		CMesh.MESH_HEIGHTSCALE, CMesh.MESH_RESOLUTION)):
		print "Error Loading Heightmap"
		sys.exit(1)
		return False

	# // Check for VBOs Supported
	g_fVBOSupported = IsExtensionSupported ("GL_ARB_vertex_buffer_object")
	if (g_fVBOSupported):
		# // Get Pointers To The GL Functions
		# In python, calling Init for the extension functions will
		# fill in the function pointers (really function objects)
		# so that we call the Extension.

		if (not glInitVertexBufferObjectARB ()):
			print "Help!  No GL_ARB_vertex_buffer_object"
			sys.exit(1)
			return False
		# Now we can call to gl*Buffer* ()
		# glGenBuffersARB
		# glBindBufferARB
		# glBufferDataARB
		# glDeleteBuffersARB
		g_pMesh.BuildVBOs 


	# Setup GL States
	glClearColor (0.0, 0.0, 0.0, 0.5);
        # // Black Background
	glClearDepth (1.0);	
        # // Depth Buffer Setup
	glDepthFunc (GL_LEQUAL);
        # // The Type Of Depth Testing
	glEnable (GL_DEPTH_TEST);
        # // Enable Depth Testing
	glShadeModel (GL_SMOOTH);									
        # // Select Smooth Shading
	glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        # // Set Perspective Calculations To Most Accurate
	glEnable(GL_TEXTURE_2D);
        # // Enable Texture Mapping
	glColor4f (1.0, 6.0, 6.0, 1.0)        
        # Return TRUE (Initialization Successful)
	return True;												




def main():
    """
    """
    app=Application()
    # app.loadRobot('/home/john/licenses/hrpmodel/HRP2JRL/model/HRP2JRLmain.wrl')
#    app.loadRobot()
#    print app.robot
    global window
    # pass arguments to init
    glutInit(sys.argv)

    # Select type of Display mode:   
    #  Double buffer 
    #  RGBA color
    # Alpha components supported 
    # Depth buffer
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
    
    # get a 640 x 480 window 
    glutInitWindowSize(640, 480)
    
    # the window starts at the upper left corner of the screen 
    glutInitWindowPosition(0, 0)
    
	# Okay, like the C version we retain the window id to use when closing,
	# but for those of you new to Python, remember this assignment would
	# make the variable local and not global if it weren't for the global
	# declaration at the start of main.
    window = glutCreateWindow("Lesson 45: NeHe & Paul Frazee's VBO Tut")

        # Register the drawing function with glut, BUT in Python land, at least
	# using PyOpenGL, we need to set the function pointer and invoke a
	# function to actually register the callback, otherwise it would be
	# very much like the C version of the code.
    glutDisplayFunc(DrawGLScene)
	
	# Uncomment this line to get full screen.
	#glutFullScreen()

	# When we are doing nothing, redraw the scene.
    glutIdleFunc(DrawGLScene)
	
	# Register the function called when our window is resized.
    glutReshapeFunc(ReSizeGLScene)
	
	# Register the function called when the keyboard is pressed.  The call
	# setup glutSpecialFunc () is needed to receive "keyboard function or
	# directional keys."
    glutKeyboardFunc(keyPressed)
    glutSpecialFunc(keyPressed)
        
	# We've told Glut the type of window we want, and we've told glut about
	# various functions that we want invoked (idle, resizing, keyboard
	# events).  Glut has done the hard work of building up thw windows DC
	# context and tying in a rendering context, so we are ready to start
	# making immediate mode GL calls.  Call to perform inital GL setup (the
	# clear colors, enabling modes, and most releveant - consturct the
	# displays lists for the bitmap font.
    InitGL(640, 480)

	# Start Event Processing Engine	
    glutMainLoop()

if __name__ == '__main__':
    main()
