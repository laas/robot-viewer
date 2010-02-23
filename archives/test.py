from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *


def display():
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glCallList(1)
    glutSwapBuffers()
    return

def set_viewport():
    glMatrixMode(GL_PROJECTION)
    gluPerspective(70.,1.,1.,40.) # angle, aspect ratio, near clip, far clip
    glMatrixMode(GL_MODELVIEW)
    gluLookAt(
	5,0,1.7,     # camera position
	0,0,0,       # where camera points
	0,0,1)       # which direction is up
    glPushMatrix()

def lightning():
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    lightZeroPosition = [0,5.,10.,1.]
    lightZeroColor = [1.0,1.0,1.0,1.0] # greenish
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1)
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05)
    glEnable(GL_LIGHT0)


def init_display_list():
    glNewList(1,GL_COMPILE)
    floor()
    glPushMatrix()
    mat_diffuse = [1.0,1.0 ,1.0,1.0] # greenish                                     
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_diffuse);
    glTranslatef(0,0,0.5) #move to where we want to put object
    glutSolidSphere(.2,20,20) # make radius 1 sphere of res 5x5
    glPopMatrix()
    glEndList()
    return

def floor():
    glBegin(GL_LINES)    
    for i in range(-10,11):   
        glVertex3f(i*0.5,5.0,0.0)
        glVertex3f(i*0.5,-5.0,0.0)
        glVertex3f(5,i*0.5,0.0)
        glVertex3f(-5,i*0.5,0.0)
    glEnd()

#     mat_diffuse = [1.0,.5 ,.8,1.0] # greenish
#     glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
#     glMaterialfv(GL_FRONT, GL_AMBIENT, mat_diffuse);
#     glBegin(GL_QUADS)
#     glVertex3f(-5,-5,0)
#     glVertex3f(-5,5,0)
#     glVertex3f(5,5,0)
#     glVertex3f(-5,-5,0)
#     glEnd()

    # draw a grid on the floor 0.5mx0.5m, in an area of 5mx5m


def main():
    glutInit("Hello, World")
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH)
    glutInitWindowSize(400,400)
    glutCreateWindow("Hello, World")
    init_display_list()
    lightning()
    set_viewport()
    glClearColor(0.,0.,0.,1.)
    glutDisplayFunc(display)
    glutMainLoop()
    print "can I do something here?"
if __name__ == "__main__":main()
