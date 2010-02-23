from pyglet.gl import *
from OpenGL.GLUT import *
from pyglet.window import Window
from pyglet import app
win = Window(fullscreen=True, visible=False)

def display():
    glCallList(1)
    return

@win.event
def on_resize(width, height):
    glMatrixMode(GL_PROJECTION)
    gluPerspective(60., width / float(height), .1, 1000.)
    glMatrixMode(GL_MODELVIEW)
    return pyglet.event.EVENT_HANDLED



def set_viewport():
    glMatrixMode(GL_PROJECTION)
    gluPerspective(70.,1.,1.,40.) # angle, aspect ratio, near clip, far clip
    glMatrixMode(GL_MODELVIEW)
    gluLookAt(
	5,0,1.7,     # camera position
	0,0,0,       # where camera points
	0,0,1)       # which direction is up
    glPushMatrix()

def vec(*args):
    return (GLfloat * len(args))(*args)

def lightning():
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    lightZeroPosition = vec(0,5.,10.,1.)
    lightZeroColor = vec(1.0,1.0,1.0,1.0) # greenish
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
#    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
#    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_diffuse);
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

    mat_diffuse = vec(1.0,.5 ,.8,1.0) # greenish
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_diffuse);
    glBegin(GL_QUADS)
    glVertex3f(-5,-5,0)
    glVertex3f(-5,5,0)
    glVertex3f(5,5,0)
    glVertex3f(-5,-5,0)
    glEnd()

    # draw a grid on the floor 0.5mx0.5m, in an area of 5mx5m

@win.event
def on_draw():
    glClear(GL_COLOR_BUFFER_BIT)
    display()

def setup():
    # One-time GL setup
    glutInit(0)
    glClearColor(1, 1, 1, 1)
    glColor3f(1, 0, 0)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_CULL_FACE)
    # Uncomment this line for a wireframe view
    #glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)

    # Simple light setup.  On Windows GL_LIGHT0 is enabled by default,
    # but this is not the case on Linux or Mac, so remember to always 
    # include it.
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHT1)

    # Define a simple function to create ctypes arrays of floats:
    def vec(*args):
        return (GLfloat * len(args))(*args)

    glLightfv(GL_LIGHT0, GL_POSITION, vec(.5, .5, 1, 0))
    glLightfv(GL_LIGHT0, GL_SPECULAR, vec(.5, .5, 1, 1))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, vec(1, 1, 1, 1))
    glLightfv(GL_LIGHT1, GL_POSITION, vec(1, 0, .5, 0))
    glLightfv(GL_LIGHT1, GL_DIFFUSE, vec(.5, .5, .5, 1))
    glLightfv(GL_LIGHT1, GL_SPECULAR, vec(1, 1, 1, 1))

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, vec(0.5, 0, 0.3, 1))
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, vec(1, 1, 1, 1))
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50)

def main():
#     glutInit("Hello, World")
#     glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH)
#     glutInitWindowSize(400,400)
#     glutCreateWindow("Hello, World")
    
#     lightning()
    win=Window()
    setup()
    init_display_list()
    set_viewport()
    app.run()

if __name__ == "__main__":main()
