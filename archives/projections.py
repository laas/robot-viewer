# About OpenGL Projections
#---------------------------------
from pyglet import window,image
from pyglet.window import key
from pyglet.gl import *

def opengl_init():
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glDepthFunc(GL_LEQUAL)

def playground():

    # Test your code here
    sphere = gluNewQuadric()
    gluSphere(sphere,10.0,10,10) 

#     glColor4f(1,1,1,1)
#     glBegin(GL_LINES)
#     glVertex3f(0,0,0)
#     glVertex3f(640,480,0)

#     glVertex3f(0,480,0)
#     glVertex3f(640,0,0)
#     glEnd()

class camera():
    mode=1
    x,y,z=0,0,512
    rx,ry,rz=30,-45,0
    w,h=640,480
    far=8192
    fov=60
           
    def view(self,width,height):
        self.w,self.h=width,height
        glViewport(0, 0, width, height)
        print "Viewport "+str(width)+"x"+str(height)
        if self.mode==2: self.isometric()
        elif self.mode==3: self.perspective()
        else: self.default()
           
    def default(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, self.w, 0, self.h, -1, 1)
        glMatrixMode(GL_MODELVIEW)
       
    def isometric(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-self.w/2.,self.w/2.,-self.h/2.,self.h/2.,0,self.far)
        glMatrixMode(GL_MODELVIEW)
       
    def perspective(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(self.fov, float(self.w)/self.h, 0.1, self.far)
        glMatrixMode(GL_MODELVIEW)
       
    def key(self, symbol, modifiers):
        if symbol==key.F1:
            self.mode=1
            self.default()
            print "Projection: Pyglet default"
        elif symbol==key.F2:
            print "Projection: 3D Isometric"
            self.mode=2
            self.isometric()
        elif symbol==key.F3:
            print "Projection: 3D Perspective"
            self.mode=3
            self.perspective()
        elif self.mode==3 and symbol==key.NUM_SUBTRACT:
            self.fov-=1
            self.perspective()
        elif self.mode==3 and symbol==key.NUM_ADD:
            self.fov+=1
            self.perspective()
        else: print "KEY "+key.symbol_string(symbol)
       
    def drag(self, x, y, dx, dy, button, modifiers):
        if button==1:
            self.x-=dx*2
            self.y-=dy*2
        elif button==2:
            self.x-=dx*2
            self.z-=dy*2
        elif button==4:
            self.ry+=dx/4.
            self.rx-=dy/4.
       
    def apply(self):
        glLoadIdentity()
        if self.mode==1: return
        glTranslatef(-self.x,-self.y,-self.z)
        glRotatef(self.rx,1,0,0)
        glRotatef(self.ry,0,1,0)
        glRotatef(self.rz,0,0,1)

def x_array(list):
    return (GLfloat * len(list))(*list)
       
def axis(d=200):
    vertices,colors=[],[]   
    #XZ RED 
    vertices.extend([-d, 0,-d, d, 0,-d, d, 0, d,-d, 0, d])
    for i in range (0,4): colors.extend([1,0,0,0.5])
    #YZ GREEN 
    vertices.extend([ 0,-d,-d, 0,-d, d, 0, d, d, 0, d,-d])
    for i in range (0,4): colors.extend([0,1,0,0.5])
    #XY BLUE 
    vertices.extend([-d,-d, 0, d,-d, 0, d, d, 0,-d, d, 0])
    for i in range (0,4): colors.extend([0,0,1,0.5])
    return x_array(vertices),x_array(colors)
AXIS_VERTICES,AXIS_COLORS=axis()

def draw_vertex_array(vertices,colors,mode=GL_LINES):
    glEnableClientState(GL_VERTEX_ARRAY)
    glEnableClientState(GL_COLOR_ARRAY)
    glColorPointer(4, GL_FLOAT, 0, colors)
    glVertexPointer(3, GL_FLOAT, 0, vertices)
    glDrawArrays(GL_QUADS, 0, len(vertices)/3)
    glDisableClientState(GL_VERTEX_ARRAY)
    glDisableClientState(GL_COLOR_ARRAY)

def draw_axis():
    glEnable(GL_DEPTH_TEST)
    draw_vertex_array(AXIS_VERTICES,AXIS_COLORS,GL_QUADS)
    glDisable(GL_DEPTH_TEST)

print "OpenGL Projections"
print "---------------------------------"
print "Projection matrix -> F1, F2, F3"
print "Camera -> Drag LMB,CMB,RMB"
print ""
cam=camera()
win = window.Window(resizable=True)
win.on_resize=cam.view
win.on_key_press=cam.key
win.on_mouse_drag=cam.drag
opengl_init()

while not win.has_exit:
    win.dispatch_events()
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    cam.apply()
    draw_axis()
    playground()
    win.flip()
