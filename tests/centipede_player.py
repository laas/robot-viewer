from Tkinter import *
import sys,time
from robotviewer.client import *

footscript="""glMaterialfv(GL_FRONT_AND_BACK,  GL_AMBIENT_AND_DIFFUSE, [1,1,1,1])
glMaterialfv(GL_FRONT_AND_BACK,  GL_SPECULAR           , [1,1,1,1])
glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1)
glBegin(GL_QUADS)
glVertex(0.105 ,0.079 ,0)
glVertex(-0.105,0.079 ,0)
glVertex(-0.105,-0.059,0)
glVertex(0.105 ,-0.059,0)
glEnd()
glMaterialfv(GL_FRONT_AND_BACK,  GL_SPECULAR           , [1,0,0,1])
glBegin(GL_LINES)
glVertex(0.105 ,0.079 ,0)
glVertex(-0.105,0.079 ,0)
glVertex(-0.105,0.079 ,0)
glVertex(-0.105,-0.059,0)
glVertex(-0.105,-0.059,0)
glVertex(0.105 ,-0.059,0)
glVertex(0.105 ,-0.059,0)
glVertex(0.105 ,0.079 ,0)
glEnd()
"""

lp=[]
N=5
L=0.5*N

for i in range(-N,N+1):   
    lp.append([i*0.5,L,0.005])
    lp.append([i*0.5,-L,0.005])
    lp.append([L,i*0.5,0.005])
    lp.append([-L,i*0.5,0.005] )
    
script = "glMaterialfv(GL_FRONT_AND_BACK,  GL_AMBIENT_AND_DIFFUSE, [1,1,1,1])\n"
script+= "glMaterialfv(GL_FRONT_AND_BACK,  GL_SPECULAR           , [1,1,1,1])\n"
script+= "glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0)\n"
script+= "glBegin(GL_LINES)\n"    
for point in lp:
    script+= "glVertex3f(%f, %f, %f)\n"%(point[0],point[1],point[2])
script+= "glEnd()"
execute('createElement','script floor %s'%(script),[])
time.sleep(1)
execute('enableElement','floor',[])

for i in range(100):
    execute('createElement','script fp%d %s'%(i,footscript),[])

class Application(Frame):

    def __init__(self, master=None):
        self.lines = open(sys.argv[1]).readlines()
        self.num_lines = len(self.lines);
        Frame.__init__(self, master)
        self.sc_val = 0
        self.pack()
        self.createWidgets()
        self.numsteps = 0

    def updateRobot(self,val):
        no_line = int (val)
        curline = self.lines[no_line]
        words = curline.split()
        extra_dof = len(words)-46
        config = [ float(words[i+extra_dof]) for i in range(46)]
        execute('updateElementConfig','hrp',config)        

        if self.numsteps < extra_dof/3:
            for i in range(extra_dof/3):
                execute('enableElement','fp%d'%i,[])
        for i in range(extra_dof/3,100):    
            execute('disableElement','fp%d'%i,[])
                        
        self.numsteps = extra_dof/3
        x = 0
        y = 0
        theta = 0
        for i in range(self.numsteps):
            x = float(words[3*i])
            y = float(words[3*i+1])
            theta = float(words[3*i+2])
            print i,x,y,theta
            execute('updateElementConfig','fp%d'%i,[x,y,0,0,0,theta])

    def createWidgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["command"] =  self.quit

        self.QUIT.pack({"side": "left"})

        self.sc = Scale (self, variable = self.sc_val, \
                             command = self.updateRobot, \
                             orient  = HORIZONTAL, length = 200, \
                             to = self.num_lines -1)
        self.sc.pack({"side": "bottom"})


root = Tk()
app = Application(master=root)
app.mainloop()
root.destroy()
