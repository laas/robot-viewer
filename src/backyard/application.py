#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang

from math import pi, sin, cos
import time,os,threading
import pyglet
from pyglet.window import Window,key,mouse
from pyglet.gl import *
from nutshell import *
import robo,robotLoader
from motionRecord import Motion
from camera import Camera,norm,normalized
from math import atan2,sin,cos,sqrt,acos,pi
import warnings,sys
import numpy as np
import re,imp

def customwarning(message,cat,filename,lineno,file="out.err",line=0):
     print "WARNING: %s"%message

warnings.showwarning=customwarning

noTkinter=False
try:
    import Tkinter, tkFileDialog 
except:
    warnings.warn("Tkinter not available")
    noTkinter=True

from simplui import *

##########################################################
################# HELPER FUNCTIONS #######################
##########################################################
        
def draw_skeleton2(robot):
    glMaterialfv(GL_FRONT_AND_BACK, \
                     GL_AMBIENT_AND_DIFFUSE, COLOR_RED)
    glMaterialfv(GL_FRONT_AND_BACK, \
                     GL_SPECULAR, COLOR_RED)
    # draw_skeleton a sphere at each mobile joint
    if robot.jointType in ["free","rotate"]:
        Tmatrix=robot.globalTransformation
        R=Tmatrix[0:3,0:3]
        p=Tmatrix[0:3,3]
        agax=rot2AngleAxis(R)        
        glPushMatrix()
        glTranslatef(p[0],p[1],p[2])
        glRotated(agax[0],agax[1],agax[2],agax[3])
        pos=robot.globalTransformation[0:3,3]
        qua = gluNewQuadric()
        r=0.01
        h=0.05

        if robot.jointType=="free":
            gluSphere(qua,r,10,5)
            glPopMatrix()
        elif robot.jointType=="rotate":         
            if robot.axis in ("X","x"):
                glRotated(90,0,1,0)
            elif robot.axis in ("Y","y"):
                glRotated(90,1,0,0)
            glTranslated(0.0,0.0,-h/2)
            gluDisk(qua,0,r,10,5)
            gluCylinder(qua,r,r,h,10,5)
            glTranslated(0.0,0.0,h)
            gluDisk(qua,0,r,10,5)
            glPopMatrix()

        if robot.jointType=="rotate":
            parent=robot.parent
            p2=parent.globalTransformation[0:3,3]
            p1=p
            ## draw link
            r=0.0025
            h=norm(p2-p1)
            dir=normalized(p2-p1)
            alpha=180/pi*acos(dir[2])
            axis=np.cross(np.array([0,0,1]),dir)
            glPushMatrix()
            glTranslatef(p1[0],p1[1],p1[2])
            glRotated(alpha,axis[0],axis[1],axis[2])
            gluCylinder(qua,r,r,h,10,5)
            glPopMatrix()

    for child in robot.children:
        draw_skeleton2(child)
        
def draw_skeleton(robot):
    glMaterialfv(GL_FRONT_AND_BACK, \
                     GL_AMBIENT_AND_DIFFUSE, COLOR_RED)
    glMaterialfv(GL_FRONT_AND_BACK, \
                     GL_SPECULAR, COLOR_RED)
    # draw_skeleton a sphere at each mobile joint
    if robot.jointType in ["free","rotate"]:
        Tmatrix=robot.globalTransformation
        R=Tmatrix[0:3,0:3]
        p=Tmatrix[0:3,3]
        agax=rot2AngleAxis(R)        

        if robot.jointType=="rotate":
            parent=robot.parent
            p2=parent.globalTransformation[0:3,3]
            p1=p
            glPushMatrix()
            glTranslated(p1[0],p1[1],p1[2])
            qua = gluNewQuadric()         
            gluSphere(qua,0.01,5,5)
            glPopMatrix()
            ## draw link
            glPushMatrix()
            glBegin(GL_LINES)
            glVertex3f(p1[0],p1[1],p1[2])
            glVertex3f(p2[0],p2[1],p2[2])
            glEnd()
            glPopMatrix()

    for child in robot.children:
        draw_skeleton(child)
        

def vec(*args):
    return (GLfloat * len(args))(*args)

COLOR_BLUE =vec(0,0,1,1)
COLOR_RED  =vec(1,0,0,1)
COLOR_GREEN=vec(0,1,0,1)
COLOR_WHITE=vec(1,1,1,1)
COLOR_BLACK=vec(0,0,0,1)


def glsetup():
    # One-time GL setup
#    glutInit(0)
#    glEnable (GL_BLEND) 
#    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
#    glEnable(GL_LINE_SMOOTH)
#    glEnable(GL_POINT_SMOOTH)
#    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
#    glShadeModel(GL_SMOOTH)
    ca=0.0
    la=0.0
    qa=0.08

    glLightfv(GL_LIGHT0, GL_POSITION, vec(-3.0,3.0,3.0,1.0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, vec(1,1,1,1))
    glLightfv(GL_LIGHT0, GL_AMBIENT, vec(1,1,1,1))
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, ca)
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, la) 
    glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, qa)

    glEnable(GL_LIGHT0)

lp=[]
N=5
L=0.5*N
for i in range(-N,N+1):   
    lp+=[i*0.5,L,0.005]
    lp+=[i*0.5,-L,0.005]
    lp+=[L,i*0.5,0.005]
    lp+=[-L,i*0.5,0.005]
numpoints=len(lp)/3
floorvtl=pyglet.graphics.vertex_list(\
    numpoints,('v3f',lp))
def draw_floor():
    glPushMatrix()
    glMaterialfv(GL_FRONT_AND_BACK, \
                     GL_AMBIENT_AND_DIFFUSE, COLOR_WHITE)
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, COLOR_WHITE)
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0)
    floorvtl.draw(GL_LINES)

    glMaterialfv(GL_FRONT_AND_BACK, \
                     GL_AMBIENT_AND_DIFFUSE, vec(.0,0.0,0.2,1))
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, vec(0.0,0.0,0.0,1))
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0)

    pyglet.graphics.draw(4,GL_QUADS,
                         ('v3f',(-L,-L,0.0, L,-L,0.0, L,L,0, -L,L,0.0)))
    glPopMatrix()


def prepareMesh(app):
    def cl(l):
        if l:
            return vec(l[0],l[1],l[2],1)
        else:
            return None
    if app.specularColor:
        glMaterialfv(GL_FRONT, GL_SPECULAR,cl(app.specularColor ))
    if app.emissiveColor:
        glMaterialfv(GL_FRONT, GL_EMISSION,cl(app.emissiveColor ))
    if app.diffuseColor:
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,cl(app.diffuseColor ))
    if app.shininess:
        glMaterialfv(GL_FRONT, GL_SHININESS,vec(app.shininess))


class StopWatch(object):
    def __init__(self):
        self.chronos=dict()
    def tic(self,events=[]):
        for event in events:
            try:
                c=self.chronos[event]
                if c[1]=="Running":
                    raise Exception\
                        ("can't tic a chrono while its running")
            except KeyError:
                self.chronos[event]=[time.time(),"Running"]

    def toc(self,event=None):
        c=None
        try:
            c=self.chronos[event]
        except KeyError:
            raise Exception("Invalid chrono %s, available chronos are %s:"\
                                 %(str(event),str(self.chronos))  )
        if c:
            c[1]="Stop"
            return time.time()-c[0]

####################################################
################# MAIN CLASS #######################
####################################################

defaultRobotKinematics="./hrpmodel/HRP2JRL/model/HRP2JRLBush_main.wrl"
defaultBasename="data/seq-cleo0.3x-wbm"



class Application(object):
     """Main class of robot-viewer, contains OpenGL window, robot's kinematics
     structures, GUI, corbaserver server
     """
     def __init__(self):
         self.robot=None
         self.state=None

         ## two beautiful window
         self.w1=None  # GUI
         self.w2=None  # OpenGL
         self.status="Uninitialized"
         self.keys=None #keyboard handler
         self.GUIthemes=None
         self.GUItheme=0
         self.GUIframe=None


         ## redering stuff
         self.RobotKinematicsFile=None
         self.numMeshes=0
         self.meshes=[]         
         self.meshBatches=[]
         self.shapeBatches=[]
         self.meshId2glListId=dict()

         ## motion records
         self.motion=Motion()
         self.bn_list=[]    # list of motion
         self.bn_info_list=[]    # list of motion
         self.current_bn_id=0


         ## legacy stuff for drawing the rotating torus
         self.rx=0
         self.ry=0
         self.rz=0
         self.line_index_pos=0
         self.line_index_wst=0
         self.line_index_rpy=0
         self.camera=Camera()

         ## flags 
         self.debug=False
         self.measureTime=True
         self.simplify=False
         self.showMesh=True
         self.verbose=False

         ## time stuff
         self.stopwatch=StopWatch()
         self.simTime=None
         self.appTime=time.time()
         self.lastTime=0
         self.count=0
         self.fps=1000
         self.timeFactor=1.0
         self.maxSimTime=0.0
         self.fps_display = pyglet.clock.ClockDisplay()


     def getRobot(self):
          """
          Getter function for robot attribute 

          :returns: The robot to be rendered in the scene
          :rtype: :class:`robo.BaseNode`.
          """
          return self.robot


     def init(self):
          '''
          Do the following tasks in order:
            * load motion segment 
            * load robot
            * init pyglet windows (OpenGL and GUI)
            * print instructions to terminal
          '''
          self.stopwatch.tic([1,2,3,4])

          self.loadBasename()

          t1=self.stopwatch.toc(1)
          self.loadRobot(self.RobotKinematicsFile)

          t2=self.stopwatch.toc(2)
          self.initWindow()

          t3=self.stopwatch.toc(3)
          
          self.bn_list=[defaultBasename]
          self.bn_info_list=[self.motion.getInfo()]
          self.current_bn_id=0

          t4=self.stopwatch.toc(4)
          if self.measureTime:
               print "loadMovement \t: %3.2fs \nloadRobot \t: %3.2fs \ninitWindow \t: %3.2fs\ninitUI \t: %3.2fs"\
                 %(t1,t2-t1,t3-t2,t4-t3)
               print "Total       \t: %3.2fs"%(t4)

          self.usage()

     def run(self):      
         '''
         Infinit loop that computes robot kinematics chain, updates openGL widonw
         '''
         pyglet.app.run()

     def loadBasename(self,abn=defaultBasename):        
         self.state="LOADING"
         print "loading %s"%abn
         try:
             self.motion.loadBasename(abn)
             self.simTime=self.motion.getTimeMin()

         except:
             warnings.warn("Unable to load motion file")
             self.simTime=0.0
             self.state=None
         print "loaded"
         self.state="STOP"

     def usage(self):
         print("""
 Press a key to:
 X       : play loaded motion
 C       : pause/resume motion
 V       : stop motion
 F       : speed up
 B       : slow down
 R       : reverse
 M       : toggle robot mesh
 """)

     def _playMovement(self):
         if self.state!="PLAY":
             self.state="PLAY"
         else:
             print "already playing. Press V to Stop or C to Pause"

     def _stopMovement(self):
         if self.state!="STOP":
             self.state="STOP"
             self.simTime=0
         else:
             print "already at rest. Moving to initial pos"
             self._setRobot(0)
             self.line_index=0

     def _pauseMovement(self):
         if self.state=="PAUSE":
             self.state="PLAY"
         elif self.state=="PLAY":
             self.state="PAUSE"

     def _on_resize(self,width, height):
         glViewport(0, 0, width, height)
         glMatrixMode(GL_PROJECTION)
         glLoadIdentity()
         gluPerspective(60., width / float(height), 0.1, 500.)
         glMatrixMode(GL_MODELVIEW)


     def _on_key_press(self,symbol, modifiers):
             if symbol == key.V:
                 self._stopMovement()
                 self.timeFactor=1
             elif symbol == key.X:
                 self._playMovement()
             elif symbol == key.C:
                 self._pauseMovement()
             elif symbol == key.F:
                 self.timeFactor*=2
             elif symbol == key.B:
                 self.timeFactor*=0.5
             elif symbol == key.R:
                 self.timeFactor*=-1
             elif symbol == key.M:
                 self.showMesh=not self.showMesh

     def loadRobot(self,RobotKinematicsFile=None):
         self.state="LOADING"
         import pickle
         self.stopwatch.tic(["lr"])
         if RobotKinematicsFile:
             self.robot=robotLoader.robotLoader(RobotKinematicsFile,\
                                                     not self.simplify)
         else:
             if not os.path.exists('lastRobot.pickle'):
                 warnings.warn("""No robot found. If this is your first time, 
                                  use -w RobotKinematics_File.wrl to load a robot.""")

                 tmp=imp.find_modue('robotviewer')
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
                  self.robot=pickle.load(open('lastRobot.pickle','r'))
             print "loaded ",self.robot.name


         foo=self.stopwatch.toc("lr")
         if self.measureTime:
             print "RobotKinematicsLoader\t: %3.2fs"%foo

         self.state="STOP"
         self.stopwatch.tic(["vl"])
         self.meshes=self.robot.mesh_list 
         self.numMeshes=len(self.meshes)
         self.shapeBatches=[None]*len(self.meshes)
         self.meshBatches=[None]*len(self.meshes)
         for i in range(self.numMeshes):
             mesh=self.meshes[i]
             shapes=mesh.shapes
             self.shapeBatches[i]=[None]*len(mesh.shapes)
             self.meshBatches[i]=pyglet.graphics.Batch()
             for j in range(len(shapes)):
                 self.shapeBatches[i][j]=pyglet.graphics.Batch()
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

                 self.meshBatches[i].add_indexed(npoints,GL_TRIANGLES,None,\
                                                      tri_list,('v3f', coord),\
                                                      ('n3f',ashape.geo.norm))
                 self.shapeBatches[i][j].add_indexed\
                     (npoints,GL_TRIANGLES,None,tri_list,('v3f', coord),\
                           ('n3f',ashape.geo.norm))

         f=open('lastRobot.pickle','w')
         pickle.dump(self.robot,f)
         f.close()
         self._setRobot(self.simTime)
         self.state="STOP"
         # create_the display list
         foo=self.stopwatch.toc("vl")
         if self.measureTime:
             print "createVList\t: %3.2fs"%foo


     ###############################
     #      INIT THE WINDOWS       #
     ###############################



     def initWindow(self):
         # disable error checking for increased performance
         pyglet.options['debug_gl'] = False
         if self.verbose:
              print "entering initWindow"
         self.w1 = Window(300,400,caption='GUI',resizable=False)
         self.w1.on_draw = self._on_draw_GUI
         self.w1.switch_to()
         pyglet.gl.glClearColor(0.8, 0.8, 1.0, 1.0)
         tmp=imp.find_module('robotviewer')
         path=tmp[1]
         if self.verbose:
              print "loading themes in %s"%path
         themes = [Theme(path+'/themes/macos'),\
                       Theme(path+'/themes/pywidget')]
         theme = 0

         # create a frame to contain our gui, the full size of our window
         self.GUIframe = Frame(themes[theme], w=300, h=400)
         # let the frame recieve events from the window
         self.w1.push_handlers(self.GUIframe)

         # create and add a window

         ## HELPER FUNCTION ##
         def button_action(button):
             name=button._get_text()
             if name=="LoadRobot":
                 if not noTkinter:
                     root = Tkinter.Tk()
                     root.withdraw()
                     filename = tkFileDialog.askopenfilename()
                     root.destroy()
                     if filename:
                         self.loadRobot(filename)
                 else:
                     warnings.warn("Tkinter not available")
             elif name=="LoadMotion":
                 if not noTkinter:
                     root = Tkinter.Tk()
                     root.withdraw()
                     filename = tkFileDialog.askopenfilename()
                     root.destroy()

                 # strip the extension
                     if filename:
                         bn=re.sub(r"\.\w+$","",filename)
                         self.loadBasename(bn)
                 else:
                     warnings.warn("Tkinter not available")
             elif name=="Play":
                 self._playMovement()
             elif name=="Pause":
                 self._pauseMovement()
             elif name=="Stop":
                 self._stopMovement()
             elif name=="Reverse":
                 self.timeFactor*=-1
             elif name=="Speedup":
                 self.timeFactor*=2
             elif name=="Slowdown":
                 self.timeFactor*=0.5
             elif name=="Exit":
                 sys.exit(0)

         def plan_and_play(button):
             self.state="COMPUTING"
             targ_text = self.GUIframe.get_element_by_name\
                 ('target_input').text            
             nsteps_text = self.GUIframe.get_element_by_name\
                 ('nstep_input').text            
             output="m_"+nsteps_text+"_"+targ_text
             output=re.sub(r" ","_",output)
             command="./ball-picking $HOME/licenses "+\
                 targ_text + " " + nsteps_text +" " +output

             print "executing '%s'"%command
             try:
                 os.system(command)
             except:
                 print "cannot execute %s"%command
             else:
                 self.loadBasename("seq-"+output+"-wbm")
                 self.simTime=0
                 self.state="STOP"

         ## END HELPER FUNCTIONS ##
         dialogue2 = Dialogue('GUI', x=0, y=400, content=
                              VLayout(w=300,children=[
                     HLayout(autosizey=True,vpadding=0,children=[
                             Button('LoadRobot',action=button_action),
                             Button('LoadMotion',action=button_action),
                             Button('Exit',action=button_action),
                             ]),
                     HLayout(autosizey=True,vpadding=0,children=[
                             Button('Play',action=button_action),
                             Button('Pause',action=button_action),
                             Button('Stop',action=button_action),
                             ]),
                     HLayout(autosizey=True,vpadding=0,children=[
                             Button('Reverse',action=button_action),
                             Button('Speedup',action=button_action),
                             Button('Slowdown',action=button_action),
                             ]),
                     FoldingBox('Info',name='Info',content=
                                VLayout(w=300,children=[
                                Label("Motion : None",name='motion_info_label'),
                                Label("Time   : None",name='time_label'),
                                Label("State  : None",name='state_label'),
                                Label("fps    : None",name='fps_label'),
                                Label("Speed    : None",name='speed_label'),
                                ]),
                                ),
 #                     FoldingBox('Plan',name='Plan',content=
 #                                VLayout(w=300,children=[
 #                                 HLayout(autosizey=True,vpadding=0,children=[
 #                                         Label("Target"),TextInput(text="0 0 0.05",name="target_input")
 #                                         ]),
 #                                 HLayout(autosizey=True,vpadding=0,children=[
 #                                         Label("Numsteps"),TextInput(text="2",name="nstep_input")
 #                                         ]),
 #                                 Button('Plan and Play',action=plan_and_play)
 #                                ]),
 #                                ),
                     ])                         

         )

         self.GUIframe.add( dialogue2 )        


         #### GL ##
         if self.verbose:
              print "creating OpenGL window"
         try:
             config = Config(sample_buffers=1, samples=4, 
                             depth_size=16, double_buffer=True,)
             self.w2 = Window(900 , 600,resizable=False,caption='OpenGL',\
                                  config=config)
         except pyglet.window.NoSuchConfigException:
             self.w2 = Window(resizable=True)
         self.w2.on_draw = self._on_draw_scene
         self.w2.on_resize = self._on_resize
         self.w2.on_key_press = self._on_key_press
         self.keys = key.KeyStateHandler()
         self.w2.push_handlers(self.keys)
         glsetup()
         self.w2.switch_to()        
         pyglet.clock.schedule(self._update)


     ###########################
     ### GUI on draw binding ###
     ###########################
     def _on_draw_GUI(self):
         if  self.state!=None:
             if self.appTime-self.lastTime < 1.0:
                 pass
             else:
                 self.lastTime=time.time()
                 self.fps=self.count
                 self.count=0
             self.w1.clear()
             foo=self.GUIframe.get_element_by_name('time_label')
             try:
                 foo.text="time         : %3.2f"%self.simTime
             except:                    
                 foo.text="time         : "+str(self.simTime)
             foo=self.GUIframe.get_element_by_name('state_label')
             foo.text="State        : "+str(self.state)
             foo=self.GUIframe.get_element_by_name('motion_info_label')
             foo.text="Motion info  : "+str(self.motion.getInfo())
             foo=self.GUIframe.get_element_by_name('fps_label')
             foo.text="Fps          : "+str(self.fps)
             foo=self.GUIframe.get_element_by_name('speed_label')            
             foo.text="Playing speed: %3.2f"%self.timeFactor
             if self.timeFactor > 0:
                 foo.text+=" (Forwards)"
             else:
                 foo.text+=" (Backwards)"

         self.GUIframe.draw()


     ##############################
     ### OpenGL on draw binding ###
     ##############################
     def _on_draw_scene(self):
         self.w2.clear()
         self.count+=1            
         glLoadIdentity()
         p=self.camera.position
         f=self.camera.lookat
         u=self.camera.up
         gluLookAt(p[0],p[1],p[2],f[0],f[1],f[2],u[0],u[1],u[2])
         draw_floor()

         if self.state=="LOADING" or self.state==None or self.robot==None:
             return
         if self.simplify or (not self.showMesh) or self.robot.mesh_list==[]:
             draw_skeleton2(self.robot)
         else:
             for i in range(self.numMeshes):
                 mesh=self.meshes[i]
                 prepareMesh(mesh.shapes[0].app)
                 Tmatrix=mesh.globalTransformation
                 R=Tmatrix[0:3,0:3]
                 p=Tmatrix[0:3,3]
                 agax=rot2AngleAxis(R)        
                 glPushMatrix()
                 glTranslatef(p[0],p[1],p[2])
                 glRotated(agax[0],agax[1],agax[2],agax[3])
                 for j in range(len(mesh.shapes)):
                     prepareMesh(mesh.shapes[j].app)
                     self.shapeBatches[i][j].draw()
                 glPopMatrix()

         glMaterialfv(GL_FRONT_AND_BACK,\
                          GL_AMBIENT_AND_DIFFUSE, COLOR_GREEN)
         glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, COLOR_GREEN)

         glPushMatrix()

         try:
             targetx=float(self.motion.info.split()[1])
             targety=float(self.motion.info.split()[2])
             targetz=float(self.motion.info.split()[3])
             glTranslatef(targetx,targety, targetz)
             sphere = gluNewQuadric() 
             gluSphere(sphere,0.03,10,10)
         except Exception,error:
             warnings.warn("Couldn't draw the target. Caught exception: %s "\
                                %error )

         glPopMatrix()

     def _update(self,dt):
         if self.state==None or self.state=="LOADING" \
                  or self.robot==None or self.state=="COMPUTING":
             return

         if self.keys[key.LEFT]:
             self.camera.rotateZ(dt*0.5)

         if self.keys[key.RIGHT]:
             self.camera.rotateZ(-dt*0.5)

         if self.keys[key.UP]:
             if self.keys[key.LSHIFT] or self.keys[key.RSHIFT] :
                 self.camera.moveR(-dt*0.5)
             else:                
                 self.camera.moveZ(dt*0.5)

         if self.keys[key.DOWN]:
             if self.keys[key.LSHIFT] or self.keys[key.RSHIFT] :
                 self.camera.moveR(dt*0.5)
             else:                
                 self.camera.moveZ(-dt*0.5)

         self.camera.adjust(self.robot,dt)
         self.appTime=time.time()


         if self.state=="STOP":
             self.simTime=0.0
             return
         elif self.state=="PAUSE":
             return
         elif self.state=="PLAY":
             self.rx += dt * 1
             self.ry += dt * 80
             self.rz += dt * 30
             self.rx %= 360
             self.ry %= 360
             self.rz %= 360
             self.simTime=min(max(self.motion.getTimeMin(),self.simTime+dt*
                                    self.timeFactor),self.motion.getTimeMax())
             self._setRobot(self.simTime)


     def _setRobot(self,atime):
         if self.motion.posRecord.isEmpty():
             return
         while self.line_index_pos<len(self.motion.posRecord.times)-1\
            and self.motion.posRecord.times[self.line_index_pos+1]<atime:
             self.line_index_pos+=1
         else:
             while self.line_index_pos >0 \
            and self.motion.posRecord.times[self.line_index_pos] > atime:
                 self.line_index_pos-=1
         # the above snipsets move the line_index point to the nearest time to
         # the left of atime in the pos_times list
         angles=self.motion.posRecord.coors[self.line_index_pos]
         try:
             self.robot.jointAngles(angles)
         except Exception,error:
             warnings.warn("_setRobot warning, caught excpetion: %s"%error)

 #         #+ Wst
 #         while self.line_index_wst < len(self.motion.wstRecord.times)-1 \
 #            and self.motion.wstRecord.times[self.line_index_wst+1]<atime:
 #             self.line_index_wst+=1
 #         else:
 #             while self.line_index_wst >0 \
 #            and self.motion.wstRecord.times[self.line_index_wst] > atime:
 #                self.line_index_wst-=1

         try:
             self.line_index_wst=self.line_index_pos
             wstPos=self.motion.wstRecord.coors[self.line_index_wst]
             self.robot.waist.translation=wstPos

 #         # Rpy
 #         while self.line_index_rpy < len(self.motion.rpyRecord.times)-1 \
 #            and self.motion.rpyRecord.times[self.line_index_rpy+1]<atime:
 #             self.line_index_rpy+=1
 #         else:
 #             while self.line_index_rpy >0 \
 #            and self.motion.rpyRecord.times[self.line_index_rpy] > atime:
 #                 self.line_index_rpy-=1

             self.line_index_rpy=self.line_index_pos
             wstRpy=self.motion.rpyRecord.coors[self.line_index_rpy]
             self.robot.waist.rpy=wstRpy

         except Exception,error:
             warnings.warn( "Couldn't set wst position. caught exception: %s "\
                                 %error)

         self.robot.update()


     def execute(self,cmd):
         '''
         Execute a string command of form:
            <path> <command> <arguments>
         '''
         words=cmd.split()
         if words==[]:
             return "Error: Blank msg"
         path=words[0]
         if words[1:]==[]:
             return """Available commands:
    list                          : list loaded segments
    play [id]                     : play a segment in the list
    stop                          :
    pause                         :
    next                          :
    prev                          :
    speed <s>                     : set playing speed
    load <segment.pos>            : load a segment
    getConfig                     : print robot config (x y z r p y q)
    setConfig <0 0.6 etc.>        : set robot to a config (x y z r p y q)
 """
         elif words[1]=="getConfig":
             return str(self.robot.waist.translation+self.robot.waist.rpy\
                             +[joint.angle for joint in self.robot.joint_list])

         elif words[1]=="setConfig":
             if self.state!="STOP":
                  return "stop the robot first"    
             else:
                  try:
                       config=[float(w) for w in words[2:]]
                       if len(config)!=len(self.robot.joint_list)+5:  
                            # first node does not count
                            return "config dimension not matched, need %d , %d given"\
                                %(len(config),len(self.robot.joint_list)+6)
                       else:
                            self.robot.waistPos(config[0:3])
                            self.robot.waistRpy(config[3:6])
                            self.robot.jointAngles(config[6:])
                            self.robot.update() # updaten the kinematic chains
                  except Exception,error:
                       return str(error)
             return ""
         elif words[1]=="status":
             return self.status

         elif words[1]=="play":
             if words[2:]:
                 id=int(words[2])
                 try:
                     self.loadBasename(self.bn_list[id])
                     self.current_bn_id=id
                 except:
                     return "Caught exception %s"%error

             self._playMovement()
             return ""    

         elif words[1]=="stop":
             self._stopMovement()
             return ""    

         elif words[1]=="pause":
             self._pauseMovement()
             return ""    

         elif words[1]=="speed":
             if len(words)==3:
                 self.timeFactor=float(words[2])
                 return ""
             else:
                 return "need 1 argument"

         elif words[1]=="list":
             s="id\tx\ty\tsteps\tbasename\n---------------\n"
             for i in range(len(self.bn_list)):
                 if i==self.current_bn_id:
                     s+="-> "
                 else:
                     s+="   "
                 s+="%d\t%s %s\n"%(i,self.bn_info_list[i],self.bn_list[i])
             return s


         elif words[1]=="add":
             if not words[2:]:
                 return "need a basename"
             bn=words[2]
             try:
                 info=Motion().loadInfo(path+bn+'.target')
                 self.bn_info_list.append(info)
                 self.bn_list.append(path+bn)
                 return ""
             except Exception,error:
                 return error

         elif words[1]=="clear":
             self.bn_list=[]
             return ""
         else:
             return "Unknown command: %s"%words[1]

