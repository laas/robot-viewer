#!/usr/bin/env python

from math import pi, sin, cos
import time
import pyglet
from pyglet.window import Window,key,mouse
from pyglet.gl import *
from nutshell import *
import robo,VRMLloader
from motionRecord import Motion
from camera import Camera
from math import atan2,sin,cos,sqrt
import warnings
##########################################################
################# MIS FUNCTIONS ##########################
##########################################################

def vec(*args):
    return (GLfloat * len(args))(*args)

COLOR_BLUE =vec(0,0,1,1)
COLOR_RED  =vec(1,0,0,1)
COLOR_GREEN=vec(0,1,0,1)
COLOR_WHITE=vec(1,1,1,1)
COLOR_BLACK=vec(0,0,0,1)


def glsetup():
    # One-time GL setup
    glutInit(0)

    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)

    ca=0.0
    la=0.6
    qa=0.1

    glLightfv(GL_LIGHT0, GL_POSITION, vec(-3.0,3.0,3.0,1.0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, vec(1,1,1,1))
    glLightfv(GL_LIGHT0, GL_AMBIENT, vec(1,1,1,1))
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, ca)
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, la)
    glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, qa)

    glLightfv(GL_LIGHT1, GL_POSITION, vec(3.0,3.0,3.0,1.0))
    glLightfv(GL_LIGHT1, GL_DIFFUSE, vec(1,1,1,1))
    glLightfv(GL_LIGHT1, GL_AMBIENT, vec(1,1,1,1))
    glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, ca)
    glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, la)
    glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, qa)

    glLightfv(GL_LIGHT2, GL_POSITION, vec(3.0,-3.0,3.0,1.0))
    glLightfv(GL_LIGHT2, GL_DIFFUSE, vec(1,1,1,1))
    glLightfv(GL_LIGHT2, GL_AMBIENT, vec(1,1,1,1))
    glLightf(GL_LIGHT2, GL_CONSTANT_ATTENUATION, ca)
    glLightf(GL_LIGHT2, GL_LINEAR_ATTENUATION, la)
    glLightf(GL_LIGHT2, GL_QUADRATIC_ATTENUATION, qa)

    glLightfv(GL_LIGHT3, GL_POSITION, vec(-3.0,-3.0,3.0,1.0))
    glLightfv(GL_LIGHT3, GL_DIFFUSE, vec(1,1,1,1))
    glLightfv(GL_LIGHT3, GL_AMBIENT, vec(1,1,1,1))
    glLightf(GL_LIGHT3, GL_CONSTANT_ATTENUATION, ca)
    glLightf(GL_LIGHT3, GL_LINEAR_ATTENUATION, la)   
    glLightf(GL_LIGHT3, GL_QUADRATIC_ATTENUATION, qa)

    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHT1)
    glEnable(GL_LIGHT2)
    glEnable(GL_LIGHT3)


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
    glEnable(GL_LINE_SMOOTH)
    glEnable(GL_POINT_SMOOTH)
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
        glMaterialfv(GL_FRONT, GL_EMISSION,cl(app.emissiveColor ))
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,cl(app.diffuseColor ))
        glMaterialfv(GL_FRONT, GL_SHININESS,vec(app.shininess))
    else:
        return
        glMaterialfv(GL_FRONT, GL_SPECULAR,vec(1,1,1,1))
        glMaterialfv(GL_FRONT, GL_EMISSION,vec(1,1,1,1))
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,vec(1,1,1,1))
        glMaterialfv(GL_FRONT, GL_SHININESS,vec(1.0))

#    if app.shininess:
#        glMaterialfv(GL_FRONT, GL_TRANSPARENCY,vec(app.transparency))


class StopWatch():
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
            raise Exception("Invalid chrono %s, available chronos are %s:"%(str(event),str(self.chronos))  )
        if c:
            c[1]="Stop"
            return time.time()-c[0]

####################################################
################# MAIN CLASS #######################
####################################################

defaultVRML="./hrpmodel/HRP2JRL/model/HRP2JRLBush_main.wrl"
defaultBasename="data/seq-cleo0.3x-wbm"

class Application:
    def __init__(self,db=False):
        self.robot=None
        self.state=None

        ## two beautiful window
        self.w1=None
        self.w2=None
        self.status="Uninitialized"
        self.keys=None #keyboard handler
        ## redering stuff
        self.VRMLFile=None
        self.meshes=None
        self.vertex_lists=None
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
        self.debug=db
        self.measureTime=True
        self.simplify=False
        self.showMesh=True

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

    def init(self):
        self.stopwatch.tic([1,2,3])

        self.loadBasename()

        t1=self.stopwatch.toc(1)
        self.loadRobot(self.VRMLfile)

        t2=self.stopwatch.toc(2)
        self.initWindow()

        t3=self.stopwatch.toc(3)
        
        self.bn_list=[defaultBasename]
        self.bn_info_list=[self.motion.getInfo()]
        self.current_bn_id=0

        if self.measureTime:
            print "loadMovement \t: %3.2fs \nloadRobot \t: %3.2fs \ninitWindow \t: %3.2fs"%(t1,t2-t1,t3-t2)
            print "Total       \t: %3.2fs"%(t3)

        self.usage()

    def run(self):
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

    def playMovement(self):
        if self.state!="PLAY":
            self.state="PLAY"
        else:
            print "already playing. Press V to Stop or C to Pause"

    def stopMovement(self):
        if self.state!="STOP":
            self.state="STOP"
            self.simTime=0
        else:
            print "already at rest. Moving to initial pos"
            self.setRobot(0)
            self.line_index=0

    def pauseMovement(self):
        if self.state=="PAUSE":
            self.state="PLAY"
        elif self.state=="PLAY":
            self.state="PAUSE"

    def on_resize(self,width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60., width / float(height), 0.1, 500.)
        glMatrixMode(GL_MODELVIEW)


    def on_key_press(self,symbol, modifiers):
            if symbol == key.V:
                self.stopMovement()
                self.timeFactor=1
            elif symbol == key.X:
                self.playMovement()
            elif symbol == key.C:
                self.pauseMovement()
            elif symbol == key.F:
                self.timeFactor*=2
            elif symbol == key.B:
                self.timeFactor*=0.5
            elif symbol == key.R:
                self.timeFactor*=-1
            elif symbol == key.M:
                self.showMesh=not self.showMesh


    def loadRobot(self,VRMLFile=None):
        import pickle
        self.stopwatch.tic(["lr"])
        if VRMLFile:
            self.robot=VRMLloader.VRMLloader(VRMLFile,not self.simplify)
            f=open('lastRobot.pickle','w')
            pickle.dump(self.robot,f)
            f.close()
        else:
            if not os.path.exists('lastRobot.pickle'):
                raise Exception("No robot found. If this is your first time, use -w VRML_File.wrl to load a robot.")
    
        print "loading the last loaded robot \n"
        self.robot=pickle.load(open('lastRobot.pickle','r'))
        print "loaded ",self.robot.name
        if self.measureTime:
            print "VRMLloader\t: %3.2fs"%self.stopwatch.toc("lr")
        self.stopwatch.tic(["vl"])
        self.meshes=self.robot.mesh_list 
        self.numMeshes=len(self.meshes)
        self.vertex_lists=[None]*self.numMeshes
        tri_lists=[None]*self.numMeshes

        for i in range(self.numMeshes):
            mesh=self.meshes[i]
            shapes=mesh.shapes
            self.vertex_lists[i]=[None]*len(shapes)
            tri_lists[i]=[None]*len(shapes)
            for j in range(len(shapes)):
                ashape=shapes[j]
                coord=ashape.geo.coord
                npoints=len(coord)/3
                tri_lists[i][j]=[]
                idx=ashape.geo.idx
#                print "idx=",idx
                ii=0
                polyline=[]
                
                while ii < len(idx):
                    if idx[ii]!=-1:
                        polyline.append(idx[ii])
                    else:
                        if len(polyline)!=3:
                            warnings.warn("oops not a triangle, n=%d. The program only support triangle mesh for the moment"\
                                              %len(polyline))
                        else:
                            tri_lists[i][j]+=polyline
                        polyline=[]
                    ii+=1                    
                        
#                print "creating list %d for mesh %d (%s) with %d vertices and %d faces"%\
                    (j,i,mesh.name,npoints,len(coord)/3)
                self.vertex_lists[i][j]=pyglet.graphics.vertex_list_indexed\
                    (npoints,tri_lists[i][j],('v3f', coord))

            glPushMatrix()
#            listIndex=0
#            while listIndex==0:
#                listIndex=glGenLists(1)
            listIndex=i+1
            self.meshId2glListId[i]=listIndex

            glNewList(listIndex,GL_COMPILE)
            for j in range(len(self.vertex_lists[i])):
                prepareMesh(mesh.shapes[j].app)         
                alist=self.vertex_lists[i][j]
                alist.draw(pyglet.gl.GL_TRIANGLES)#,tri_lists[i][j])
            glEndList()
            glPopMatrix()

        self.setRobot(self.simTime)
        # create_the display list
        if self.measureTime:
            print "createVList\t: %3.2fs"%self.stopwatch.toc("vl")

    def initWindow(self):        
        self.w1 = Window(600,100,caption='Text',resizable=False)
        self.w1.on_draw = self.on_draw_text
        self.w1.switch_to()

    # Try and create a window with multisampling (antialiasing)
        try:
            config = Config(sample_buffers=1, samples=4, 
                            depth_size=16, double_buffer=True,)
            self.w2 = Window(900 , 600,resizable=False,\
                                 config=config)
        except pyglet.window.NoSuchConfigException:
            self.w2 = Window(resizable=True)

        self.w2.on_draw = self.on_draw_scene
        self.w2.on_resize = self.on_resize
        self.w2.on_key_press = self.on_key_press
        self.keys = key.KeyStateHandler()
        self.w2.push_handlers(self.keys)

        glsetup()
        self.w2.switch_to()
        pyglet.clock.schedule(self.update)


    def on_draw_text(self):
        if self.state=="LOADING":
            return
        else:
            try:
                self.appTime=time.time()
                if self.appTime - self.lastTime >= 1:
                    self.fps=self.count
                    self.count=0
                    self.lastTime=self.appTime

                glClear(GL_COLOR_BUFFER_BIT)    
                text1=""

                if abs(self.timeFactor)>=1:
                    sp="%dx\t"%int(abs(self.timeFactor))
                else:
                    sp="1/%dx\t"%int(1/abs(self.timeFactor))
                if self.timeFactor > 0:
                    sp+= ">>"
                else:
                    sp+= "<<"
                text2="time=%3.1f fps=%3.0f speed=%s"\
                    %(self.simTime , self.fps,sp)  
                text3="self.state=%s"%self.state
                if self.state=="STOP":
                    text1="Bored, nothing to do"
                elif self.state in ["PLAY","PAUSE"]: 
                    text1="target: (%3.2f,%3.2f)"\
                        %(self.motion.targetx,self.motion.targety)
                else:
                    text1="OOPS, where am I?"

                label1 = pyglet.text.Label(\
                    text1,font_name='Times New Roman',\
                        font_size=24,x=0, y=70 )
                label2 = pyglet.text.Label(\
                    text2,font_name='Times New Roman',\
                        font_size=24,x=0, y=40 )
                label3 = pyglet.text.Label(\
                    text3,font_name='Times New Roman',\
                        font_size=24,x=0, y=10 )
                self.status="%s\n%s\n%s"%(text1,text2,text3)
                label1.draw()    
                label2.draw()    
                label3.draw()
            except Exception,error:
                print error


    def on_draw_scene(self):
        self.w2.clear()
        self.count+=1
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.fps_display.draw()
        glLoadIdentity()
        p=self.camera.position
        f=self.camera.lookat
        u=self.camera.up
        gluLookAt(p[0],p[1],p[2],f[0],f[1],f[2],u[0],u[1],u[2])
        draw_floor()
        if self.simplify or (not self.showMesh):
            glMaterialfv(GL_FRONT_AND_BACK, \
                             GL_AMBIENT_AND_DIFFUSE, COLOR_RED)
            glMaterialfv(GL_FRONT_AND_BACK, \
                             GL_SPECULAR, COLOR_RED)
            self.robot.draw_skeleton()

        else:
            for i in range(self.numMeshes):
                mesh=self.meshes[i]
                Tmatrix=mesh.globalTransformation
                R=Tmatrix[0:3,0:3]
                p=Tmatrix[0:3,3]
                agax=rot2AngleAxis(R)        
                glPushMatrix()
                glTranslatef(p[0],p[1],p[2])
                glRotated(agax[0],agax[1],agax[2],agax[3])
                
                for j in range(len(self.vertex_lists[i])):
                    prepareMesh(mesh.shapes[j].app)         
                    alist=self.vertex_lists[i][j]
                    alist.draw(pyglet.gl.GL_TRIANGLES)

                    ## glList id is eq to meshId
                    ## drawing by list
    #                li=self.meshId2glListId[i]
    #                glCallList(li)
                glPopMatrix()

        glFlush()

        if self.state=="LOADING":
            return
        else:
            try:
                glMaterialfv(GL_FRONT_AND_BACK,\
                             GL_AMBIENT_AND_DIFFUSE, COLOR_GREEN)
                glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, COLOR_GREEN)

                glPushMatrix()
                glTranslatef(self.motion.targetx, self.motion.targety, 0.05)
                glutSolidSphere(0.03,10,10)
                glPopMatrix()
            except Exception,error:
                print error

    def update(self,dt):
        
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
            self.setRobot(self.simTime)


    def setRobot(self,atime):
        if self.motion.posRecord.isEmpty():
            return
        while self.line_index_pos<len(self.motion.posRecord.times)-1\
           and self.motion.posRecord.times[self.line_index_pos+1]<atime:
            self.line_index_pos+=1
        else:
            while self.line_index_pos >0 \
           and self.motion.posRecord.times[self.line_index_pos] > atime:
                self.line_index_pos-=1
        ## the above snipsets move the line_index point to the nearest time to the left
        ## of atime in the pos_times list       
        angles=self.motion.posRecord.coors[self.line_index_pos]
        self.robot.jointAngles(angles)

#         #+ Wst
#         while self.line_index_wst < len(self.motion.wstRecord.times)-1 \
#            and self.motion.wstRecord.times[self.line_index_wst+1]<atime:
#             self.line_index_wst+=1
#         else:
#             while self.line_index_wst >0 \
#            and self.motion.wstRecord.times[self.line_index_wst] > atime:
#                self.line_index_wst-=1

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


        self.robot.update()


    def execute(self,cmd):
        words=cmd.split()
        if words==[]:
            return "Error: Blank msg"
        path=words[0]
        if words[1:]==[]:
            return """Possible command:
   play
   stop
   pause
   speed 2.3 (float)
   
   
"""
        if words[1]=="status":
            return self.status

        elif words[1]=="play":
            if words[2:]:
                id=int(words[2])
                try:
                    self.loadBasename(self.bn_list[id])
                    self.current_bn_id=id
                except:
                    return "something went wrong"

            self.playMovement()
            return ""    

        elif words[1]=="stop":
            self.stopMovement()
            return ""    

        elif words[1]=="pause":
            self.pauseMovement()
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
            return "Unknown command"

#########################################################
################   MAIN PROGRAM    ######################
#########################################################

import getopt,sys
from getopt import getopt
def main():
    debug=False
    mst=False
    smp=False
    VRMLmesh=None
    standalone=False
    try:
        opts,args=getopt(sys.argv[1:],
                         "l:w:sdmho:v", ["load=","wrl=","simplify","debug","help", "measure-time","output=","standalone"])
    except getopt.GetoptError, err:
        # print help information and exit:
        print str(err) # will print something like "option -a not recognized"
 #       usage()
        sys.exit(2)
    output = None
    sequenceFile = None
    verbose = False
    for o, a in opts:
        
        if o == "-v":
            verbose = True
        elif o in ("-h", "--help"):
#            Application().usage()
            sys.exit()
 
        elif o in ("-d", "--debug"):
            debug=True 
        
        elif o in ("-s", "--simplify"):
            smp=True

        elif o in ("-m", "--measure-time"):
            mst=True

        elif o in ("-o", "--output"):
            output = a

        elif o in ("-l", "--load"):
            sequenceFile=a        


        elif o in ("-w", "--wrl"):
            VRMLmesh=a

        elif o in ("--standalone"):
            standalone=True
        else:
            assert False, "unhandled option"
            
    
    app=Application(debug)
    if sequenceFile:
        app.basename=sequenceFile    
    app.VRMLfile=VRMLmesh
    app.measureTime=mst
    app.simplify=smp
    app.init()

    if not standalone:
         
        ##################################
        #      omniORB
        ##################################
        from omniORB import CORBA, PortableServer

        # Import the stubs for the Naming service
        import CosNaming

        # Import the stubs and skeletons
        import RoboViewer, RoboViewer__POA

        # Define an implementation of the Echo interface
        class Request_i (RoboViewer__POA.Request):
            def req(self, mesg):
                print "request %s:", mesg
                return app.execute(mesg)

        # Initialise the ORB
        orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

        # Find the root POA
        poa = orb.resolve_initial_references("RootPOA")

        # Create an instance of Request_i
        ri = Request_i()

        # Create an object reference, and implicitly activate the object
        ro = ri._this()

        # Obtain a reference to the root naming context
        obj         = orb.resolve_initial_references("NameService")
        rootContext = obj._narrow(CosNaming.NamingContext)

        if rootContext is None:
            print "Failed to narrow the root naming context"
            sys.exit(1)

        # Bind a context named "test.my_context" to the root context
        name = [CosNaming.NameComponent("test", "my_context")]

        try:
            testContext = rootContext.bind_new_context(name)
            print "New test context bound"

        except CosNaming.NamingContext.AlreadyBound, ex:
            print "Test context already exists"
            obj = rootContext.resolve(name)
            testContext = obj._narrow(CosNaming.NamingContext)
            if testContext is None:
                print "test.mycontext exists but is not a NamingContext"
                sys.exit(1)

        # Bind the Echo object to the test context
        name = [CosNaming.NameComponent("Request", "Object")]

        try:
            testContext.bind(name, ro)
            print "New Request object bound"

        except CosNaming.NamingContext.AlreadyBound:
            testContext.rebind(name, ro)
            print "Request binding already existed -- rebound"

            # Note that is should be sufficient to just call rebind() without
            # calling bind() first. Some Naming service implementations
            # incorrectly raise NotFound if rebind() is called for an unknown
            # name, so we use the two-stage approach above

        # Activate the POA
        poaManager = poa._get_the_POAManager()
        poaManager.activate()

        # Everything is running now, but if this thread drops out of the end
        # of the file, the process will exit. orb.run() just blocks until the
        # ORB is shut down




    ##################################
    
    app.run()
if __name__=="__main__": main()
