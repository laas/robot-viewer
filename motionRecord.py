
#--------------------------------------
# CLASS Record
# A Record contain a list of lines
# Each line is a list of numbers (a least 2), the first number
#            is timestamp, the rest is coordinates
defaultBn="data/seq-cleo0.3x-wbm"
class Record():
    def __init__(self):
        self.times=[]
        self.coors=[]
        self.ncoor=0

    def isEmpty(self):
        if len(self.coors)==0:
            return True
        else:
            return False

    def __str__(self):
        s=""
        s+="Record has %.2d lines ;"%len(self.times)
        s+="each coor lines contains %d elements\n"%self.ncoor
        return s

    def addALine(self,line):
        n=len(line)-1
        if n < 1:
            raise Exception\
                ("A valid line must contain time and values")
        if self.isEmpty():
            self.ncoor=n
        elif  n!=self.ncoor:
            raise Exception("New line has invalid dimension %d"%n)

        self.times.append(line[0])
        self.coors.append(line[1:])
    
    def addLines(self,lines):
        if lines==[]:
            return
        firstLine=lines[0]
        self.addAPos(firstLine)
        ## recursively add the rest
        self.addPosLines(lines[1:])
        

    def loadFile(self,afile):
        lines=open(afile,'r').readlines()
        for line in lines:
            words=line.split()
            numbers=[float(word) for word in words]
            self.addALine(numbers)
 
    def getTimeMax(self):
        return max(self.times)
 
    def getTimeMin(self):
        return min(self.times)

#--------------------------------------

class Motion():
    def __init__(self):
        self.basename=None
        self.posRecord=Record()
        self.wstRecord=Record()
        self.rpyRecord=Record()

        self.targetx=None
        self.targety=None
        self.numsteps=None
        
    def getInfo(self):
        return ("%1.2f\t%1.2f\t%d \t%s")%\
            (self.targetx,self.targety,self.numsteps,self.basename)

    def __str__(self):
        s=""
        s+="posRecord :%s"%self.posRecord.__str__()
        s+="wstRecord :%s"%self.wstRecord.__str__()
        s+="rpyRecord :%s"%self.rpyRecord.__str__()
        s+="\nTarget : %3.2f %3.2f \t\tNo of steps: %d"%(self.targetx,self.targety,self.numsteps)
        s+="\nMax simTime: %3.2f"%self.getTimeMax()
        return s

    def loadInfo(self,infoFile):
        st=open(infoFile,"r").read()
        words=st.split()
        self.targetx=float(words[0])
        self.targety=float(words[1])
        self.numsteps=int(words[2])
        return self.getInfo()

    def loadBasename(self,bn):
        self.__init__()
        self.basename=bn
        self.posRecord.loadFile("%s.pos"%bn)
        self.wstRecord.loadFile("%s.wst"%bn)
        self.rpyRecord.loadFile("%s.rpy"%bn)
        self.loadInfo("%s.target"%bn)
#        print '''loaded pos (%d lines), wst (%d lines) 
#    and rpy (%d lines)''' %(len(self.poslines),len(wstlines),len(rpylines))

    def getTimeMax(self):
        return min( self.posRecord.getTimeMax(),\
                        self.wstRecord.getTimeMax(),\
                        self.rpyRecord.getTimeMax()\
                        )


    def getTimeMin(self):
        return max( self.posRecord.getTimeMin(),\
                        self.wstRecord.getTimeMin(),\
                        self.rpyRecord.getTimeMin()\
                        )



if __name__=="__main__":
    motion=Motion()
    motion.loadBasename(defaultBn)
    print motion
