#! /usr/bin/env python
## lauch remote plan to generate motions satisfying certain criteria

# planning with goals anywhere in the 2mx2m area around the robot
# grid of 10cm (400 points in total)

# a test record is a list of 4 number
# x,y,n,status (1:done, 0: notdone)
import pickle,random,os,sys
L = 1
d = 0.1
N=int(L/d)
TestQueue=[]
import math
for i in range(N+1):
    for j in range (N+1):
        x=d*i
        y=d*j
        r=math.sqrt(x*x+y*y)
        # hyteristically: numstep ~= r/0.3 +-1
        nsteps=int(round(r/0.3))
        for nst in range(max(nsteps,2),max(nsteps+1,2)):
            if nst <=3:
                TestQueue.append([x,y,nst,0])
#sys.exit(0)
NN=len(TestQueue)

rlist=[i for i in range(NN)]
#random.shuffle(rlist)

for i in rlist:
    test=TestQueue[i]
    print test
    x=test[0]
    y=test[1]
    nsteps=test[2]
    print nsteps
    os.system("python remotePlan.py %1.1f %1.1f %d walk_%1.1f_%1.1f_%d"\
                  %(x,y,nsteps,x,y,nsteps))
    TestQueue[i][3]=1
    f=open("testRecord.pickle","w")
#    pickle.dump(rlist,f)
#    pickle.dump(i,f)
    pickle.dump(TestQueue,f)
    f.close()
