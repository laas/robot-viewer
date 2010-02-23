#! /usr/bin/env python
import os,sys,time
argc=len(sys.argv)
if argc <5 or argc >6:
    print "Usage: launch-script targetx targety numsteps output server[optional]"
    sys.exit(1)
tic=time.time()

targetx=(float)(sys.argv[1])
targety=(float)(sys.argv[2])
numsteps=(int)(sys.argv[3])
output=sys.argv[4]
if argc >=6:
    server=sys.argv[5]
else:
    server="cleo"
cmd="""ssh -l nddang %s.laas.fr "cd /home/nddang/softs/devel/hpplocalstepper/build-%s/tests/; ./simple-test %s %s %s %s && mv seq-%s* ~/var/seq/  && echo '%f %f %d' > ~/var/seq/seq-%s-wbm.target"
"""%(server,server,sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4],output,targetx,targety,numsteps,sys.argv[4])

os.system(cmd)

toc=time.time()
print "Total time: (plan+step)=",toc-tic

