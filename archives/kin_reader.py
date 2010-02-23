#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang

## quick, hacky kinematics reader


import sys, re
import robo


def beautify(s):
#     remove space at the begining and end of a string
    # print "before:",s
    s=re.sub(r"(?<=\A)\s+","",s)
    s=re.sub(r"\r\n","",s)
    # print "after:",s
    return s

def VRMLloader(filename):
    branch1=[-999,0,1,2,3,4,5]
    branches=[]
    branches.append([-999,0,1,2,3,4,5])
    branches.append([-999,6,7,8,9,10,11])
    branches.append([-999,12,13,14,15])
    branches.append([13,16,17,18,19,20,21,22,30,31,32,33,34])
    branches.append([13,23,24,25,26,27,28,29,35,36,37,38,39])



    lines=open(filename,'r').readlines()
    kine_lines=[]
    
    keywords=["DEF \w+ Joint","JointType","jointId","jointAxis"\
          ,"translation","rotation \d \d \d"
          ]
    pattern=""
    for key in keywords:
        if pattern=="":
            pattern="\A\s*"+key
        else:
            pattern=pattern+"|\A\s*"+key
    expression = re.compile(r"%s"%pattern)
    kine_groups=[]
    kine_group=""
    for line in lines:
        if expression.search(line):
            line=beautify(line)
            kine_lines.append(line)
            if re.search(r"\ADEF \w+ Joint",line):
                kine_group=""
            kine_group+="\n"+line
        
            if re.search(r"rotation",line):
                kine_groups.append(kine_group)
                kine_group=""


    new_groups=[]
    for group in kine_groups:
        if re.search(r"jointId \d+",group):
            new_groups.append(group)


    list_joints=[]
    base=robo.baseNode()
    list_joints.append(base)

    for group in new_groups:    
        m=re.search(r"jointId (\d+)",group)
    #    print group
        if m: 
            id=int(m.group(1))
        m=re.search(r"translation ([\-\d\.]+) ([\-\d\.]+) ([\-\d\.]+)",group)    
        if m: 
            translation=[float(m.group(1)),float(m.group(2))\
                             ,float(m.group(3))]
    
        m=re.search(r"rotation ([\-\d\.]+) ([\-\d\.]+) ([\-\d\.]+) ([\-\d\.]+)",group)
        if m:
            rotation=[float(m.group(1)),float(m.group(2))\
                     ,float(m.group(3)), float(m.group(4))]
        m=re.search(r"jointAxis \"(\w)\"",group)
        if m:
            axis=m.group(1)

        if rotation and translation and axis:
            new_joint=robo.joint(id,translation,rotation,axis)
            new_joint.jointType="rotate"
            list_joints.append(new_joint)


    for joint in list_joints:
        base.joint_dict[joint.id]=joint

    for branch in branches:
        for i in range(len(branch)-1):
            joint_parent=base.joint_dict[branch[i]]
            joint_child=base.joint_dict[branch[i+1]]
            joint_parent.addChild(joint_child)
            joint_child.setParent(joint_parent)


    base.joint_list=[]
    from collections import deque
    pile=deque()
    pile.append(base)


    while not len(pile)==0:
        ajoint=pile.pop()
        base.joint_list.append(ajoint)
        for child in reversed(ajoint.children):
            pile.append(child)            
    base.waist=base    
    base.jointType="free"
    base.init()
    return base

import numpy as np
def main():
    base=VRMLloader(sys.argv[1])
    angles=0.5*np.ones((40,1))
    base.jointAngles(angles)
    for joint in base.joint_list:
        print joint
        print "\n\n=============================="


if __name__ == "__main__":
    main()
