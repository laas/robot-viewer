#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang

import xml.etree.ElementTree as ET
import robo,sys,re
xmlFilePath="./"

def XMLloader(filename):
    s=open(filename,'r').read()
    root = ET.fromstring(s)
    robots=[]
    for element in root.getchildren():
        if element.tag=="baseNode":
            robot=parseBaseNode(element)
            robots.append(robot)
    robots[0].printJoints()
    return robots[0]

floatPattern="[\d\.]+"

def parseBaseNode(element):
    robot=robo.baseNode()
    robot.jointType="free"
    for child in element.getchildren():
        if child.tag=="robotName":
            robot.name=child.text

        if child.tag=="pos":
            s=child.text
            print s
            pattern=re.compile(r"^\s*(%s)\s*\,(%s)\s*\,(%s)\s*$"\
                                   %(floatPattern,floatPattern,floatPattern))
            m=pattern.match(s)
            if m:
                robot.translation=[float(m.group(1)),float(m.group(2)),float(m.group(3))]

        if child.tag=="rot":
            s=child.text
            print s
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")
            m=pattern.match(s)
            if m:
                robot.rotation=[float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))]


                
        if child.tag=="jointNode":
            aJoint=parseJoint(child)
            robot.addChild(aJoint)
            aJoint.setParent(robot)        
    robot.init()
    return robot

def parseJoint(element):
    joint=robo.joint()
    for child in element.getchildren():
        
        if child.tag=="jointName":
            joint.name=child.text

        if child.tag=="ID":
            joint.id=int(child.text)

        if child.tag=="type":
            if child.text in ("r","R"):
                joint.jointType="rotate"
            else:
                joint.jointType="free"

        if child.tag=="axis":
            joint.axis=child.text

        if child.tag=="pos":
            s=child.text
            print s
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")                                 
            m=pattern.match(s)
            if m:
                joint.translation=[float(m.group(1)),float(m.group(2)),float(m.group(3))]

        if child.tag=="rot":
            s=child.text
            print s
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")
            m=pattern.match(s)
            if m:
                joint.rotation=[float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))]



        if child.tag=="jointNode":
            childJoint=parseJoint(child)
            joint.addChild(childJoint)
            childJoint.setParent(joint)        
    
    return joint
    

def main():
    robots=XMLloader(sys.argv[1])
    for robot in robots:
#        print robot
        robot.printJoints()
        pass
if __name__=="__main__":
    main()
