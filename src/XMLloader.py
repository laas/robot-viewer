#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang

import xml.etree.ElementTree as ET
import robo,sys,re
import meshLoader
xmlFilePath="./"

def XMLloader(filename):
    global xmlFilePath
    xmlFilePath=re.sub(r"[\w_\-\d]+\.xml","",filename)
    s=open(filename,'r').read()
    root = ET.fromstring(s)
    robots=[]
    for element in root.getchildren():
        if element.tag=="BaseNode":
            robot=parseBaseNode(element)
            robots.append(robot)
#    robots[0].printJoints()
    return robots[0]

def parseBaseNode(element):
    global xmlFilePath    
    robot=robo.BaseNode()
    robot.jointType="free"
    imageRot=[1,0,0,0]
    imagePos=[0,0,0]

    for child in element.getchildren():


        if child.tag=="robotName":
            robot.name=child.text

        if child.tag=="pos":
            s=child.text
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")
                                   
            m=pattern.match(s)
            if m:
                robot.translation=[float(m.group(1)),float(m.group(2)),float(m.group(3))]

        if child.tag=="rot":
            s=child.text
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")
            m=pattern.match(s)
            if m:
                robot.rotation=[float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))]

                
        if child.tag=="jointNode":
            aJoint=parseJoint(child)
            robot.addChild(aJoint)
            aJoint.setParent(robot)        

        if child.tag=="imagePos":
            s=child.text
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")                                 
            m=pattern.match(s)
            if m:
                imagePos=[float(m.group(1)),float(m.group(2)),float(m.group(3))]


        if child.tag=="imageRot":
            s=child.text
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")
            m=pattern.match(s)
            if m:
                imageRot=[float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))]
        
        if child.tag=="url":            
            meshFile=child.text
            amesh=meshLoader.meshLoader(xmlFilePath+meshFile)
            amesh.position=imagePos
            amesh.rotation=imageRot
            robot.addChild(amesh)
            amesh.setParent(robot)        

    robot.init()
    return robot

def parseJoint(element):
    global xmlFilePath    
    joint=robo.joint()
    imageRot=[1,0,0,0]
    imagePos=[0,0,0]
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
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")                                 
            m=pattern.match(s)
            if m:
                joint.translation=[float(m.group(1)),float(m.group(2)),float(m.group(3))]

        if child.tag=="imagePos":
            s=child.text
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")                                 
            m=pattern.match(s)
            if m:
                imagePos=[float(m.group(1)),float(m.group(2)),float(m.group(3))]
        if child.tag=="rot":
            s=child.text
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")
            m=pattern.match(s)
            if m:
                joint.rotation=[float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))]

        if child.tag=="imageRot":
            s=child.text
            pattern=re.compile("\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*,\s*([-+\d\.]+)\s*")
            m=pattern.match(s)
            if m:
                imageRot=[float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))]

        if child.tag=="url":            
            meshFile=child.text
            amesh=meshLoader.meshLoader(xmlFilePath+meshFile)
            amesh.translation=imagePos
            amesh.rotation=imageRot
            joint.addChild(amesh)
            amesh.setParent(joint)        
            print amesh


        if child.tag=="jointNode":
            childJoint=parseJoint(child)
            joint.addChild(childJoint)
            childJoint.setParent(joint)        
            
    return joint
    

def main():
    robots=XMLloader(sys.argv[1])
    for robot in robots:
        print robot
#        robot.printJoints()
        pass
if __name__=="__main__":
    main()
