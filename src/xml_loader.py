# Copyright (c) 2010-2011, Duong Dang <mailto:dang.duong@gmail.com>
# This file is part of robot-viewer.

# robot-viewer is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# robot-viewer is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with robot-viewer.  If not, see <http://www.gnu.org/licenses/>.
#! /usr/bin/env python


import xml.etree.ElementTree as ET
import kinematic_chain,sys,re
import meshLoader
xmlFilePath="./"

def load(filename):
    global xmlFilePath
    xmlFilePath=re.sub(r"[\w_\-\d]+\.xml","",filename)
    s=open(filename,'r').read()
    root = ET.fromstring(s)
    robots=[]
    for element in root.getchildren():
        if element.tag=="Robot":
            robot=parseRobot(element)
            robots.append(robot)
#    robots[0].printJoints()
    return robots[0]

def parseRobot(element):
    global xmlFilePath
    robot=kinematic_chain.Robot()
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
    joint=kinematic_chain.joint()
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
    robots=load(sys.argv[1])
    for robot in robots:
        print robot
#        robot.printJoints()
        pass
if __name__=="__main__":
    main()
