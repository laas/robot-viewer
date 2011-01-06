# -*- coding: utf-8 -*-

# Copyright 2010 CNRS
# Author: Florent Lamiraux, Duong Dang
#
# Release under LGPL license: see COPYING.LESSER at root of the project.
#

import sys
import xml.dom.minidom as dom
import robo
# from dynamic_graph.sot.dynamics.dynamic import Dynamic
# from dynamic_graph.sot import SE3, R3, SO3

class Parser (object):
    """
    Adapt from parser in sot-dynamic project https://github.com/jrl-umi3218/sot-dynamic
    Parser to build robo.BaseNode entities.

    Format is kxml, Kineo CAM robot description format.
    """
    robotFloatProperties = ['GAZEORIGINX', 'GAZEORIGINY', 'GAZEORIGINZ',
                            'GAZEDIRECTIONX',
                            'GAZEDIRECTIONY',
                            'GAZEDIRECTIONZ',
                            'ANKLEPOSINLEFTFOOTFRAMEX',
                            'ANKLEPOSINLEFTFOOTFRAMEY',
                            'ANKLEPOSINLEFTFOOTFRAMEZ',
                            'SOLECENTERINLEFTFOOTFRAMEX',
                            'SOLECENTERINLEFTFOOTFRAMEY',
                            'SOLECENTERINLEFTFOOTFRAMEZ',
                            'SOLELENGTH', 'SOLEWIDTH',
                            'LEFTHANDCENTERX',
                            'LEFTHANDCENTERY',
                            'LEFTHANDCENTERZ',
                            'LEFTTHUMBAXISX',
                            'LEFTTHUMBAXISY',
                            'LEFTTHUMBAXISZ',
                            'LEFTFOREFINGERAXISX',
                            'LEFTFOREFINGERAXISY',
                            'LEFTFOREFINGERAXISZ',
                            'LEFTPALMNORMALX',
                            'LEFTPALMNORMALY',
                            'LEFTPALMNORMALZ']

    robotStringProperties = ['WAIST', 'CHEST', 'LEFTWRIST',
                             'RIGHTWRIST', 'LEFTANKLE', 'RIGHTANKLE', 'GAZE']

    jointFloatProperties = ['MASS', 'COM_X', 'COM_Y', 'COM_Z',
                            'INERTIA_MATRIX_XX', 'INERTIA_MATRIX_YY',
                            'INERTIA_MATRIX_ZZ', 'INERTIA_MATRIX_XY',
                            'INERTIA_MATRIX_XZ', 'INERTIA_MATRIX_YZ']

    jointTypes = ['HPP_FREEFLYER_JOINT', 'HPP_ROTATION_JOINT',
                  'HPP_TRANSLATION_JOINT', 'HPP_ANCHOR_JOINT']

    jointType = {'HPP_FREEFLYER_JOINT':'freeflyer',
                 'HPP_ROTATION_JOINT':'rotation',
                 'HPP_TRANSLATION_JOINT':'translation',
                 'HPP_ANCHOR_JOINT':'anchor'}

    robotTag = "HPP_HUMANOID_ROBOT"

    def __init__(self, robot_name, filename):
        self.robot_name = robot_name
        self.filename = filename

    def parse (self):
        dom1 = dom.parse(self.filename)
        hNode = dom1.getElementsByTagName(self.robotTag)[0]
        for p in self.robotStringProperties:
            value = self.findStringProperty(hNode, p)
            # print p,value
            setattr(self, p, value)

        for p in self.robotFloatProperties:
            value = self.findFloatProperty(hNode, p)
            # print p,value
            setattr(self, p, value)

        # self.robot.createRobot()
        rootJointNode = self.findRootJoint(hNode)
        self.robot = self.createJoint(rootJointNode, parent = None)
        self.robot.init()
        return self.robot

    def createJoint (self, node, parent = None):
        if not parent:
            joint = robo.BaseNode()
        else:
            joint = robo.Joint()
            parent.children.append(joint)
            joint.parent = parent

        sotJointType = self.jointType[node.nodeName]
        jointName = self.findStringProperty(node, 'NAME')
        joint.name = jointName
        # recursively create child joints
        childJointNodes = filter(lambda n:n.nodeName in self.jointTypes,
                             node.childNodes)

        for childJointNode in childJointNodes:
            childJoint = self.createJoint(childJointNode, joint)

        return joint


    def attachJointToParent(self, parentName, jointName):
        if parentName == self.robotTag:
            self.robot.setRootJoint(jointName)
        else:
            self.robot.addJoint(parentName, jointName)

    def findRootJoint (self, hNode):
        rJoint = filter(lambda n:n.nodeName in self.jointTypes,
                        hNode.childNodes)
        if len(rJoint) == 0:
            raise RuntimeError("Robot should have at least one joint.")
        if len(rJoint) > 1:
            raise RuntimeError("Robot should have exactly one root joint.\n" +
                               "This one has " + str(len(rJoint)) + ".")
        return rJoint[0]

    def findJointBounds(self, node, jointName):
        dofList = filter(lambda n: n.nodeName == 'DOF', node.childNodes)
        bounds = []
        for dof in dofList:
            dofName = self.findStringProperty(dof, 'NAME')
            minValue = -1e-6
            maxValue = 1e-6
            try:
                minValue = self.findFloatProperty(dof, 'DOF_MIN_VALUE')
            except RunTimeError:
                print ("min value of dof %s of joint %s is not specified." %
                       (dofName, jointName))
                print ("Set to -1e-6")
            try:
                maxValue = self.findFloatProperty(dof, 'DOF_MAX_VALUE')
            except RunTimeError:
                print ("max value of dof %s of joint %s is not specified." %
                       (dofName, jointName))
                print ("Set to 1e-6")

            bounds.append((minValue, maxValue))
        return bounds

    def findStringProperty (self, node, prop):
        return self.findProperty(node, prop, str)

    def findFloatProperty (self, node, prop):
        return self.findProperty(node, prop, float)

    def findIntProperty (self, node, prop):
        return self.findProperty(node, prop, int)

    def findProperty(self, node, prop, cast):
        properties = filter(lambda n: n.nodeName == 'PROPERTY', node.childNodes)
        theProperty = filter (lambda p: p._attrs['stringId'].nodeValue == prop,
                        properties)
        if len(theProperty) != 1:
            raise RuntimeError(prop +
                               ' should be specified once and only once.')
        theProperty = theProperty[0]
        value = filter(lambda n:n.nodeType == n.TEXT_NODE,
                       theProperty.childNodes)
        if len(value) != 1:
            raise RuntimeError('One and only one name should be specified for '
                               + prop)
        value = value[0]
        return cast(value.data)

    def findJointPosition(self, node):
        tag = 'CURRENT_POSITION'
        posNode = filter(lambda n: n.nodeName == tag,
                         node.childNodes)
        if len(posNode) == 0:
            print ("Position of joint not specified: tag " + tag + ",")
            print ("Setting to identity")
            return ((1.,0.,0.,0.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.))

        if len(posNode) > 1:
            raise RuntimeError("'CURRENT_POSITION' specified more than once")

        posNode = posNode[0]
        if len(posNode.childNodes) != 1 and (posNode.childNodes[0].typeNode ==
                                             posNode.TEXT_NODE):
            raise RunTimeError("Position matrix ill defined")

        # Remove spurious characters at beginning and end of matrix string
        data = posNode.childNodes[0].data.strip('\n\t ')
        # Split by lines and remove spurious characters at begginning and end of
        # each line.
        lines = map (lambda l: l.strip('\t '), data.split('\n'))
        # Split each line between spaces
        matrix = map (lambda l: l.split(' '), lines)
        # cast each matrix element into float
        matrix = map (lambda l: map (float, l), matrix)
        # transform list into tuple
        return tuple (map (tuple, matrix))

    def handSymmetry(self, vector):
        """
        Conversion of local coordinates from left hand to right hand

          Input:
            - a vector: locally expressed in the local frame of left wrist,
          Return:
            - a vector: locally expressed in the local frame of the right wrist.

          The conversion is done in such a way that input and output are
          symmetric with respect to plane (xz) in global frame.
        """
        # input vector expressed in global frame
        vector = R3(vector)
        globalLeftVector = SE3(self.leftWristPosition) * vector
        globalRightVector = R3(globalLeftVector)
        globalRightVector = R3(globalRightVector[0],
                               -globalRightVector[1],
                               globalRightVector[2])
        output = SE3(self.rightWristPosition).inverse()*globalRightVector
        return tuple(output)


def load(filename):
    parser = Parser("nao",filename)
    return parser.parse()

if __name__ == '__main__':
    print load(sys.argv[1])
