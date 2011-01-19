# -*- coding: utf-8 -*-

# Copyright 2010 CNRS
# Author: Florent Lamiraux, Duong Dang
#
# Release under LGPL license: see COPYING.LESSER at root of the project.
#

import sys
import xml.dom.minidom as dom
import kinematic_chain
# from dynamic_graph.sot.dynamics.dynamic import Dynamic
# from dynamic_graph.sot import SE3, R3, SO3
import numpy
import numpy.linalg
import vrml_parser
from mathaux import *
import os, logging
import ml_parser
import copy

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("kxml_parser")
logger.addHandler(NullHandler())

class Parser (object):
    """
    Adapt from parser in sot-dynamic project https://github.com/jrl-umi3218/sot-dynamic
    Parser to build kinematic_chain.BaseNode entities.

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

    relPathTag = "RELATIVE_FILENAME"
    solidrefTag = "SOLID_REF"

    assemblyTag = "ASSEMBLY"
    polyhedronTag = "POLYHEDRON"
    relPosTag =  "RELATIVE_POSITION"
    motionFrameTag = "MOTION_FRAME"
    diffuseColorTag = "GEOMETRY_DIFFUSE_COLOR"
    specularColorTag = "GEOMETRY_SPECULAR_COLOR"
    ambientColorTag = "GEOMETRY_AMBIENT_COLOR"
    shininessTag = "GEOMETRY_SHININESS"
    nameTag      = "NAME"

    def __init__(self, robot_name, filename):
        self.robot_name = robot_name
        self.filename = filename
        self.kxml_dir_name = os.path.abspath(os.path.dirname(filename))
        self.shapes = {}
        self.globalTransformations = {}

    def parse_geometry(self, dom1):
        assembly_nodes = dom1.getElementsByTagName(self.assemblyTag)
        for assembly_node in assembly_nodes:
            assembly_nid = int(assembly_node.attributes["id"].value)
            assembly_obj = kinematic_chain.GenericObject()
            assembly_obj.id = assembly_nid
            self.shapes[assembly_nid] = assembly_obj
            for rel_pos_node in assembly_node.childNodes:
                if rel_pos_node.nodeName != self.motionFrameTag:
                    continue
                try:
                    data =  rel_pos_node.childNodes[0].nodeValue.split()
                    data = [ float(e) for e in data ]
                except:
                    raise Exception("Invalid position node %s"%rel_pos_node.toprettyxml())

                if len(data) != 16:
                    raise Exception("Wrong dimension for position %s"%str(data))

                rel_pos = numpy.array([[data[0],  data[1],  data[2],  data[3]],
                                   [data[4],  data[5],  data[6],  data[7]],
                                   [data[8],  data[9],  data[10], data[11]],
                                   [data[12], data[13], data[14], data[15]]
                                   ]
                                  )

                #assembly_obj.translation = rel_pos[0:3,3]
                #assembly_obj.rotation = rot2AxisAngle(rel_pos[0:3,0:3])

            polyhedron_nodes = assembly_node.getElementsByTagName(self.polyhedronTag)

            for polyhedron_node in polyhedron_nodes:
                polyhedron_nid = int(polyhedron_node.attributes["id"].value)
                polyhedron_obj = kinematic_chain.GenericObject()
                polyhedron_obj.id = polyhedron_nid
                polyhedron_obj.name = self.findStringProperty(polyhedron_node, self.nameTag)
                assembly_obj.addChild(polyhedron_obj)
                self.shapes[polyhedron_nid] = polyhedron_obj

                for child_node in polyhedron_node.childNodes:
                    if child_node.nodeName == self.motionFrameTag:
                        rel_pos_node = child_node
                        try:
                            data =  rel_pos_node.childNodes[0].nodeValue.split()
                            data = [ float(e) for e in data ]
                        except:
                            raise Exception("Invalid position node %s"%rel_pos_node.toprettyxml())

                        if len(data) != 16:
                            raise Exception("Wrong dimension for position %s"%str(data))

                        rel_pos = numpy.array([[data[0],  data[1],  data[2],  data[3]],
                                           [data[4],  data[5],  data[6],  data[7]],
                                           [data[8],  data[9],  data[10], data[11]],
                                           [data[12], data[13], data[14], data[15]]
                                           ]
                                          )
                        #polyhedron_obj.translation = rel_pos[0:3,3]
                        #polyhedron_obj.rotation = rot2AxisAngle(rel_pos[0:3,0:3])


                    elif child_node.nodeName == self.relPathTag:
                        rel_path_node = child_node
                        rel_path = rel_path_node.childNodes[0].nodeValue
                        logger.info("Parsing %s "%(os.path.join(self.kxml_dir_name, rel_path)))
                        objs = ml_parser.parse(os.path.join(self.kxml_dir_name, rel_path))
                        for obj in objs:
                            if isinstance(obj,kinematic_chain.GenericObject):
                                polyhedron_obj.addChild(obj)
                                # obj.translation = [0,0,0]
                            else:
                                logger.debug("Ignoring %s"%str(obj))

                diffuseColor  = self.findVecProperty(polyhedron_node,self.diffuseColorTag)[:-1]
                specularColor = self.findVecProperty(polyhedron_node,self.specularColorTag)[:-1]
                ambientColor  = self.findVecProperty(polyhedron_node,self.ambientColorTag)[:-1]
                shininess     = self.findFloatProperty(polyhedron_node,self.shininessTag)
                def propagate_geo_param(obj, key, value):
                    if isinstance(obj, kinematic_chain.Mesh):
                        old_val =  getattr(obj.app, key)
                        if old_val != value:
                            logger.debug("Mesh %s: %s changed from %s to %s"
                                         %(polyhedron_obj.name, key, str(old_val), str(value)))
                        setattr(obj.app, key, value)

                    for child in obj.children:
                        propagate_geo_param(child, key, value)

                for key,value in [#("diffuseColor",diffuseColor),
                                  ("specularColor",specularColor),
                                  ("ambientColor",ambientColor),
                                  ("shininess",shininess),
                                  ]:
                    propagate_geo_param(polyhedron_obj, key, value)

        solidref_nodes = dom1.getElementsByTagName(self.solidrefTag)

        for n in solidref_nodes:
            nid = int(n.attributes["id"].value)
            refid = int(n.attributes["referencedComponentId"].value)
            self.shapes[nid] = self.shapes[refid]

    def parse (self):
        dom1 = dom.parse(self.filename)
        self.parse_geometry(dom1)

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
        robots = []
        for rootJointNode in self.findRootJoints(hNode):
            robot = self.createJoint(rootJointNode, parent = None)
            robots.append(robot)
        return robots

    def compute_localT_from_globalT_(self,joint_):
        if joint_.parent:
             joint_.localTransformation = numpy.dot( numpy.linalg.inv(joint_.parent.globalTransformation),
                                                    joint_.globalTransformation)
             # print joint_.id, joint_.localTransformation
             joint_.translation = joint_.localTransformation[0:3,3]
             joint_.localR = joint_.localTransformation[0:3,0:3]

             joint_.localR2 = axisNameAngle2rot(joint_.axis,joint_.angle)
             joint_.localR1 = numpy.dot(joint_.localR,
                                        numpy.linalg.inv(joint_.localR2))
             joint_.rotation = rot2AxisAngle(joint_.localR1)
             # print joint_.name
             # print joint_.localR1
             # print joint_.rotation
             # print joint_.globalTransformation
             # print joint_.parent.globalTransformation
             # print "---"
             # joint_.initLocalTransformation()
             # print joint_.id, joint_.rotation, joint_.localTransformation


        for child in [ c for c in joint_.children if isinstance(c,kinematic_chain.Joint)]:
            self.compute_localT_from_globalT_(child)



    def createJoint (self, node, parent = None):
        if node.nodeName == "HPP_FREEFLYER_JOINT":
            joint = kinematic_chain.BaseNode()
        else:
            joint = kinematic_chain.Joint()
            joint.id = int(node.attributes["id"].value)
            if not parent:
                raise Exception("Expected a parent for node %s"%node.nodeName)
            parent.addChild(joint)


        sotJointType = self.jointType[node.nodeName]
        jointName = self.findStringProperty(node, 'NAME')
        joint.name = jointName
        joint.jointType = sotJointType
        if joint.jointType == "rotation":
            joint.angle = self.findJointValue(node)
            joint.axis = "X"

        current_position , relative_solid_position, solid_id = self.findJointPositions(node)
        joint.globalTransformation = copy.copy(current_position)
        self.globalTransformations[joint.id] = copy.copy(current_position)

        # recursively create child joints
        childJointNodes = filter(lambda n:n.nodeName in self.jointTypes,
                             node.childNodes)

        for childJointNode in childJointNodes:
            childJoint = self.createJoint(childJointNode, joint)

        # add shape object already loaded at the beginning
        solid = kinematic_chain.GenericObject()
        # solid.translation = relative_solid_position[0:3,3]
        solid.rotation    = rot2AxisAngle(relative_solid_position[0:3,0:3])
        solid.addChild(self.shapes[solid_id])
        joint.addChild(solid)

        if isinstance(joint, kinematic_chain.BaseNode):
            self.compute_localT_from_globalT_(joint)
            joint.init()
            for i,j in enumerate(joint.joint_list):
                if (numpy.linalg.norm(j.globalTransformation -
                                      self.globalTransformations[j.id])) > 1e-6:
                    msg = ("""
Wrong transformation for %dth joint (id = %d)%s:
computed transformation=
%s
vs.
kxml transformation=
%s

joint.localT=
%s
"""
                                    %(i, j.id, j.name, str(j.globalTransformation),
                                      str(self.globalTransformations[j.id]),
                                      str(j.localTransformation),
                                      ))
                    msg += "\nParent joints:"
                    jj = j
                    while jj:
                        msg += "\n---\n"
                        msg + str(jj)
                        msg += "\nkxml global pos = %s"%str(self.globalTransformations[jj.id])
                        jj = jj.getParentJoint()
                    raise Exception(msg)
                    # logger.exception(msg)
        return joint

    def findJointPositions(self, node):
        posNodes = filter(lambda n: n.nodeName in ['CURRENT_POSITION',
                                                   'RELATIVE_SOLID_POSITION'],
                                                   node.childNodes)
        current_position = None
        relative_solid_position = None
        solid_id = None
        for n in posNodes:
            try:
                data =  n.childNodes[0].nodeValue.split()
                data = [ float(e) for e in data ]
            except:
                raise Exception("Invalid position node %s"%n.toprettyxml())

            if len(data) != 16:
                raise Exception("Wrong dimension for position %s"%str(data))

            pos = numpy.array([[data[0],  data[1],  data[2],  data[3]],
                               [data[4],  data[5],  data[6],  data[7]],
                               [data[8],  data[9],  data[10], data[11]],
                               [data[12], data[13], data[14], data[15]]
                               ]
                              )
            if n.nodeName == "CURRENT_POSITION":
                current_position = pos
            else:
                solid_id = int(n.attributes["solidRefId"].value)
                relative_solid_position = pos
        return ( current_position , relative_solid_position, solid_id )


    def attachJointToParent(self, parentName, jointName):
        if parentName == self.robotTag:
            self.robot.setRootJoint(jointName)
        else:
            self.robot.addJoint(parentName, jointName)

    def findRootJoints (self, hNode):
        rJoint = filter(lambda n:n.nodeName in self.jointTypes,
                        hNode.childNodes)
        if len(rJoint) == 0:
            raise RuntimeError("Robot should have at least one joint.")
        if len(rJoint) > 1:
            raise RuntimeError("Robot should have exactly one root joint.\n" +
                               "This one has " + str(len(rJoint)) + ".")
        return rJoint

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

    def findJointValue(self, node):
        dofList = filter(lambda n: n.nodeName == 'DOF', node.childNodes)
        value = 0
        for dof in dofList:
            value = self.findFloatProperty(dof, 'DOF_VALUE')
        return value


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

    def findVecProperty(self, node, prop):
        s = self.findStringProperty(node, prop)
        return [ float(w) for w in s.split()]


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


def parse(filename):
    parser = Parser("nao",filename)
    return parser.parse()

if __name__ == '__main__':
    sh = logging.StreamHandler()
    sh.setLevel(logging.DEBUG)
    logger.addHandler(sh)
    robot = parse(sys.argv[1])[0]
    print robot
    # print robot.printJoints()
