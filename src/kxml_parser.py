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
# -*- coding: utf-8 -*-

# Copyright 2010 CNRS
# Author: Florent Lamiraux, Duong Dang
#
# Release under LGPL license: see COPYING.LESSER at root of the project.
#

import sys
import xml.dom.minidom as dom
import kinematics
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

logger = logging.getLogger("robotviewer.kxml_parser")
logger.addHandler(NullHandler())

class Parser (object):
    """
    Adapt from parser in sot-dynamic project https://github.com/jrl-umi3218/sot-dynamic
    Parser to build kinematics.Robot entities.

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
        self.shape_types = {}
        self.globalTransformations = {}


    def parse_geometry(self, dom1):
        assembly_nodes = dom1.getElementsByTagName(self.assemblyTag)
        for assembly_node in assembly_nodes:
            assembly_nid = int(assembly_node.attributes["id"].value)
            assembly_obj = kinematics.GenericObject()
            assembly_obj.id = assembly_nid
            rel_pos = None
            motion_frame = None
            self.shapes[assembly_nid] = assembly_obj
            self.shape_types[assembly_nid] = "assembly"
            rel_pos = self.findMatNode(assembly_node, self.relPosTag)
            motion_frame = self.findMatNode(assembly_node,
                                                    self.motionFrameTag)
            assembly_obj.translation = rel_pos[0:3,3]
            assembly_obj.rotation = rot2AxisAngle(rel_pos[0:3,0:3])

            polyhedron_nodes = assembly_node.getElementsByTagName(self.
                                                                  polyhedronTag)

            for polyhedron_node in polyhedron_nodes:
                polyhedron_nid = int(polyhedron_node.attributes["id"].value)
                polyhedron_obj = kinematics.GenericObject()
                polyhedron_obj.id = polyhedron_nid
                polyhedron_obj.name = self.findStringProperty(polyhedron_node,
                                                              self.nameTag)
                assembly_obj.add_child(polyhedron_obj)
                self.shapes[polyhedron_nid] = polyhedron_obj
                self.shape_types[polyhedron_nid] = "polyhedron"
                poly_rel_pos = self.findMatNode(polyhedron_node,
                                                    self.relPosTag)
                poly_motion_frame = self.findMatNode(polyhedron_node,
                                                         self.motionFrameTag)

                polyhedron_obj.translation = poly_rel_pos[0:3,3]
                polyhedron_obj.rotation = rot2AxisAngle(poly_rel_pos[0:3,0:3])
                rel_path = self.findStringNode(polyhedron_node, self.relPathTag)
                logger.info("Parsing %s "%(os.path.join
                                           (self.kxml_dir_name, rel_path)))
                try:
                    objs = vrml_parser.parse(os.path.join
                                           (self.kxml_dir_name, rel_path))
                except:
                    print "Problem occured when parsing {0}".format(
                        os.path.join(self.kxml_dir_name, rel_path))
                    raise
                for obj in objs:
                    if isinstance(obj,kinematics.GenericObject):
                        polyhedron_obj.add_child(obj)
                        # obj.translation = [0,0,0]
                    else:
                        logger.debug("Ignoring %s"%str(obj))

                diffuseColor  = self.findVecProperty(polyhedron_node,
                                                     self.diffuseColorTag)[:-1]
                specularColor = self.findVecProperty(polyhedron_node,
                                                     self.specularColorTag)[:-1]
                ambientColor  = self.findVecProperty(polyhedron_node,
                                                     self.ambientColorTag)[:-1]
                shininess     = self.findFloatProperty(polyhedron_node,
                                                       self.shininessTag)


                for key,value in [#("diffuseColor",diffuseColor),
                                  ("specularColor",specularColor),
                                  ("ambientColor",ambientColor),
                                  ("shininess",shininess),
                                  ]:
                    self._propagate_geo_param(polyhedron_obj, key, value)

                # RELATIVE_POSITION est la position de l'objet dans le repère
                # de son assemblage parent (ou dans le repère du monde si c'est
                # un objet racine).

                # MOTION_FRAME correspond à la position absolue du point de
                # manipulation (point dont les coordonnées sont affichées pour
                # représenter la position de l'objet solide, point utilisé pour
                # afficher le contrôleur graphique de position, etc.).

                # Lors du chargement d'un VRML, les positions indiquées dans le
                # fichier sont bien respectées mais le point de manipulation
                # est initialisé au centre de la boîte englobante des triangles
                # de l'objet. Tu peux réinitialiser cette position à la matrice
                # identité en sélectionnant l'objet et en cliquant sur le
                # premier bouton ("Set Scene Frame") dans le panneau
                # "Manipulate" à droite de la vue.
                # polyhedron_obj.init()
                # bbox = None

                # for mesh in polyhedron_obj.mesh_list:
                #     mbbox = mesh.bounding_box_global()
                #     bbox = self._merge_bboxes(bbox, mbbox)

                # if bbox:
                #     bbox_center = [bbox[0][i]/2 + bbox[1][i]/2 for i in range(3)]
                #     for i in range(3):
                #        polyhedron_obj.translation[i] -= bbox_center[i]
            # assembly_obj.init()
            # bbox = None
            # for mesh in assembly_obj.mesh_list:
            #     mbbox = mesh.bounding_box_global()
            #     bbox = self._merge_bboxes(bbox, mbbox)

            # if bbox:
            #     bbox_center = [bbox[0][i]/2 + bbox[1][i]/2 for i in range(3)]
            #     for i in range(3):
            #         assembly_obj.translation[i] -= bbox_center[i]


        solidref_nodes = dom1.getElementsByTagName(self.solidrefTag)
        for n in solidref_nodes:
            nid = int(n.attributes["id"].value)
            refid = int(n.attributes["referencedComponentId"].value)
            self.shapes[nid] = self.shapes[refid]
            self.shape_types[nid] = self.shape_types[refid]

    def _merge_bboxes(self, box1, box2):
        if box1 == None:
            return box2
        if box2 == None:
            return box1
        res = [None, None]
        res[0] = [min(box1[0][i], box2[0][i]) for i in range(3)]
        res[1] = [min(box1[1][i], box2[1][i]) for i in range(3)]
        return res

    def _propagate_geo_param(self, obj, key, value):
        if isinstance(obj, kinematics.Mesh):
            old_val =  getattr(obj.app, key)
            if old_val != value:
                logger.debug("Mesh %s: %s changed from %s to %s"
                             %(obj.name, key,
                               str(old_val), str(value)))
            setattr(obj.app, key, value)

        for child in obj.children:
            self._propagate_geo_param(child, key, value)
    def parse (self):
        dom1 = dom.parse(self.filename)
        self.parse_geometry(dom1)

        hNodes = dom1.getElementsByTagName(self.robotTag)
        if not hNodes[:]:
            obj = kinematics.GenericObject()
            for id, shape in self.shapes.items():
                obj.add_child(shape)
            obj.init()
            return [obj]

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
             joint_.localTransformation = numpy.dot(
                 numpy.linalg.inv(joint_.parent.globalTransformation),
                 joint_.globalTransformation)
             # print joint_.id, joint_.localTransformation
             joint_.translation = joint_.localTransformation[0:3,3]
             joint_.localR = joint_.localTransformation[0:3,0:3]

             joint_.localR2 = axis_name_angle2rot(joint_.axis,joint_.angle)
             joint_.localR1 = numpy.dot(joint_.localR,
                                        numpy.linalg.inv(joint_.localR2))
             joint_.rotation = rot2AxisAngle(joint_.localR1)

        for child in [ c for c in joint_.children
                       if isinstance(c,kinematics.Joint)]:
            self.compute_localT_from_globalT_(child)



    def createJoint (self, node, parent = None):
        if node.nodeName == "HPP_FREEFLYER_JOINT":
            joint = kinematics.Robot()
        else:
            joint = kinematics.Joint()
            joint.id = int(node.attributes["id"].value)
            if not parent:
                raise Exception("Expected a parent for node %s"%node.nodeName)
            parent.add_child(joint)


        sotJointType = self.jointType[node.nodeName]
        jointName = self.findStringProperty(node, 'NAME')
        joint.name = jointName
        joint.jointType = sotJointType
        if joint.jointType == "rotation":
            joint.angle = self.findJointValue(node)
            joint.axis = "X"

        current_position, relative_solid_position, solid_id = self.findJointPositions(node)

        joint.globalTransformation = copy.copy(current_position)
        self.globalTransformations[joint.id] = copy.copy(current_position)

        # recursively create child joints
        childJointNodes = filter(lambda n:n.nodeName in self.jointTypes,
                             node.childNodes)

        for childJointNode in childJointNodes:
            childJoint = self.createJoint(childJointNode, joint)

        # add shape object already loaded at the beginning
        # print solid_id, self.shape_types[solid_id]
        solid = self.shapes[solid_id]
        solid.translation = relative_solid_position[0:3,3]
        solid.rotation    = rot2AxisAngle(relative_solid_position[0:3,0:3])
        # solid.add_child(self.shapes[solid_id])
        joint.add_child(solid)

        if isinstance(joint, kinematics.Robot):
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
                        msg += "\nkxml global pos = %s"%str(
                            self.globalTransformations[jj.id])
                        jj = jj.get_parent_joint()
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

    def findStringNode(self, node, node_name):
        node = node.getElementsByTagName(node_name)[-1].childNodes[0]
        s = node.data
        s = s.strip()
        return s

    def findMatNode(self, node, node_name):
        s = self.findStringNode(node, node_name)
        return numpy.array( [ [ float(w) for w in line.split() ]
                              for line in s.splitlines()
                              ]
                            )

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
    # print robot.print_joints()
