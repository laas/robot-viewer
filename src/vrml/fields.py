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

"""
from fields import *
>>> SFBool('FALSE')
False
>>> SFColor('0.5 0.5 0.5')
[0.5, 0.5, 0.5]
>>> MFColor('[ 1.0 0. 0.0, 0 1 0, 0 0 1 ]')
[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
>>> SFFloat('0.123456')
0.123456
>>> MFFloat('0.12, 1.')
[0.12, 1.0]
>>> SFInt32('123456')
123456
>>> r = SFRotation('0.0 1.0 0.0  3.14')
>>> r == [0.0, 1.0, 0.0, 3.14]
True
>>> vecs = MFVector2f('[ 42 666, 7, 94 ]')
>>> vecs == [[42,666],[7,0],[94,0]]
True
>>> vecs = MFVector3f('[ 1 42 666, 7, 94, 0 ]')
>>> vecs == [[1, 42, 666],[7,0,0],[94,0,0],[0,0,0]]
True
"""

import node
import abc
import re

def listify(parser):
    def list_parser(s):
        s = s.lstrip('[')
        s = s.rstrip(']')
        return [parser(w) for w in s.split(",")]
    return list_parser


def SFBool(s):
    if s == "FALSE":
        return False
    elif s == "TRUE":
        return True
    else:
        raise ValueError("Unknown value for SFBool: "+s)

def SFColor(s):
    words = s.split()
    if len(words) != 3:
        raise ValueError("SFColor content should contain 3 numbers: "+s)
    l = []
    for w in words:
        v = float(w)
        if v < 0 or v > 1:
            raise ValueError("All color component must lie bt 0.0 and 1.0"+s)
        l.append(v)
    return l


def SFFloat(s):
    return float(s)


def SFInt32(s):
    return int(s)


def SFRotation(s):
    if len(s.split()) != 4:
        raise Exception("Expected 4 number for a rotation: "+s)
    return [float(w) for w in s.split()]


def SFString(s):
    return eval(s)

def SFVector2f(s):
    res = [0., 0.]
    for i, w in enumerate(s.split()):
        res[i] = float(w)
    return res


def SFVector3f(s):
    res = [0., 0., 0.]
    for i, w in enumerate(s.split()):
        res[i] = float(w)
    return res


def SFNode(Field):
    def parse(self, s):
        self.data = nodes.parse(s)


MFColor = listify(SFColor)
MFFloat = listify(SFFloat)
MFInt32 = listify(SFInt32)
MFRotation = listify(SFRotation)
MFString = listify(SFString)
SFTime = SFString
SFImage = SFString
MFTime = listify(SFTime)
MFVector2f = listify(SFVector2f)
MFVector3f = listify(SFVector3f)
MFNode = listify(SFNode)

if __name__ == '__main__':
    import doctest
    doctest.testmod(optionflags=doctest.ELLIPSIS)

