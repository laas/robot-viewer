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
>>> MFString('''[ "One, Two, Three", "He said, 'Immel did it!'" ]''')
['One, Two, Three', "He said, 'Immel did it!'"]
"""

import abc
import re
import tokenize
import io

def nocomment(s):
    result = []
    g = tokenize.generate_tokens(io.BytesIO(s).readline)
    for toknum, tokval, _, _, _  in g:
        # print(toknum,tokval)
        if toknum != tokenize.COMMENT:
            result.append((toknum, tokval))
    return tokenize.untokenize(result)



def clean(f):
    def new_f(s):
        s = s.strip()
        s = nocomment(s)
        s = s.strip()
        return f(s)
    return new_f

def listify(parser):
    def list_parser(s):
        s = nocomment(s)
        s = s.strip()
        s = s.lstrip('[')
        s = s.rstrip(']')
        return [parser(w) for w in s.split(",") if w !=""]
    return list_parser

@clean
def SFBool(s):
    if s == "FALSE":
        return False
    elif s == "TRUE":
        return True
    else:
        raise ValueError("Unknown value for SFBool: '%s'"%s)

@clean
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

@clean
def SFFloat(s):
    return float(s)

@clean
def MFFloat(s):
    s = s.lstrip('[')
    s = s.rstrip(']')
    s = s.replace(","," ")
    return [float(w) for w in s.split() if w !=""]


@clean
def SFInt32(s):
    return int(s)

@clean
def SFRotation(s):
    if len(s.split()) != 4:
        raise Exception("Expected 4 number for a rotation: "+s)
    return [float(w) for w in s.split() if w!= '']

@clean
def SFVec2f(s):
    res = [0., 0.]
    for i, w in enumerate(s.split()):
        res[i] = float(w)
    return res

@clean
def SFVec3f(s):
    res = [0., 0., 0.]
    for i, w in enumerate(s.split()):
        res[i] = float(w)
    return res


@clean
def SFImage(s):
    return s

MFColor = listify(SFColor)
MFRotation = listify(SFRotation)
MFVec2f = listify(SFVec2f)
MFVec3f = listify(SFVec3f)

if __name__ == '__main__':
    import doctest
    doctest.testmod(optionflags=doctest.ELLIPSIS)

