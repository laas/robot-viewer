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

import node
import abc
import re

class Field(object):
    __metaclass__ = abc.ABCMeta
    data = None


    def __init__(self, s = None):
        if s:
            self.parse(s)

    def __str__(self):
        s = str(self.data)
        if len(s) > 500:
            s = s[:500] + "..."
        return "{0}: {1}".format(self.__class__.__name__,s )

    def content(self, s):
        p = re.compile(r"\s*\[(?P<data>.*)\]\s*")
        m = p.match(s)
        if m:
            return m.group('data')
        raise Exception("Couldn't get content of" + s)

    @abc.abstractmethod
    def parse(self, s):
        return

class SFBool(Field):
    def parse(self, s):
        if s == "FALSE":
            self.data = False
        elif s == "TRUE":
            self.data = True
        else:
            raise ValueError("Unknown value for SFBool: "+s)

class SFColor(Field):
    def parse(self, s):
        content = self.content(s)
        self.data = self.parse_content(content)

    def parse_content(self, content):
        words = content.split()
        if len(words) != 3:
            raise ValueError("SFColor content should contain 3 numbers: "+s)
        l = []
        for w in words:
            v = float(w)
            if v < 0 or v > 1:
                raise ValueError("All color component must lie bt 0.0 and 1.0"+s)
            l.append(v)
        return l


class MFColor(Field):
    def parse(self, s):
        content = self.content(s)
        words = content.split(",")
        self.data = []
        for w in words:
            c = SFColor()
            l = c.parse_content(w)
            self.data.append(l)

class SFFloat(Field):
    def parse(self, s):
        self.data = float(s)

class MFFloat(Field):
    def parse(self, s):
        content = self.content(s)
        self.data = [float(w) for w in content.split(",")]

class SFInt32(Field):
    def parse(self, s):
        self.data = int(s)

class MFInit32(Field):
    def parse(self, s):
        content = self.content(s)
        self.data = [int(w) for w in content.split(",")]

class SFImage(Field):
    def parse(self, s):
        self.data = s


class SFRotation(Field):
    def parse(self, s):
        if len(s.split()) != 4:
            raise Exception("Expected 4 number for a rotation: "+s)
        self.data = [float(w) for w in s.split()]

class MFRotation(Field):
    def parse(self, s):
        self.data = []
        for w in s.split(","):
            r = SFRotation(w)
            self.data.append(r.data)

class SFString(Field):
    def parse(self, s):
        print s
        self.data = eval(s)

class MFString(Field):
    def parse(self, s):
        content = self.content(s)
        self.data = []
        for w in content.split(','):
            sfs = SFString(w)
            self.data.append(sfs.data)


class SFTime(Field):
    def parse(self, s):
        self.data = s

class MFTime(Field):
    def parse(self, s):
        self.data = s

class SFVector2f(Field):
    def parse(self, s):
        content = self.content(s)
        words = content.split()
        if len(words) !=2:
            raise Exception("Expected 2 words: "+s)
        self.data = [float(w) for w in words]

class MFVector2f(Field):
    def parse(self, s):
        self.data = []
        content = self.content(s)
        words = content.split(',')
        for word in words:
            self.data.append([float(c) for c in words])

class SFVector3f(Field):
    def parse(self, s):
        content = self.content(s)
        words = content.split()
        if len(words) !=2:
            raise Exception("Expected 2 words: "+s)
        self.data = [float(w) for w in words]

class MFVector3f(Field):
    def parse(self, s):
        self.data = []
        content = self.content(s)
        words = content.split(',')
        for word in words:
            self.data.append([float(c) for c in word.split()])

if __name__ == '__main__':
    print SFBool('FALSE')
    print MFColor('[ 1.0 0. 0.0, 0 1 0, 0 0 1 ]')
    print MFFloat('[ 3.1415926, 12.5e-3, .0001 ]')
    print SFImage('1 2 1 0xFF 0x00')
    #print MFInt32('[ 17, -0xE20, -518820 ]')
    print SFRotation('0.0 1.0 0.0  3.14159265')
    print MFString('[ "One, Two, Three", "He said, \"Immel did it!\"" ]')
    print MFVector3f('[ 1 42 666, 7 94 0 ]')

def SFNode(Field):
    def parse(self, s):
        self.data = nodes.parse(s)

def MFNode(Field):
    def parse(self, s):
        self.data = nodes.parse(s)
