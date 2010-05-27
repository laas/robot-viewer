#! /usr/bin/env python
from simpleparse.common import numbers, strings, comments
from simpleparse.parser import Parser
from simpleparse.dispatchprocessor import *

import os
import pprint

CONFIGDEF='''# note use of raw string when embedding in python code...
file           := config
config         :=  [ \t\n]*, section+
section        :=  '[',identifier,']', ts,'\n', body
body           :=  statement*
statement      :=  (ts,';',comment,'\n')/equality/nullline
nullline       :=  ts,'\n'
comment        :=  -'\n'*
equality       :=  ts, identifier,ts,('=')?,ts,identified,ts,'\n'
identifier     :=  [a-zA-Z], [a-zA-Z0-9_]*
filename       :=  [a-zA-Z/.], [a-zA-Z0-9_/.]*
identified     :=  filename/('"',string,'"')/number/identifier
ts             :=  [ \t]*
char           :=  -[\134"]+
number         :=  [0-9eE+.-]+
string         :=  (char/escapedchar)*
escapedchar    :=  '\134"' / '\134\134'
'''



class ConfigProcessorClass( DispatchProcessor ):
    def __init__(self):
#        DispatchProcessor.__init__(self)
        self.config_dict = dict()
        self.current_section = None
        self.current_identifier = None

    def config( self, (tag,start,stop,subtags), buffer ):        
        dispatchList( self, subtags, buffer )
        return self.config_dict
    
    def section( self, (tag,start,stop,subtags), buffer ):        
        """Proess the section production and it's children"""
        # explicitely add a member to the class

        section_name = dispatch(self,subtags[0],buffer)
        self.current_section = section_name
        if not self.config_dict.has_key(section_name):
            self.config_dict[section_name] = dict()
        dispatchList( self, subtags[1:], buffer )

    def identifier( self, (tag,start,stop,subtags), buffer ):
        """Process the given production and it's children"""
        self.current_identifier = buffer[start:stop]
        return self.current_identifier

    def ts( self, (tag,start,stop,subtags), buffer ):
        return

    def equality( self, (tag,start,stop,subtags), buffer ):
        dispatchList( self, subtags, buffer )

    def nullline( self, (tag,start,stop,subtags), buffer ):
        return

    def comment( self, (tag,start,stop,subtags), buffer ):
        return

    def body( self, (tag,start,stop,subtags), buffer ):
        dispatchList( self, subtags, buffer )
        
    def statement( self, (tag,start,stop,subtags), buffer ):
        dispatchList( self, subtags, buffer )

    def identified( self, (tag,start,stop,subtags), buffer ):
        value = buffer[start:stop]
        (self.config_dict[self.current_section])[self.current_identifier] = value
        self.current_identifier = None
        
          
class ConfigParser( Parser ):
	def buildProcessor( self ):
            return ConfigProcessorClass()
         

def buildConfigParser( declaration = CONFIGDEF ):
	return ConfigParser( declaration, "file" )

def parseConfig(config_file):
    data = open(config_file).read()
    try:
        parser = buildConfigParser()
        result = parser.parse(data)
    except:
        raise Exception("Couldn't parse %s. Check your config file"%config_file)
    return result[1][0]

if __name__ == "__main__":
    testData='''
[robots]
hrp=/local/nddang/openrobots/OpenHRP/Controller/IOserver/robot/HRP2JRL/model/HRP2JRLmain.wrl

[joint_rank]
hrp=/local/nddang/openrobots/share/hrp2_14/HRP2LinkJointRank.xml

; [GLObjects]
; floor /local/nddang/openrobots/share/robotviewer/floor.gl
'''
    parser = buildConfigParser()

    robots=dict()
    link_ranks=dict()

    pprint.pprint( parser.parse( testData)[1][0] )
