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
"""VRML97-compliant Parser

This example is a full VRML97 parser, originally created
for the mcf.vrml VRML-processing system.  It supports all
VRML97 constructs, and should be correct for any VRML97
content you can produce.  The parser is fairly fast
(parsing around 280,000 cps on a 1GHz Athlon machine).
"""
from simpleparse.parser import Parser
import pprint

VRMLPARSERDEF = r'''
<header>         := -[\n]*
vrmlScene        := rootItem*
>rootItem<       := ts,(Proto/ExternProto/ROUTE/('USE',ts,USE,ts)/Script/Node),ts
<Proto>          := 'PROTO',ts,nodegi,ts,'[',ts,(fieldDecl/eventDecl)*,']', ts, '{', ts, vrmlScene,ts, '}', ts
fieldDecl	 := fieldExposure,ts,dataType,ts,name,ts,Field,ts
fieldExposure    := 'field'/'exposedField'
dataType         := 'SFBool'/'SFString'/'SFFloat'/'SFTime'/'SFVec3f'/'SFVec2f'/'SFRotation'/'SFInt32'/'SFImage'/'SFColor'/'SFNode'/'MFBool'/'MFString'/'MFFloat'/'MFTime'/'MFVec3f'/'MFVec2f'/'MFRotation'/'MFInt32'/'MFColor'/'MFNode'
eventDecl        := eventDirection, ts, dataType, ts, name, ts
eventDirection   := 'eventIn'/'eventOut'
<ExternProto>    := 'EXTERNPROTO',ts,nodegi,ts,'[',ts,(extFieldDecl/eventDecl)*,']', ts, ExtProtoURL
extFieldDecl     := fieldExposure,ts,dataType,ts,name,ts
ExtProtoURL      := '['?,(ts,SFString)*, ts, ']'?, ts  # just an MFString by another name :)
<ROUTE>          := 'ROUTE',ts, name,'.',name, ts, 'TO', ts, name,'.',name, ts
Node             := (Def)?,nodegi,ts,'{',ts,(Proto/ExternProto/ROUTE/Attr)*,ts,'}', ts
<Script>         := (Def)?,'Script',ts,'{',ts,(ScriptFieldDecl/ScriptEventDecl/Proto/ExternProto/ROUTE/Attr)*,ts,'}', ts
ScriptEventDecl  := eventDirection, ts, dataType, ts, name, ts, ('IS', ts, IS,ts)?
ScriptFieldDecl  := fieldExposure,ts,dataType,ts,name,ts,(('IS', ts,IS,ts)/Field),ts
SFNull           := 'NULL', ts

# should really have an optimised way of declaring a different reporting name for the same production...
Def              :='DEF',ts,name,ts
USE              := name
IS               := name
>nodegi<         := name
Attr             := name, ts, (('IS', ts,IS,ts)/Field), ts
Field            := ( '[',ts,((SFNumber/SFBool/SFString/('USE',ts,USE,ts)/Script/Node),ts)*, ']', ts )/((SFNumber/SFBool/SFNull/SFString/('USE',ts,USE,ts)/Script/Node),ts)+

name             := -[][0-9{}\000-\020"'#,.\\ ],  -[][{}\000-\020"'#,.\\ ]*
SFNumber         := [-+]*, ( ('0',[xX],[0-9]+) / ([0-9.]+,([eE],[-+0-9.]+)?))
SFBool           := 'TRUE'/'FALSE'
SFString         := '"',(CHARNODBLQUOTE/ESCAPEDCHAR/SIMPLEBACKSLASH)*,'"'
<CHARNODBLQUOTE> :=  -[\134"]+
<SIMPLEBACKSLASH>:= '\134'
<ESCAPEDCHAR>    := '\\"'/'\134\134'
<ts>             :=  ( [ \011-\015,]+ / ('#',-'\012'*,'\n')+ )*
'''
def buildVRMLParser( declaration = VRMLPARSERDEF ):
	return Parser( declaration, "vrmlScene" )

if __name__ == "__main__":
	import os, sys, time
	if sys.argv[1:]:
		filename = sys.argv[1]
		data = open(filename).read()
		parser = buildVRMLParser()
		t = time.time()
		success, tags, next = parser.parse( data)
		d = time.time()-t
		print "parsed %s characters of %s in %s seconds (%scps)"%( next, len(data), d, next/(d or 0.000000001) )
                pprint.pprint(tags[len(tags)-1])

