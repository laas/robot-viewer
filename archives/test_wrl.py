#! /usr/bin/env python
'''Demonstration of profiling a vrml_view context.  Requires PyGame
'''
from OpenGLContext.testingcontext import getVRML
BaseContext = getVRML()
from OpenGLContext import vrmlcontext
from OpenGLContext.scenegraph.basenodes import *
from OpenGLContext.scenegraph import nurbs
from OpenGLContext.loaders.loader import Loader

import sys
USE_HOTSHOT = 1

class TestContext(BaseContext ):
    """VRML97-loading Context testing class for use with profiling"""

    def OnInit( self ):
#        initialPosition = (0.0,0,0)
        """Load the image on initial load of the application"""
        filename = sys.argv[1]
        body=Loader.load( filename )        
        print "body has %d children:"%len(body.children)
        print body.children
        self.sg = sceneGraph(
            children = [
                  Transform(
#                    rotation=[0,1,0,-3.14],
#                    translation=[0,0,8],
                    children = body.children,
                    ),
                  ],
            )

        for child in self.sg.children:
            print "trans=",child.translation,";rotation=",child.rotation

if __name__ == "__main__":
    TestContext.ContextMainLoop()
