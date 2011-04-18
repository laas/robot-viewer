import OpenGL
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL.ARB.vertex_buffer_object import *
import time
import sys
from camera import Camera
from mathaux import *
import numpy
import traceback
import logging
# Some api in the chain is translating the keystrokes to this octal string
# so instead of saying: ESCAPE = 27, we use the following.
ESCAPE = '\033'
old_cam_up = None;

logger = logging.getLogger("robotviewer.openglaux")

def IsExtensionSupported (TargetExtension):
    """ Accesses the rendering context to see if it supports an extension.
    Note, that this test only tells you if the OpenGL library
    supports the extension. The PyOpenGL system might not actually
    support the extension.
    """
    Extensions = glGetString (GL_EXTENSIONS)
    # python 2.3
    # if (not TargetExtension in Extensions):
    #	gl_supports_extension = False
    #	print "OpenGL does not support '%s'" % (TargetExtension)
    #	return False

    # python 2.2
    Extensions = Extensions.split ()
    found_extension = False
    for extension in Extensions:
        if extension == TargetExtension:
            found_extension = True
            break;
    if (found_extension == False):
        gl_supports_extension = False
        if logger:
            logger.fatal("OpenGL rendering context does not support '%s'" % (TargetExtension))
        return False

    gl_supports_extension = True

    # Now determine if Python supports the extension
    # Exentsion names are in the form GL_<group>_<extension_name>
    # e.g.  GL_EXT_fog_coord
    # Python divides extension into modules
    # g_fVBOSupported = IsExtensionSupported ("GL_ARB_vertex_buffer_object")
    # from OpenGL.GL.EXT.fog_coord import *
    if (TargetExtension [:3] != "GL_"):
        # Doesn't appear to following extension naming convention.
        # Don't have a means to find a module for this exentsion type.
        return False

    # extension name after GL_
    afterGL = TargetExtension [3:]
    try:
        group_name_end = afterGL.index ("_")
    except:
        # Doesn't appear to following extension naming convention.
        # Don't have a means to find a module for this exentsion type.
        return False

    group_name = afterGL [:group_name_end]
    extension_name = afterGL [len (group_name) + 1:]
    extension_module_name = "OpenGL.GL.ARB.%s" % (extension_name)

    import traceback
    try:
        __import__ (extension_module_name)
        if logger:
            logger.info("PyOpenGL supports '%s'" % (TargetExtension))
    except:
        traceback.print_exc()
        if logger:
            logger.info( 'Failed to import %s'%extension_module_name)
            logger.info("OpenGL rendering context supports '%s'" % (TargetExtension))
        return False

    return True


