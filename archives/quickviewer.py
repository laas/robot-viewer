#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang

from pyglet import window
from pyglet.gl import *
from pyglet.gl.glu import *
import pyglet.clock

def resize(width, height):
    if height==0:
        height=1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def draw():
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    glBegin(GL_TRIANGLES)
    sphere = gluNewQuadric()
    gluSphere(sphere,10.0,10,10)

def main():
    win = window.Window(width=640,height=480,visible=False)
    win.on_resize=resize

    init()

    win.set_visible()
    clock=pyglet.clock.Clock()

    while not win.has_exit:
        win.dispatch_events()

        draw()

        win.flip()
        clock.tick()

    print "fps:  %d" % clock.get_fps()

if __name__ == '__main__': 
    main() 
