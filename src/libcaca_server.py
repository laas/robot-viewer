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

import os, sys
import logging
logger = logging.getLogger("robotviewer.libcaca_server")
class NullHandler(logging.Handler):
    def emit(self, record):
        pass
logger.addHandler(NullHandler())

from kinematic_server import KinematicServer
from camera import Camera
from kinematics import Robot

import caca
from caca.canvas import Canvas
from caca.display import Display, Event
import time

class LibcacaServer(KinematicServer):
    def __init__(self, *args, **kwargs):
        """
        """
        KinematicServer.__init__(self, *args, **kwargs)
        self.robots = [r for (name, r) in self.elements.items()
                       if isinstance(r, Robot)]
        self.cv = Canvas()

        self.dp = Display(self.cv)
        self.ev = Event()
        self.quit = False
        self.width = self.cv.get_width()
        self.height = self.cv.get_height()
        self.camera = Camera(self, 640 , 480)
        self.camera.translation = [3.5, 0, 1]
        self.camera.init()
        self.fps = -1.0
        self.frames = 0
        self.last_t = 0

    def compute_fps(self):
        now = time.time()
        if self.last_t == 0:
            self.last_t = now
            return
        PER = 2.0
        if now - self.last_t >= PER:
            self.fps = self.frames / PER
            self.frames = 0
            self.last_t = now
            return
        else:
            self.frames += 1


    def key_cb(self):
        UP, DOWN, LEFT, RIGHT = 273,274,275,276
        if self.dp.get_event(caca.EVENT_KEY_PRESS, self.ev, 0):
            ch = self.ev.get_key_ch()
            if ch == ord('q'):
                self.quit = True
            elif ch == UP:
                self.camera.rotate(0, 1)
            elif ch == DOWN:
                self.camera.rotate(0, -1)
            elif ch == LEFT:
                self.camera.rotate(1, 0)
            elif ch == RIGHT:
                self.camera.rotate(-1, 0)


    def project(self, p):
        u, v = self.camera.project(p)
        up, vp = int(u*self.width/self.camera.width), int((v)*self.height/self.camera.height)
        return up, self.height - vp


    def draw_floor(self):
        lines=[]
        L = 5
        w = 1
        N = int(L/w)
        self.cv.set_color_ansi(caca.COLOR_WHITE, caca.COLOR_BLACK)

        for i in range(-N,N+1):
            lines.append([i*w, L,0.005])
            lines.append([i*w,-L,0.005])

            lines.append([L    , i*w,0.005])
            lines.append([-L   , i*w,0.005] )


        for i  in range(len(lines)/2):
            p1 = lines[2*i]
            p2 = lines[2*i+1]
            u1, v1 = self.project(p1)
            u2, v2 = self.project(p2)
            self.cv.draw_thin_line(int(u1), int(v1), int(u2), int(v2))

    def draw_robot(self, r):
        self.cv.set_color_ansi(caca.COLOR_GREEN, caca.COLOR_BLACK)
        for j in r.moving_joint_list:
            u, v = self.project(j.T[:3,3])
            radius = 0
            if j.parent:
                up, vp = self.project(j.parent.T[:3,3])
                self.cv.draw_thin_line(int(u), int(v), int(up), int(vp))

        self.cv.set_color_ansi(caca.COLOR_RED, caca.COLOR_BLACK)
        for j in r.moving_joint_list:
            u, v = self.project(j.T[:3,3])
            self.cv.draw_circle(u, v, radius, '@')

    def run(self):
        try:
            while not self.quit:
                self.key_cb()
                self.compute_fps()
                self.cv.clear()
                self.draw_floor()
                for r in self.robots:
                    self.draw_robot(r)

                self.cv.put_str(0,0,"%3.1f FPS"%self.fps)
                self.dp.refresh()
            del self.cv, self.dp, self.ev
        except:
            del self.cv, self.dp, self.ev
