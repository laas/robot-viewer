#!/usr/bin/env python
import sys
from distutils.core import setup

setup(name='robot-viewer',
      version='0.1.1',
      license='BSD',
      platforms='Linux/MacOSX',
      description='A viewer tool for robots',
      long_description='A viewer tool for robots',
      author='Duong Dang',
      author_email='nddang@laas.fr',
      url='www.laas.fr/~nddang',
      packages=['robotviewer',\
                    'robotviewer.pyglet',\
                    'robotviewer.pyglet.text',\
                    'robotviewer.pyglet.text.formats',\
                    'robotviewer.pyglet.image',\
                    'robotviewer.pyglet.image.codecs',\
                    'robotviewer.pyglet.app',\
                    'robotviewer.pyglet.graphics',\
                    'robotviewer.pyglet.media',\
                    'robotviewer.pyglet.media.drivers',\
                    'robotviewer.pyglet.media.drivers.openal',\
                    'robotviewer.pyglet.media.drivers.directsound',\
                    'robotviewer.pyglet.media.drivers.alsa',\
                    'robotviewer.pyglet.window',\
                    'robotviewer.pyglet.window.carbon',\
                    'robotviewer.pyglet.window.xlib',\
                    'robotviewer.pyglet.window.win32',\
                    'robotviewer.pyglet.gl',\
                    'robotviewer.pyglet.font',\
                    'robotviewer.simplui'],
      package_dir={'robotviewer':'src'},
      package_data={'robotviewer':['themes/macos/theme.json',\
                                      'themes/macos/theme.png',\
                                      'themes/pywidget/theme.json',\
                                      'themes/pywidget/theme.png']},
#      data_files=[('bin',['src/robot-viewer','src/robot-viewer-cli'])]
      data_files=[('bin',['src/robot-viewer'])],
      requires=['simpleparse']
     )
