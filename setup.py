#!/usr/bin/env python

from distutils.core import setup, Extension
import datetime
from distutils.command.build import build
from distutils.core import Command
from distutils.command.install import install
from distutils.command.install_data import install_data
from distutils.errors import DistutilsOptionError
import distutils.sysconfig
import distutils.config
import os
import sys
from pprint import pprint
execfile('src/version.py')
execfile('src/build_manpage.py')
config_dir = os.path.join("share", 'robot-viewer')
import numpy

class build_pc(Command):
    description = "install .pc file"

    user_options = [('install-dir=', 'd',
                     "directory to install pc files to"),
                    ('force', 'f',
                     "force installation (overwrite existing files)"),
                   ]

    boolean_options = ['force']


    def initialize_options (self):
        self.install_dir = None
        self.force = 0
        self.outfiles = []

    def finalize_options (self):
        self.set_undefined_options('install',
                                   ('install_headers', 'install_dir'),
                                   ('force', 'force'))



    def run(self):
        prefix = self.install_dir
        for i in range(3):
            prefix = os.path.dirname(prefix)
        s = """prefix={0}
datarootdir={0}/share
Name: {1}
Description:
Version: {2}
""".format(prefix,
           'robot-viewer',
           __version__,
           )
        pkgconfig_dir = os.path.join(prefix, 'lib', 'pkgconfig')
        pc_file = os.path.join("data", "robot-viewer.pc")
        with open(pc_file, 'w') as f:
            f.write(s)
            f.close()

build.sub_commands.append(('build_manpage',  lambda *a: True))
build.sub_commands.append(('build_pc',  lambda *a: True))


setup(name='robot-viewer',
      version=__version__,
      license='L-GPL',
      platforms='Linux/MacOSX',
      description='A vizualization tool for robotics',
      long_description="""robot-viewer is a vizualization tool written in Python and OpenGL

Main features:

    3d vizualization of multiple robots and objects.
    simulation view of on-robot cameras.
    screen/screen capture builtin.
    server-client design.
      """,
      author='Duong Dang',
      author_email='dang.duong@gmail.com',
      url='https://github.com/laas/robot-viewer',
      packages=['robotviewer',\
                    'robotviewer.idl',\
                    'robotviewer.vrml',\
                    'robotviewer.idl.robotviewer_corba',\
                    'robotviewer.idl.robotviewer_corba__POA',
                ],
      package_dir={'robotviewer':'src'},
      package_data={'robotviewer': ['vrml/vrml.sbnf','vrml/standard_nodes.wrl',
                                    'obj.sbnf',
                                    'fragment_shader.c','vertex_shader.c', 'models/*'
                                    ]},
      requires=['sphinx (>=0.6)','pyopengl'],
      data_files=[('bin',['bin/robotviewer','bin/robotviewer-gtk']),
                  (config_dir,['data/floor.py',
                               'data/config.example','data/sample.wrl',
                               'data/bunny.obj',
                               'data/coord.py',
                               ]),
                  ("share/idl/robot-viewer",['src/idl/RobotViewer.idl',]),
                  ("share/applications",["data/robotviewer.desktop"]),
                  ("share/pixmaps", ['data/robot-viewer.ico','data/robot-viewer.png',
                                          'data/robot-viewer.xpm']),
                  ('share/man/man1',['data/robotviewer.1','data/robotviewer-gtk.1']),
                  ('lib/pkgconfig',['data/robot-viewer.pc']),
                  ],
      cmdclass={'build_manpage': build_manpage,
                'build_pc': build_pc,
                },

      ext_modules = [Extension(name = "robotviewer._transformations",
                               sources = ["src/transformations.c"],
                               include_dirs=[numpy.get_include()], extra_compile_args=[])],

      )

