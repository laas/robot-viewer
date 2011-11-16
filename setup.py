#!/usr/bin/env python

from distutils.core import setup, Extension
import datetime
from distutils.command.build import build
from distutils.core import Command
from distutils.command.install import install
from distutils.errors import DistutilsOptionError
from distutils import sysconfig
import os
import sys
from pprint import pprint
execfile('src/version.py')
execfile('src/build_manpage.py')
config_dir = os.path.join("share", 'robot-viewer')
import numpy

class install_pc(install):
    def run(self):
        s = """prefix={0}
datarootdir={0}/share
Name: {1}
Description:
Version: {2}
""".format(self.prefix,
           self.config_vars['dist_name'],
           self.config_vars['dist_version'],
           )
        pkgconfig_dir = os.path.join(self.prefix, 'lib', 'pkgconfig')
        if not os.path.isdir(pkgconfig_dir):
            os.makedirs(pkgconfig_dir)
        pkgconfig_file = os.path.join(pkgconfig_dir, self.config_vars['dist_name'] + ".pc")
        with open(pkgconfig_file, 'w') as f:
            f.write(s)
        install.run(self)

build.sub_commands.append(('build_manpage',  lambda *a: True))


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
                    'robotviewer.idl.robotviewer_corba',\
                    'robotviewer.idl.robotviewer_corba__POA',
                ],
      package_dir={'robotviewer':'src'},
      package_data={'robotviewer': ['vrml.sbnf','obj.sbnf','fragment_shader.c','vertex_shader.c'
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
                  ],
      cmdclass={'build_manpage': build_manpage,
                'install': install_pc,
                },

      ext_modules = [Extension(name = "robotviewer._transformations",
                               sources = ["src/transformations.c"],
                               include_dirs=[numpy.get_include()], extra_compile_args=[])],

      )

