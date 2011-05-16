#!/usr/bin/env python

from distutils.core import setup, Extension
import os
execfile('src/version.py')
execfile('src/build_manpage.py')
config_dir = os.path.join("share", 'robot-viewer')

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
      package_data={'robotviewer': ['vrml.sbnf','obj.sbnf',
                                    ]},
      requires=['sphinx (>=0.6)','pyopengl'],
      data_files=[('bin',['bin/robotviewer','bin/robotviewer-gtk']),
                  (config_dir,['data/floor.py',
                               'data/config.example','data/sample.wrl',
                               'data/bunny.obj',
                               'data/coord.py']),
                  ("share/applications",["data/robotviewer.desktop"]),
                  ("share/pixmaps", ['data/robot-viewer.ico','data/robot-viewer.png',
                                          'data/robot-viewer.xpm']),
                  ('share/man/man1',['data/robotviewer.1','data/robotviewer-gtk.1']),
                  ],
      cmdclass={'build_manpage': build_manpage,
                #'build_manpage2':build_manpage
                },

      #ext_modules = [Extension("robotviewer.oglc",
      #                          sources = ["src/oglc.cc"],
      #                          libraries = ['GL','GLU'],
      #                          )]
      )

