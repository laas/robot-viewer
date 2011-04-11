#!/usr/bin/env python

from distutils.core import setup
import os
execfile('src/version.py')
home_dir = os.environ['HOME']
os.system('mkdir -p $HOME/.robotviewer')
config_dir = home_dir + '/.robotviewer'

setup(name='robot-viewer',
      version=__version__,
      license='L-GPL',
      platforms='Linux/MacOSX',
      description='A viewer tool for robots',
      long_description='A viewer tool for robots',
      author='Duong Dang',
      author_email='nddang@laas.fr',
      url='www.laas.fr/~nddang',
      packages=['robotviewer',\
                    'robotviewer.idl',\
                    'robotviewer.idl.robotviewer_corba',\
                    'robotviewer.idl.robotviewer_corba__POA',
                ],
      package_dir={'robotviewer':'src'},
      requires=['sphinx (>=0.6)','pyopengl'],
      data_files=[('bin',['bin/robotviewer','bin/robotviewer-gtk']),
                  (config_dir,['data/floor.py','data/config.example'])
                  ]
      )

