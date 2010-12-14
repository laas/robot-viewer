#!/usr/bin/env python

from distutils.core import setup
import os

home_dir = os.environ['HOME']
os.system('mkdir -p $HOME/.robotviewer')
config_dir = home_dir + '/.robotviewer'

setup(name='robot-viewer',
      version='1.3.1',
      license='L-GPL',
      platforms='Linux/MacOSX',
      description='A viewer tool for robots',
      long_description='A viewer tool for robots',
      author='Duong Dang',
      author_email='nddang@laas.fr',
      url='www.laas.fr/~nddang',
      packages=['robotviewer',\
                    'robotviewer.idl',\
                    'robotviewer.idl.hpp',\
                    'robotviewer.idl.hpp__POA',
                ],
      package_dir={'robotviewer':'src'},
      requires=['sphinx (>=0.6)','pyopengl'],
      data_files=[('bin',['examples/robotviewer','examples/rv-centipede-cli','examples/rv-sot-bridge']),
                  (config_dir,['data/floor.py','data/config'])
                  ]
      )
