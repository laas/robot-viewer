#!/usr/bin/env python

from distutils.core import setup

setup(name='robot-viewer',
      version='1.2.1',
      license='BSD',
      platforms='Linux/MacOSX',
      description='A viewer tool for robots',
      long_description='A viewer tool for robots',
      author='Duong Dang',
      author_email='nddang@laas.fr',
      url='www.laas.fr/~nddang',
      packages=['robotviewer',\
                    'robotviewer.corba',\
                    'robotviewer.corba.RobotViewer',\
                    'robotviewer.corba.RobotViewer__POA',\
                    'robotviewer.corba.hppCorbaServer',\
                    'robotviewer.corba.hppCorbaServer__POA'],
      package_dir={'robotviewer':'src'},
      requires=['sphinx >=0.6','pyopengl'],
      )


