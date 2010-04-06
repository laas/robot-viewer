
#!/usr/bin/env python
import sys
#from distutils.core import setup
try:
    from setuptools import setup
except:
    raise Exception("""
ERROR: python-setuptools not found on your system. 
ERROR: Please resolve this problem first.
ERROR: You might want to try something like:
ERROR:   On Ubuntu: sudo apt-get install python-setuptools
ERROR:   On Fedora: sudo yum install python-setuptools
ERROR: ==========================================================""")
else:
    setup(name='robot-viewer',
          version='0.3',
          license='BSD',
          platforms='Linux/MacOSX',
          description='A viewer tool for robots',
          long_description='A viewer tool for robots',
          author='Duong Dang',
          author_email='nddang@laas.fr',
          url='www.laas.fr/~nddang',
          packages=['robotviewer',\
                        'robotviewer.simplui',\
                        'robotviewer.RobotViewer',\
                        'robotviewer.RobotViewer__POA'],
          package_dir={'robotviewer':'src'},
          package_data={'robotviewer':['themes/macos/*',\
                                          'themes/pywidget/*',\
                                           '../data/*'\
                                           ]},
          data_files=[('bin',['src/robot-viewer','src/robot-viewer-cli'])],
    #      data_files=[('bin',['src/robot-viewer'])],
    #      requires=['simpleparse'],
          install_requires=['simpleparse >=2.1','sphinx >=0.6','pyglet >=1.1.4'],
         )
