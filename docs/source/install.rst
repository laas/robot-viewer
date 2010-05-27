Installation
************

.. toctree::
   :maxdepth: 2

Linux/Unix
==========

Get the latest the tar ball from [FIX-ME]. You
should be able to install the package by the usual way::
  
  tar xvf robot-viewer.tar.gz
  cd robot-viewer
  python setup.py install

Copy ``robotviewer`` (server) in ``examples`` directory into the bin directory
at your convenience.

robotpkg
--------

robot-viewer can also be installed as a standard robotpkg-wip package::
  
  cd your/path/robotpkg-wip
  git pull
  cd robot-viewer
  make update
             
Mac OS X
========

The same instructions apply for Mac OS X in general.

On Snow Leopard (OS 10.6), python is set to 64 bit by default, that can cause pyglet to
crash, if that happens on your system, try::

       defaults write com.apple.versioner.python Prefer-32-Bit -bool yes   


Windows
=======

Excecutable for Windows will come in future releases.


Configuration
=============

If you install ``robot-viewer`` by ``robotpkg``, skip this section. 

Otherwise, do the following::

   mkdir $HOME/.robotviewer

Create and edit $HOME/.robotviewer/config, to add the following sections::

   [robots]
   some_robot_name  /path/to/wrl/or/xml/file

   [scripts]
   some_object_name  /path/to/pyopengl/script  # an example is provided in data/floor.py

Once this is done, robotviewer server should start with the robot(s) and
object(s) indicated in the config file. 

Of course, you can always add/remove/enable/disable/update robots,
objects in the scene later on via ``robotviewer.client``.






    
