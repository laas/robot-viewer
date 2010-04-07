Installation
************

.. toctree::
   :maxdepth: 2

Linux/Unix
==========

Get the latest the tar ball from http://www.laas.fr/~nddang/robot-viewer. You
should be able to install the package by the usual way::
  
  tar xvf robot-viewer.tar.gz
  cd robot-viewer
  python setup.py install

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










    
