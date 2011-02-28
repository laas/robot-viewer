robot-viewer
============

This package provides a viewer written in OpenGL.


Setup
-----
Make sure you have the following dependency:
 
 * python 2.6

 * pyopengl, numpy, python-numeric

 * [SimpleParse 2.1](http://simpleparse.sourceforge.net/)

 * [omniORB 4.1.4 and omniORBpy](http://omniORB.sourceforge.net) (optional)


To install this package:
    python setup.py install --prefix $YOUR_PREFIX

Usage
-----

 * Edit $HOME/.robotviewer/config to add your robot. robot-viewer supports vrml and kxml formats for robot representation.
 * Start the server:
        robotviewer -s SERVER_TYPE

 where SERVER_TYPE is either "CORBA" or "XML-RPC". If no option is
 given, CORBA will be used.

 * In a python prompt, start the client:
        import robotviewer
        clt = robotviewer.client(server=SERVER_TYPE)

   where SERVER_TYPE corresponds to the kind of server you started in
   previous step. (i.e. "CORBA" or "XML-RPC")

  The following functions are available for the client:
     * createElement(type, name, description)
     * destroyElement(element_name)
     * enableElement(element_name)
     * disableElement(element_name)
     * updateElementConfig(element_name, config)
     * getElementConfig(element_name)
     * listElement()
