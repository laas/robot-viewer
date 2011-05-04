# robot-viewer
A vizualization tool for robots using OpenGL and Python.

Main features:

  *  3d vizualization of multiple robots and objects.
  *  simulation view of on-robot cameras.
  *  screen/screen capture builtin.
  *  server-client design.

## Installation
### Binary package
#### Ubuntu:

    apt-add-repository ppa:dang-duong/ppa
    apt-get update
    apt-get install robot-viewer

#### Fedora:

Create a robotviewer.repo file inside /etc/yum.repo.d with the following lines:
       
    [robotviewer]
    name=Repository for robot-viewer.
    baseurl=http://homepages.laas.fr/nddang/repos/fedora14
    enabled=1
    
Change the line "fedora14" to the appropriate version of your system. Install robot-viewer by the usual way:
  
    yum install robot-viewer
         
         




### From source
Make sure you have the following dependency:
 
 * python 2.6 or 2.7.

 * pyopengl, numpy

 * [SimpleParse 2.1](http://simpleparse.sourceforge.net/)

 * [omniORB 4.1.4 and omniORBpy](http://omniORB.sourceforge.net) (optional)


Install this package as an usual python package:

    python setup.py install --prefix $YOUR_PREFIX

## Usage
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
