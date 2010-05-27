Quickstart
**********

1. Make sure you have omniORB up and running

2. Start the name server if needed::

   $ pgrep omniNames || omniNames &

3. Start robot-viewer server::

   $ robotviewer

   Hopefully, after a few seconds, everything described in the config file
   shows up.

   .. image:: ../images/qstart3.png

   You can now use the mouse to move around, zoom in, zoom out...

4. Fire up the client. On another terminal, start a python intepreter::
   
     $ python
     >>> import robotviewer.client as rvcli
     >>> rvcli.help()

     Avalable commands
     help()                              print this message
     list()                              list all objects on server
     disable(name)                       make something invisible
     enable(name)                        make somethin visible
     create(type,name,description)       create a new element 
                                            e.g. create('robot','hrp','/path/to/HRP2.wrl'
     updateConfig(name,config)           move an element around  

     >>> rvcli.list()
     hrp
         type	: Robot
         config	: [0.0, 0.0, 0.70499999999999996, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
     floor    
         type	: script
         config	: [0, 0, 0, 0, 0, 0


     >>> angles = [ 0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0,-24.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, 15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0,-10.0, 10.0, -10.0, 10.0, -10.0, -10.0, 10.0, -10.0, 10.0, -10.0 ]
     >>> from math import pi
     >>> angles = [ angles[i]*pi/180 for i in range(len(angles))]
     >>> waist_pos = [ 0, 0, 0.6487 ]
     >>> waist_rpy = [ 0, 0, 0      ]
     >>> conf = waist_pos + waist_rpy + angles
     >>> rvcli.updateConfig('hrp',conf)
     OK


   Hopefully, after the last command, you can get HRP2 to "half-sitting"
   position.

   .. image:: ../images/qstart4.png

   Now that you are familiar with ``robotviewer.client``, write your own
   client. 

   Some examples, including rv_sot_bridge, are explained 
   :doc:`../example` section of
   this documentation

.. toctree::
   :maxdepth: 3

