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
     >>> from robotviewer import client

     >>> client.list()
     ['hrp', 'floor']

     >>> angles = [ 0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0,-24.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, 15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0,-10.0, 10.0, -10.0, 10.0, -10.0, -10.0, 10.0, -10.0, 10.0, -10.0 ]
     >>> from math import pi
     >>> angles = [ angles[i]*pi/180 for i in range(len(angles))]
     >>> waist_pos = [ 0, 0, 0.6487 ]
     >>> waist_rpy = [ 0, 0, 0      ]
     >>> conf = waist_pos + waist_rpy + angles
     >>> client.updateElementConfig('hrp',conf)
     OK


   Hopefully, after the last command, you can get HRP2 to "half-sitting"
   position.

   .. image:: ../images/qstart4.png

   Now that you are familiar with ``robotviewer.client``, write your own
   client.

   Some examples, including rv_sot_bridge, are explained
   :doc:`../example` section of
   this documentation

   Other available commands is described in the :doc:`../idl`

.. toctree::
   :maxdepth: 3

