Examples
********


.. toctree::
   :maxdepth: 3

StackOfTasks display client
===========================

If installed by robotpkg, robot-viewer will copy the executable
``rv_sot_bridge`` to ``$ROBOTPKG_BASE/bin``. To use this client you need:

1. Start the display server::

     $ robotviewer

2. Start sot shell and run ``simu``, ``coshell`` scripts respectively::

     $ echo $ROBOTPKG_BASE                                     
     /local/nddang/openrobots
     $ rlwrap $ROBOTPKG_BASE/bin/sot/test_shell                
     > run /local/nddang/openrobots/script/simu
     Creating the standard dynamic 
     > run /local/nddang/openrobots/script/coshell
     Warning naming context already bound

3. On another terminal, start the bridge::

     $ rv-sot-bridge

   which should fetch robot state from the sot and send display command to
   robotviewer server. Hopefully you get something like this:

   .. image:: ../images/ex1.png
           

Local-stepper client
====================

This client is used to display robot path using ``hpp-localstepper`` software

   .. image:: ../images/ex2.png
