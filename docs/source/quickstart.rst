Quickstart
**********

Writing your own script
=======================


The following snippet should load your HRP robot with a segment contained in
``demo.pos, demo.wst, demo.rpy`` and play the loaded motion segment::

    #! /usr/bin/env python
    from robotviewer.application import Application

    app=Application()
    app.init()
    app.loadRobot('./hrpmodel/HRP2JRL/model/HRP2JRLmain.wrl')
    app.loadBaseName('demo')
    app.state="PLAY"
    app.run()


Using provided robot-viewer script
==================================

Standalone mode
---------------

 * Fire up ``robot-viewer`` and use the GUI to load robot and motion

Client-Server mode
------------------

 * Fire up (in seperate terminals) ``omniNames`` and ``robot-viewer``
 * In a third terminal, use robot-viewer-cli to interact with the
   display. Use a robot-viewer-cli (with out argument) to get help::

     $ robot-viewer-cli 

        Available commands:  
        list                          : list loaded segments
        play [id]                     : play a segment in the list
        stop                          :
        pause                         :
        next                          :
        prev                          :
        speed <s>                     : set playing speed
        load <segment.pos>            : load a segment
        getConfig                     : print robot config (x y z r p y q)
        setConfig <0 0.6 etc.>        : set robot to a config (x y z r p y q)

You need omniORB up and running to use this feature

.. toctree::
   :maxdepth: 3

