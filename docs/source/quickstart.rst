Quickstart
**********

The following snippet should load your HRP robot with a segment contained in
``demo.pos, demo.wst, demo.rpy`` and play the loaded motion segment::

    # python snippet here
    from robotviewer.application import Application

    app=Application()
    app.verbose=True
    app.RobotKinematicsFile='HRP2.wrl'
    app.loadBaseName('demo')
    app.init()
    app.run()



.. toctree::
   :maxdepth: 3

