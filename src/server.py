#! /usr/bin/env python

import sys,os

from displayserver import DisplayServer

corba_path = os.path.dirname(os.path.abspath(__file__)) + '/corba'
sys.path = [corba_path] + sys.path

import RobotViewer, RobotViewer__POA

class DisplayServerCorba(DisplayServer):
    """Corba implement of OpenGL server
    """

    def __init__(self):
        """

        Arguments:
        - `element_dict`: dictionary containing all elements to be rendered
        """
        DisplayServer.__init__(self)
        self._initCorba()

    def _initCorba(self):
        edict = self._element_dict
        ds = self
        ##################################
        #      omniORB
        ##################################
        from omniORB import CORBA, PortableServer

        # Import the stubys for the Naming service
        import CosNaming

        # Import the stubs and skeletonsg
        import imp
        import RobotViewer, RobotViewer__POA


        # Define an implementation of the Echo interface
        class Request_i (RobotViewer__POA.Request):
            def req(self, cmd, str_args, conf):
                return ds.run_cmd(cmd, str_args, conf)

        # Initialise the ORB
        orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

        # Find the root POA
        poa = orb.resolve_initial_references("RootPOA")

        # Create an instance of Request_i
        ri = Request_i()

        # Create an object reference, and implicitly activate the object
        ro = ri._this()

        # Obtain a reference to the root naming context
        obj         = orb.resolve_initial_references("NameService")
        rootContext = obj._narrow(CosNaming.NamingContext)

        if rootContext is None:
            print "Failed to narrow the root naming context.\n"+\
                "Is omniNames running?"
            sys.exit(1)

        # Bind a context named "test.my_context" to the root context
        name = [CosNaming.NameComponent("robotviewer", "context")]

        try:
            testContext = rootContext.bind_new_context(name)
            print "New robotviewer context bound"

        except CosNaming.NamingContext.AlreadyBound, ex:
            print "robotviewer context already exists"
            obj = rootContext.resolve(name)
            testContext = obj._narrow(CosNaming.NamingContext)
            if testContext is None:
                print "robotviewer.context exists but is not a NamingContext"
                sys.exit(1)

        # Bind the Echo object to the test context
        name = [CosNaming.NameComponent("Request", "Object")]

        try:
            testContext.bind(name, ro)
            print "New Request object bound"

        except CosNaming.NamingContext.AlreadyBound:
            testContext.rebind(name, ro)
            print "Request binding already existed -- rebound"

            # Note that is should be sufficient to just call rebind() without
            # calling bind() first. Some Naming service implementations
            # incorrectly raise NotFound if rebind() is called for an unknown
            # name, so we use the two-stage approach above

        # Activate the POA
        poaManager = poa._get_the_POAManager()
        poaManager.activate()

        # Everything is running now, but if this thread drops out of the end
        # of the file, the process will exit. orb.run() just blocks until the
        # ORB is shut down

def main():
    """Main function
    """
    ds=DisplayServerCorba()
    ds.run()

if __name__ == '__main__':
    main()

