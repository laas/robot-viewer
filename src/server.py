#! /usr/bin/env python
import sys,imp, os, stat, glob
from optparse import OptionParser
import logging
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
import version
__version__ = version.__version__
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)
# create formatter and add it to the handlers
#formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
formatter = logging.Formatter("%(name)s:%(levelname)s:%(message)s")
ch.setFormatter(formatter)
# add the handlers to the logger
logger.addHandler(ch)


def get_parser():
    usage = "usage: [options]"
    description ="""the server side of robot-viewer

    It does the following in order:

    * Create OpenGL context(s)
    * Parse user config file at $HOME/.robotviewer/config
    * Display initial scene
    * Start communication server (CORBA or XML-RPC)

"""
    parser = OptionParser(usage=usage, description = description)

    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")

    parser.add_option("-s","--server", dest="server", default = "CORBA",
                      help="set server type to be used (default: CORBA)")

    parser.add_option("-c","--config-file", dest="config_file",
                      help="set config file")

    parser.add_option("--no-cache",
                      action="store_true", dest="no_cache", default=False,
                      help="force parsing robots' kinematics file")

    parser.add_option("--use-vbo",
                      action="store_true", dest="use_vbo", default=False,
                      help="use vbo instead of display list to draw triangles")

    parser.add_option("--run-once",
                      action="store_true", dest="run_once", default=False,
                      help="render one frame, make a screenshot and quit (debug use only)")

    parser.add_option("--skeleton",
                      action="store_true", dest="skeleton", default=False,
                      help="only display robot skeletons")

    parser.add_option("--strict",
                      action="store_true", dest="strict", default=False,
                      help="kill OpenGL on exception (might be caused by client).")

    parser.add_option("--version",
                      action="store_true", dest="version", default=False,
                      help="print version and exit")

    parser.add_option("--off-screen",
                       action="store_true", dest="off_screen", default=False,
                       help="offscreen mode")

    parser.add_option("-r", "--refresh-rate",
                      action="store", dest="refresh_rate", type = "int", default = 1000,
                      help="set refresh rate")

    parser.add_option("--num-windows",
                      action="store", dest="num_windows", type = "int", default = 1,
                      help="set number(s) of windows")

    parser.add_option("--width",
                      action="store", dest="width", type = "int", default = 640,
                      help="set window(s) width")

    parser.add_option("--height",
                      action="store", dest="height", type = "int", default = 480,
                      help="set window(s) height")

    parser.add_option("--stream",
                      action="store", dest="port", type = "int",
                      help="stream PNG to a port (UDP)")

    parser.add_option("--intel",
                      action="store_true", dest="intel",  default=False,
                      help="tell robot-viewer that it is running on an Intel card")
    return parser

def main():
    """
    """

    config_dir = os.path.join(os.environ['HOME'], ".robotviewer")
    if not os.path.isdir(config_dir):
        os.mkdir(config_dir)
    os.chmod(config_dir,
             stat.S_IWUSR | stat.S_IRUSR | stat.S_IXUSR)
    prefix_dir = os.path.abspath(os.path.dirname(__file__))
    prefix_dir = os.path.split(prefix_dir)[0]
    prefix_dir = os.path.split(prefix_dir)[0]
    prefix_dir = os.path.split(prefix_dir)[0]
    prefix_dir = os.path.split(prefix_dir)[0]

    default_config_dir = os.path.join( prefix_dir,
                                      "share","robot-viewer")

    def visit_cb(arg, dirname, names):
        for name in names:
            if os.path.exists(os.path.join(config_dir,name)):
                os.remove(os.path.join(config_dir,name))
            os.symlink(os.path.join(dirname, name),
                       os.path.join(config_dir,name)
                       )
    os.path.walk(default_config_dir, visit_cb, None)
    config_file = os.path.join(config_dir, 'config')


    # os.system("source $ROBOTPKG_BASE/OpenHRP/bin/unix/config.sh")
    parser = get_parser()
    (options, args) = parser.parse_args()

    if options.version:
        print __version__
        sys.exit(0)


    if options.verbose:
        ch.setLevel(logging.DEBUG)

    if options.server == "CORBA":
        logger.info("Starting robotviewer in corba mode")
        import displayserver_corba
        DisplayServer = displayserver_corba.DisplayServerCorba
    elif options.server == "XML-RPC":
        logger.info("Starting robotviewer in xmlrpc mode")
        import displayserver_xmlrpc
        logger.debug("Imported displayserver_xmlrpc")
        DisplayServer = displayserver_xmlrpc.DisplayServerXmlrpc
    else:
        raise Exception ("Not supported server type %s"%options.server)
    server = DisplayServer(options,args)
    logger.debug("created server")
    server.run()

if __name__ == '__main__':
    main()
