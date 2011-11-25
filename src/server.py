# Copyright (c) 2010-2011, Duong Dang <mailto:dang.duong@gmail.com>
# This file is part of robot-viewer.

# robot-viewer is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# robot-viewer is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with robot-viewer.  If not, see <http://www.gnu.org/licenses/>.
#! /usr/bin/env python

import sys,imp, os, stat, glob
from optparse import OptionParser
import logging
from server_factory import create_server, CORBA, XML_RPC, KINEMATIC, DISPLAY


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

    parser.add_option("--no-gl",
                      action="store_true", dest="no_gl",
                      help="start kinematic server only, no GL")

    parser.add_option("--no-shader",
                      action="store_false", dest="use_shader", default = True,
                      help="no shader")

    parser.add_option("--log-module",
                      action="store", dest="log_module",
                      help="limite logging to this module only (to be used with -v)")

    parser.add_option("--ros",
                      action="store_true", dest="ros",
                      help="start a ros node")

    # parser.add_option("--intel",
    #                   action="store_true", dest="intel",  default=False,
    #                   help="tell robot-viewer that it is running on an Intel card")
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

    # os.system("source $ROBOTPKG_BASE/OpenHRP/bin/unix/config.sh")
    parser = get_parser()
    (options, args) = parser.parse_args()

    if options.version:
        print __version__
        sys.exit(0)

    type = DISPLAY
    if options.no_gl:
        type = KINEMATIC
    if options.server == "CORBA":
        com_type = CORBA
    elif options.server == "XML-RPC":
        com_type = XML_RPC
    else:
        raise Exception ("Not supported server type %s"%options.server)

    if options.log_module:
        logger = logging.getLogger("robotviewer."+options.log_module)
    else:
        logger = logging.getLogger()

    logger.handlers = []
    import version
    __version__ = version.__version__
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    # create formatter and add it to the handlers
    #formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")

    class MyFormatter(logging.Formatter):
        width = 25

        def format(self, record):
            max_filename_width = self.width - 3 - len(str(record.lineno))
            filename = record.filename
            if len(record.filename) > max_filename_width:
                filename = record.filename[:max_filename_width]
            a = "%s:%s"% (filename, record.lineno)
            return "%s %s:%s" % (a.ljust(self.width), record.levelname, record.msg)


    #formatter = logging.Formatter("%(name)s:%(levelname)s:%(message)s")
    formatter = MyFormatter("%(levelname)s:%(message)s")
    ch.setFormatter(formatter)
    # add the handlers to the logger
    logger.addHandler(ch)
    logger.setLevel(logging.INFO)


    if options.verbose:
        ch.setLevel(logging.DEBUG)
        logger.setLevel(logging.DEBUG)

    def visit_cb(arg, dirname, names):
        for name in names:
            f = os.path.join(config_dir,name)
            if os.path.exists(f):
                logger.debug("Removing %s"%f)
                os.remove(f)
            os.symlink(os.path.join(dirname, name),
                       os.path.join(config_dir,name)
                       )
    os.path.walk(default_config_dir, visit_cb, None)
    config_file = os.path.join(config_dir, 'config')



    server = create_server(type, com_type, options, args)
    logger.debug("created server")

    if options.ros:
        from ros_bridge import Bridge
        bridge = Bridge(server)

    server.run()

if __name__ == '__main__':
    main()
