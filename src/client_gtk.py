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
__author__ = "Duong Dang"
__version__ = "0.1"

import logging, sys, os, re
import ConfigParser
from xml.dom.minidom import parse, parseString
import math
import client
import yaml
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("robotviewer.client_gtk")
logger.addHandler(NullHandler())
logger.setLevel(logging.DEBUG)

config_file = os.path.join(os.environ['HOME'],'.robotviewer/config')
def replace_env_var(s):
    matches = re.findall(r'\$(\w+)',s)
    for m in matches:
        var = m
        val = os.environ[var]
        s = s.replace(var,val)
        s = s.replace('$','')
    return s

import optparse

def get_parser():
    usage = "usage: [options]"
    description ="""an example client for robot-viewer

    Provides a GUI written with GTK to control objects in a robotviewer scene
"""

    parser = optparse.OptionParser(
        usage = usage, description = description)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")
    parser.add_option("-p", "--pattern",
                      action="store", dest="pattern",
                      help="Regular expression describing elements to be controlled.")

    parser.add_option("-s","--server", dest="server", default = "CORBA",
                      help="set server type to be used (default: CORBA)")

    return parser

def main():
    from controller_widget import ControllerWidget
    import gtk
    parser = get_parser()
    (options, args) = parser.parse_args(sys.argv[1:])

    import gtk
    window = gtk.Window (gtk.WINDOW_TOPLEVEL)
    window.set_default_size(720,480)
    window.connect("destroy", lambda w: gtk.main_quit())
    window.set_title("Position Control")
    clt = client.client(options.server)

    notebook = gtk.Notebook()
    window.add(notebook)

    cfg_notebook = gtk.Notebook()
    notebook.append_page(cfg_notebook, gtk.Label("Configs"))


    import display_server
    keys = display_server.DisplayServer.param_keys
    N = len (keys)
    param_tab = gtk.Table(N, 2, True)

    param_labels = {}
    param_entries = {}


    for i, key in enumerate(keys):
        label = gtk.Label(key)
        label.set_justify(gtk.JUSTIFY_LEFT)
        param_tab.attach(label, 0, 1, i, i+1)#,gtk.EXPAND)
        entry = gtk.Entry()
        param_tab.attach(entry, 1, 2, i, i+1)#,gtk.EXPAND)

        def callback(entry, key):
            val = entry.get_text()
            val = display_server.DisplayServer.parse_number(val)
            d = { key: val}
            clt.setParams(yaml.dump(d))

        entry.connect("activate", callback, key)


        param_labels[key] = label
        param_entries[key] = entry


    #param_notebook = gtk.Notebook()
    notebook.append_page(param_tab, gtk.Label("Params"))


    for obj in clt.listElements():
        if ":" in obj:
            continue
        if "_" in obj:
            continue
        sw = gtk.ScrolledWindow()
        tab = ControllerWidget(obj, clt)
        vbox = gtk.VBox()
        button = gtk.Button(obj)
        def cb(button, data = None):
            print clt.getElementConfig(button.get_label())
        button.connect("clicked", cb)
        vbox.pack_start(button, expand=False, fill=False)
        vbox.pack_end(tab)

        sw.add_with_viewport(vbox)
        cfg_notebook.append_page(sw, gtk.Label(obj))


    s = clt.getParams()
    params = yaml.load(s)

    def to_text(l):
        if type(l) == list:
            return ", ".join(["{0:0.02f}".format(w) for w in l])
        return "{0:0.02f}".format(l)

    for key in params:
        param_entries[key].set_text(to_text(params[key]))


    window.show_all()
    gtk.main()


if __name__ == '__main__':
    main()
