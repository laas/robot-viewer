#! /usr/bin/env python

__author__ = "Duong Dang"
__version__ = "0.1"

import logging, sys, os, re
import ConfigParser
from xml.dom.minidom import parse, parseString
import math
import gtk
import client
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("possetter")
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

class RangeWidgets(gtk.Table):
    def __init__(self, names = []):
        self.names = names
        no_cols = 3
        no_rows = len(names)/3 + 1
        gtk.Table.__init__(self, no_rows, no_cols, True)
        self.show()
        self.hscales = []
        self.adjustments = []
        self.adj_ids = {}
        for i,name in enumerate(names):
            if i < 3:
                adj = gtk.Adjustment(0.0, -5, 5 , 0.01, 0.1, 0.1)
            else:
                adj = gtk.Adjustment(0.0, -180, 180 , 0.1, 1.0, 1.0)
            hscale = gtk.HScale(adj)
            stab = gtk.Table(1,4,True)
            stab.attach(gtk.Label(name), 0,1,0,1)
            stab.attach(hscale, 1,4,0,1)
            self.hscales.append(hscale)
            self.attach(stab, i% no_cols, i%no_cols + 1, i/no_cols, i/no_cols + 1)
            self.adj_ids[adj] = i
            adj.connect("value-changed", self.changed_cb)
            self.adjustments.append(adj)

    def changed_cb(self, adj, data = None):
        print self.adj_ids[adj]
        print adj.value

class ControllerWidget(RangeWidgets):
    def __init__(self, objname):
        self.objname = objname
        self.clt = client.client()
        print self.objname
        start_pos = self.clt.getElementConfig(self.objname)
        print start_pos
        names = self.clt.listElementDofs(self.objname)

        RangeWidgets.__init__(self, names)
        for i, val in enumerate(start_pos):
            if i > 2:
                val *= 180/math.pi
            self.adjustments[i].set_value(val)

    def changed_cb(self, adj, data = None):
        pos = [adj.value*math.pi/180 for adj in self.adjustments]
        for i in range(3):
            pos[i] *= 180/math.pi
        self.clt.updateElementConfig(self.objname, pos)

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
    return parser

def main():

    parser = get_parser()
    (options, args) = parser.parse_args(sys.argv[1:])

    patt = options.pattern
    if not patt:
        patt = ".*"

    window = gtk.Window (gtk.WINDOW_TOPLEVEL)
    window.connect("destroy", lambda w: gtk.main_quit())
    window.set_title("Position Control")
    clt = client.client()
    notebook = gtk.Notebook()
    window.add(notebook)
    print notebook
    for obj in clt.listElements():
        tab = ControllerWidget(obj)
        if not re.match(r"%s"%patt, obj):
            continue
        notebook.append_page(tab, gtk.Label(obj))

    window.show_all()
    gtk.main()


if __name__ == '__main__':
    main()
