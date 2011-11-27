__version__ = "0.1"

import logging, sys, os, re
import ConfigParser
from xml.dom.minidom import parse, parseString
import math
import client
import gtk
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("robotviewer.controller_widget")
logger.addHandler(NullHandler())
logger.setLevel(logging.DEBUG)


class RangeWidgets(gtk.Table):
    import gtk
    def __init__(self, names = []):
        self.names = names
        no_cols = 2
        no_rows = len(names)/no_cols + 1
        gtk.Table.__init__(self, no_rows, no_cols, True)
        self.show()
        self.hscales = []
        self.adjustments = []
        self.adj_ids = {}
        for i,name in enumerate(names):
            if i < 3:
                adj = gtk.Adjustment(0.0 , -5, 5 , 0.001, 0.01, 0.01)
            else:
                adj = gtk.Adjustment(0.0 , -180, 180 , 0.1, 1.0, 1.0)
            hscale = gtk.HScale(adj)
            hscale.set_digits(3)
            stab = gtk.Table(1,4,True)
            stab.attach(gtk.Label(name), 0,1,0,1)
            stab.attach(hscale, 1,4,0,1)
            self.hscales.append(hscale)
            self.attach(stab, i% no_cols, i%no_cols + 1,
                        i/no_cols, i/no_cols + 1)
            self.adj_ids[adj] = i
            adj.connect("value-changed", self.changed_cb)
            self.adjustments.append(adj)

    def changed_cb(self, adj, data = None):
        print self.adj_ids[adj]
        print adj.value

class ControllerWidget(RangeWidgets):
    def __init__(self, objname, clt):
        self.objname = objname
        self.clt = clt
        start_pos = self.clt.getElementConfig(self.objname)
        print objname, " initial config:", start_pos
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
