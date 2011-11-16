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

import logging, os, sys

from simpleparse.common import numbers, strings, comments
from simpleparse.parser import Parser
from simpleparse.dispatchprocessor import *
from numbers import Number
import pprint
import inspect
import copy
from decimal import Decimal
from decimal import Decimal
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("robotviewer.vrml.parser")
logger.addHandler(NullHandler())
logger.setLevel(logging.INFO)


class VrmlNode(dict):
    def __init__(self, n = None):
        self.name = n
        self['name'] = n

class definition_str(str):
    pass

class Attribute(tuple):
    pass

class ISDef(str):
    pass

class Prototype(type):
    def __new__(cls, name, bases, dct):
        try:
            proto_name = dct['name']
            attrs = dct['attrs']
        except:
            attrs = {}

        return type.__new__(cls, proto_name , bases, attrs)

def compare(a, b):
    diff = sum([(a[i] - b[i])**2 for i in range(len(a))])
    return diff < 1e-6


def validate_field(fdata, ftype):
    # if ftype == "SFBool":
    #     if not isinstance(fdata, bool):
    #         raise "invalid field {0} of type {1}".format(fdata, ftype)
    # elif ftype == "SFColor":
    #     if (len(fdata) !=3 or (not isinstance(fdata[0], float))
    #         or (not isinstance(fdata[1], float))
    #         or (not isinstance(fdata[2], float))
    #         ):
    #         raise "invalid field {0} of type {1}".format(fdata, ftype)
    # elif
    # FIXME
    if ftype == "SFInt32":
        fdata = int(fdata)

    return fdata

class Node(object):
    _parent = None
    _route = []
    _aliases = []
    children = []
    _declared = []
    _ftypes = []

    def __getattr__(self, att):
        for route, name, alias in self._aliases:
            if alias == att:
                try:
                    return getattr(self.get_descendant(route), name)
                except:
                    logger.exception("Failed to delegate alias {0} to {1} in {2}".
                                     format(att, route, self))
                    raise
        raise AttributeError("Invalid Tribute {0} in {1}".format(att, self.__class__.__name__))

    def __setattr__(self, att, value):
        for route, name, alias in self._aliases:
            if alias == att:
                logger.debug("setting {0} {1} {2} {3} {4} \non {5}".
                            format(route, name, alias, att, value, self))
                desc = self.get_descendant(route)
                res =  setattr(desc, name, value)
                if name == 'children':
                    for child in desc.children:
                        child._parent = desc
                return res
        return object.__setattr__(self, att, value)

    @classmethod
    def ftype(cls, fname):
        if fname in cls._ftypes:
            return cls._ftypes.get(fname)
        base = cls.__bases__[0]
        if base == object:
            return None
        return base.ftype(fname)

    def depth(self):
        if self._parent == None:
            return 0
        return 1 + self._parent.depth()

    def get_descendant(self, route):
        result = self
        count = 0
        while count < len(route):
            i = route[len(route)- 1 - count]
            result = result.children[i]
            count += 1
        return result

    def __str__(self):
        s = "|{0}{1}:{2}:".format(self.depth()*"_",self.depth(), self.__class__.__name__, )
        try:
            name = getattr(self, 'name')
        except AttributeError:
            name = ""
        s += name
        s += " "

        #s += "{0}".format(self._parent.__class__.__name__)'
        children = self.children
       # s += "({0})".format(len(children))

        attrs = [(key, getattr(self, key)) for key in self._declared if (key != "children")]
        for key, value in attrs:
            s += key + ":"
            if isinstance(value, list):
                s += "["
                s += ",".join(str(v) for v in value[:10])
                if len(value) > 10:
                    s += ",...."
                s += "]"
            else:
                s += str(value)
            s += ","
        for child in children:
            s += "\n{0}".format(child)
        return s

class VrmlProcessor(DispatchProcessor):
    """
    """
    def __init__(self, root_path = None, prototypes = {}):
        self.def_dict = {}
        self.root_path = root_path
        self.prototypes = prototypes
        self.proto_aliases = []
        self.clss = []

    @property
    def cls(self):
        return self.clss[-1]

    def DefNode(self,(tag,start,stop,subtags), buffer ):
        try:
            key, n = dispatchList(self, subtags, buffer)
        except ValueError:
            logger.exception(buffer[start:stop][:500])
            raise
        self.def_dict[key] = n
        n.name = key
        return n

    def NodewoDef(self,(tag,start,stop,subtags), buffer ):
        name = dispatch(self, subtags[0], buffer)
        cls = self.prototypes[name]
        self.clss.append(cls)
        node = self.cls()
        node._route = []
        attrs = [ a for a in dispatchList(self, subtags[1:], buffer)
                  if type(a) == Attribute ]
        node._declared = []
        for key, value in attrs:
            if type(value) == ISDef:
                self.proto_aliases.append((node._route, key, value))
            else:
                try:
                    getattr(node, key)

                except AttributeError:
                    raise RuntimeError("Invalide declaration of {0}, {1} in {2}"
                                       .format(key, value, node))
                setattr(node, key, value)
                node._declared.append(key)

        children = node.children
        for i,child in enumerate([c for c in children if isinstance(c, Node)]):
            child._parent = node
            child._route.append(i)

        # logger.debug("Created node {0}".format(node))
        self.clss.pop()
        return node

    def inline(self,(tag,start,stop,subtags), buffer ):
        attrs = dispatchList(self, subtags, buffer)
        for key, val in attrs:
            if key == 'url':
                vrml_path = val
                if self.root_path:
                    vrml_path = os.path.join(self.root_path, vrml_path)
                    if os.path.isfile(vrml_path):
                        data = parse(vrml_path)
                        return data[0]
                    else:
                        logger.exception("Couldnt find %s"%(vrml_path))

    def Proto(self,(tag,start,stop,subtags), buffer ):
        proto_name = dispatch(self, subtags[0], buffer)
        proto_attrs = {}
        base_class = Node
        base_attrs = {}

        children_tags = dispatchList(self, subtags[1:], buffer)
        # print proto_name, self.proto_aliases
        nodes = [n for n in children_tags if isinstance(n, Node)]
        if nodes[:]:
            #print buffer[start:stop]
            base_node = nodes[0]
            base_class = base_node.__class__
            base_attrs = [ (name, getattr(base_node, name)) for name in dir(base_node)
                               if not (name.startswith('_') or callable(getattr(base_node, name))
                                       )
                           ]

            def init(cls, *kargs, **kwargs):
                for name, value in base_attrs:
                    cls.__dict__[name] = copy.copy(value)
            proto_attrs['__init__'] = init



        fields = [f for f in children_tags if type(f) == tuple]

        self.proto_aliases = [a for a in self.proto_aliases if not (a[0] == [] and a[1] == a[2])]


        proto_ftypes = {}
        for ftype, fname, fdata in fields:
            proto_ftypes[fname] = ftype
            aliased = False
            for key in [a[2] for a in self.proto_aliases]:
                if key == fname:
                    aliased = True
            if not aliased:
                proto_attrs[fname] = fdata
            else:
                #print "Ignoring {0} in {1}".format(fname, proto_name)
                pass
        proto_attrs['_aliases'] = copy.deepcopy(self.proto_aliases)
        proto_attrs['_ftypes'] = copy.deepcopy(proto_ftypes)

        class Proto(base_class):
            __metaclass__ = Prototype
            name = proto_name
            attrs = proto_attrs

        self.prototypes[proto_name] = Proto

        if base_class != Node:
            logger.debug("new prototype {0} ({1}): {2}".format(proto_name,
                                                                        base_class.__name__,
                                                                        str(proto_attrs)))

        if self.proto_aliases:
             logger.debug("prototype aliases {0}: {1}".format(Proto.__name__, self.proto_aliases))
        self.proto_aliases = []
        return Proto

    def name(self,(tag,start,stop,subtags), buffer ):
        return str(buffer[start:stop])

    def Attr(self,(tag,start,stop,subtags), buffer ):
        fname = dispatch(self,subtags[0],buffer)
        ftype = self.cls.ftype(fname)
        if subtags[1][0] == "Field":
            fdata = self.parse_field(ftype, subtags[1], buffer)
            if not ftype:
                logger.exception("Unknown type for {0} in class {1}. {2}\n {3}...".
                                 format(fname, self.cls.__name__,
                                        self.cls._ftypes, buffer[start-100:stop]
                                    ))
        else:
            fdata = dispatch(self, subtags[1], buffer)
        return Attribute((fname, fdata))

    def Def(self,(tag,start,stop,subtags), buffer ):
        name = dispatch(self, subtags[0], buffer)
        return definition_str(name)


    def Field(self,(tag,start,stop,subtags), buffer ):
        l =  dispatchList(self, subtags, buffer)
        if ( len(l) == 1 and (isinstance(l[0], Number)
                              or isinstance(l[0],basestring) )):
            return l[0]
        else:
            return l

        return dispatchList(self, subtags, buffer)

    def SFNull(self,(tag,start,stop,subtags), buffer ):
        return None

    def SFNumber(self,(tag,start,stop,subtags), buffer ):
        """
        >>> parser = VrmlParser(root_node = "Fields")
        >>> parser.parse('1')[1]
        [1.0]
        """
        s = buffer[start:stop]
        return float(s)

    def SFBool(self,(tag,start,stop,subtags), buffer ):
        """
        >>> parser = VrmlParser(root_node = "Fields")
        >>> parser.parse('FALSE')[1]
        [False]
        """
        s = buffer[start:stop]
        if s.upper() == "TRUE":
            return True
        elif s.upper() == "FALSE":
            return False
        else:
            raise Exception("Failed to read a bool %s"%s)

    def SFString(self,(tag,start,stop,subtags), buffer ):
        s = buffer[start:stop]
        s=s.replace('"',"")
        return s

    def USE(self,(tag,start,stop,subtags), buffer ):
        return None
        s = buffer[start:stop]

        if s not in self.def_dict.keys():
            caller = inspect.stack()[1][3]
            raise Exception("{0} used before node definition: {1}".format(callser, s[:100]))
        return self.def_dict[s]

    def IS(self,(tag,start,stop,subtags), buffer ):
        s = buffer[start:stop]
        return ISDef(s)

    def fieldDecl(self,(tag,start,stop,subtags), buffer ):
        ftype, fname = dispatchList(self, subtags[:2], buffer)

        fdata = self.parse_field(ftype, subtags[2], buffer)
        return ftype, fname, fdata

    def parse_field(self, ftype, tag, buffer):
        fdata =  dispatch(self, tag, buffer)
        if ftype == "SFNode":
            fdata = fdata[0]
        elif ftype == "SFInt32":
            fdata = int(fdata)
        elif ftype == "MFInt32":
            fdata = [int(e) for e in fdata]
        return fdata

    dataType = name

class VrmlParser(Parser):
    def __init__(self, grammar = None, root_node = "vrmlScene"):
        path = os.path.abspath(os.path.dirname(__file__))
        if not grammar:
            grammar_file = os.path.join(path,"vrml.sbnf" )
            # print("Using grammar {0}".format(grammar_file))
            grammar = open(grammar_file).read()
        #logger.info("Grammar: {0}".format(grammar))
        Parser.__init__(self, grammar, root_node)
        logging.debug("created parser instance")
        self.root_path = ""
        self.prototypes = {}
        spec_data = open(os.path.join(path, 'standard_nodes.wrl')).read()
        self.parse(spec_data)
        logging.debug("Parsed vrml2.0 specs")

    def buildProcessor( self ):
        return VrmlProcessor(root_path = self.root_path, prototypes = self.prototypes)

    def parse_file(self, fname):
        path = os.path.abspath(os.path.dirname(fname))
        self.root_path = path
        logger.info("Parsing "+fname)
        return self.parse(open(fname).read())[1]

    def parse(self, *args, **kwargs):
        res = Parser.parse(self, *args, **kwargs)
        l = [r for r in res[1] if isinstance(r, Node)]
        count = 0
        while count < len(l):
            l += l[count].children
            count += 1
        for e in l:
            if e.__class__.__name__ == "Inline":
                url = os.path.join(self.root_path, e.url)
                logger.info("Parse inlined vrml {0}".format(url))
                e.children = Parser.parse(self, open(url).read())[1]
                for child in e.children:
                    child._parent = e
        code = "from parser import Node\n\n"
        for name, prototype in self.prototypes.items():
            obj = prototype()
            attrs = [(key, getattr(obj, key)) for key in dir(obj)
                     if not ( key.startswith("_") or callable(getattr(obj, key))
                              or key == "children")]
            code += "class {0}({1}):\n".format(name, "object")#prototype.__bases__[0].__name__)
            #print obj, dir(obj), "\n---\n", obj._ftypes, "\n---\n",attrs
            code += "    def __init__(self):\n"
            for key, value in attrs:
                code += "        self.{0} = {1} #{2}\n".format(key, repr(value),  prototype.ftype(key))
            code += "\n"
        f = open("/tmp/robotviewer_protos.py",'w')
        f.write(code)
        f.close()
        logger.debug("internally generated foloowing classes:\n{0}".format(code))
        return res[0], res[1], res[2]

def _test():
    """Inline Doctest activated. Cool! :D
    This means that whenever the module is called in python

    > python thismodule.py -v

    the doctest function will try all the tests implemented in doctest.
    """
    import doctest
    doctest.testmod()


parser = VrmlParser()
for proto_name, proto in parser.prototypes.items():
    exec("{0} = proto".format(proto_name))

logger = logging.getLogger("robotviewer.vrml.parser")
logger.addHandler(NullHandler())
logger.setLevel(logging.INFO)

def parse(filename):
    vrml_dir_path = os.path.abspath(os.path.dirname(filename))
    return parser.parse_file(filename)

def main():
    import optparse
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    sh = logging.StreamHandler()
    sh.setLevel(logging.INFO)
    logger.addHandler(sh)
    formatter = logging.Formatter("%(name)s:%(levelname)s:%(message)s")
    sh.setFormatter(formatter)

    parser = optparse.OptionParser(
        usage='\n\t%prog [options]',
        version='%%prog %s' % __version__)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")
    (options, args) = parser.parse_args(sys.argv[1:])
    if options.verbose:
        sh.setLevel(logging.DEBUG)
    if not args[:]:
        _test()
        return

    parser = VrmlParser()
    if args[:]:
        results = parser.parse_file(args[0])
        results = [r for r in results if type(r.__class__) != type]
        for result in results:
            print result#, dir(result), result.children
            if result.__class__.__name__ == "Humanoid":
                print result.humanoidBody#, result.humanoidBody[0]._parent3
            pass
if __name__ == '__main__':
    main()
