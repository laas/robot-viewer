
import logging, os, sys
from simpleparse.common import numbers, strings, comments
from simpleparse.parser import Parser
from simpleparse.dispatchprocessor import *
import pprint
import optparse
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger()
logger.addHandler(NullHandler())
import kinematics


def get_parser():
    usage = "usage: [options]"
    parser = optparse.OptionParser(usage=usage)

    parser.add_option("-v", "--verbose",
                      action = "store_true", dest = "verbose",
                      help = "be verbose`"
                      )


    return parser

class ObjProcessor(DispatchProcessor):
    def __init__(self, shape):
        self.shape = shape


    def basis_matrix(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching basis_matrix')
        return None


    def bevel(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching bevel')
        return None


    def c_interp(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching c_interp')
        return None


    def connectivity(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching connectivity')
        return None


    def cstype(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching cstype')
        return None


    def ctech(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching ctech')
        return None


    def curve(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching curve')
        return None


    def curve_2d(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching curve_2d')
        return None


    def d_interp(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching d_interp')
        return None


    def degree(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching degree')
        return None


    def end(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching end')
        return None


    def face(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching face')
        if self.shape.geo.idx != []:
            self.shape.geo.idx.append(-1)
        for id, texture, normal in dispatchList(self, subtags, buffer):
            id -= 1
            self.shape.geo.idx.append(id)



    def Float(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching Float')
        s = buffer[start:stop]
        return float(s)

    def ggroup(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching ggroup')
        return None


    def hole(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching hole')
        return None


    def Integer(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching Integer')
        s = buffer[start:stop]
        return int(s)


    def line(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching line')
        return None


    def lod(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching lod')
        return None


    def maplib(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching maplib')
        return None


    def matrix(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching matrix')
        return None


    def matrix_line(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching matrix_line')
        return None


    def merge(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching merge')
        return None


    def mtllib(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching mtllib')
        return None


    def normal(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching normal')
        return None


    def object_name(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching object_name')
        return None


    def ObjFile(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching ObjFile')
        return None


    def off(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching off')
        return None


    def on(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching on')
        return None


    def param(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching param')
        return None


    def parameter(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching parameter')
        return None


    def point(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching point')
        return None


    def rat(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching rat')
        return None


    def SFBool(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching SFBool')
        return None


    def SFString(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching SFString')
        return None


    def shadow_obj(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching shadow_obj')
        return None


    def slash(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching slash')
        s = buffer[start:stop]
        return s

    def smooth(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching smooth')
        return None


    def special_curve(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching special_curve')
        return None


    def special_point(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching special_point')
        return None


    def stech(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching stech')
        return None


    def step(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching step')
        return None


    def surface(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching surface')
        return None


    def texture(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching texture')
        return None


    def trace_obj(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching trace_obj')
        return None


    def trim(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching trim')
        return None


    def type(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching type')
        return None


    def u_or_v(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching u_or_v')
        return None


    def usemap(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching usemap')
        return None


    def usemtl(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching usemtl')
        return None

    def vertex(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching vertex')
        self.shape.geo.coord += dispatchList(self, subtags, buffer)

    def ver_tex(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching ver_tex')
        return None

    def ver_tex_nor(self,(tag,start,stop,subtags), buffer ):
        '''FIXME'''
        logger.debug('dispatching ver_tex_nor')
        l = dispatchList(self, subtags, buffer)
        idx = None
        tex = None
        nor = None
        idx = l[0]
        if not l[1:]:
            return [idx, tex, idx]
        if l[1] != "/":
            raise Exception ("Unexpected pattern in vertex %s"%buffer[start:stop])
        if l[2] != "/":
            tex = l[2]
            if l[4:]:
                nor = l[4]
        else:
            if l[3:]:
                nor = l[3]
        return [idx, tex, idx]






class ObjParser(Parser):
    def __init__(self, *args, **kwargs):
        Parser.__init__(self,*args, **kwargs)
        self.shape = kinematics.Shape()

    def buildProcessor(self):
        return ObjProcessor(self.shape)


def parse(filename):
    path = os.path.abspath(os.path.dirname(__file__))
    grammar_file = os.path.join(path,"obj.sbnf" )
    DEF = open(grammar_file).read()
    parser = ObjParser(DEF, 'ObjFile')
    parser.parse(open(filename).read())
    parser.shape.init()
    return [parser.shape]


def main():
    oparser = get_parser()
    opts, args = oparser.parse_args()
    logger = logging.getLogger()
    sh = logging.StreamHandler()
    if opts.verbose:
        sh.setLevel(logging.DEBUG)
        logger.setLevel(logging.DEBUG)


    logger.addHandler(sh)
    for m in parse(args[0]):
        print m


if __name__ == '__main__':
    main()

