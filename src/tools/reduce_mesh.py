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

import logging, sys
logger = logging.getLogger("robotviewer.reduce_mesh")

import re,os
import tempfile
import scipy.io
import subprocess, time, signal
def reduce(mesh_verts, mesh_idxs, factor = 0.1):
    """
    """
    matlab_script = ""
    matlab_script += "verts = ["
    for i,p in enumerate(mesh_verts):
        matlab_script += "%f "%p
        if i % 3 == 2 and mesh_verts[i+1:]:
            matlab_script += ";\n"
    matlab_script += "];\n"

    # Remove -1s and non triangular indexes
    i = 0
    idxs = []
    poly = []
    script_file = tempfile.mkstemp()[1]
    mat_file = tempfile.mkstemp()[1]
    for a_idx in mesh_idxs:
        if a_idx != -1:
            poly.append(a_idx)
            continue

        # idx=-1
        if len(poly)!=3:
            raise Exception("""oops not a triangle, %s.
                                  Only support triangle mesh for the moment"""%str(poly))
            poly=[]
            continue
        # idx=-1 and poly is a triangle
        idxs += poly
        poly = []
    # print len(idxs)
    matlab_script += "faces = ["
    for i,p in enumerate(idxs):
        matlab_script += "%d "%(p+1)
        if i % 3 == 2 and idxs[i+1:]:
            matlab_script += ";\n"
    srcpath = os.path.abspath(os.path.dirname(__file__))

    matlab_script += "];\n"
    matlab_script += """
path(path,'%s')
[nf,nv]=reduce_mesh(verts, faces, %f);
save('%s.mat','nf','nv')
"""%(srcpath,factor,  os.path.basename(mat_file))
    f=open("%s.m"%script_file,'w')
    f.write(matlab_script)
    f.close()
    cmd = "cd /tmp && matlab -nosplash 	-nojvm -r %s"%os.path.basename(script_file)
    mat_proc = subprocess.Popen(cmd, shell = True, preexec_fn=os.setsid)
    while not os.path.isfile("%s.mat"%mat_file):
        time.sleep(0.5)
    os.killpg(mat_proc.pid,signal.SIGTERM)

    mat_contents = scipy.io.loadmat("%s.mat"%mat_file)
    return mat_contents['nv'], mat_contents['nf']



def main():
    import optparse
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    parser = optparse.OptionParser(
        usage='\n\t%prog [options] input',
        version='%%prog %s' % __version__)

    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")

    parser.add_option("-o", "--output", type = "string",default = None,
                      action="store", dest="output",
                      help="specify output default: overwrite input")

    parser.add_option("-f", "--factor", type = "float",
                      action="store", dest="factor", default = 0.1,
                      help="specify the fraction of the original number of faces to be kept")


    (options, args) = parser.parse_args(sys.argv[1:])

    input_file = args[0]

    data = open(input_file).read()
    re_coord = re.compile(r"point\s*\[(?P<POINTS>[^]]+)")
    re_idx = re.compile(r"coordIndex\s*\[(?P<IDX>[^]]+)")
    coords = re_coord.findall(data)
    idxs = re_idx.findall(data)

    if len(coords) != len(idxs):
        raise Exception("Number of coords and idxs not equal")

    for i, coord_str in enumerate(coords):
        idx_str = idxs[i]
        coord = [ float(w) for w in coord_str.split()]
        idx = [ int(w) for w in idx_str.replace(","," ").split()]
        new_coord, new_idx = reduce(coord, idx,options.factor)
        new_coord_str = " ".join(["%f %f %f \n"%(p[0], p[1], p[2]) for p in new_coord])
        new_idx_str = ""
        for j, b in enumerate(new_idx):
            new_idx_str+= " %d %d %d -1,\n"%(b[0]-1,b[1]-1,b[2]-1)

        data = data.replace(coord_str, new_coord_str)
        data = data.replace(idx_str, new_idx_str)

    if not options.output:
        options.output = input_file
    f = open(options.output, 'w')
    f.write(data)
    f.close()


if __name__ == '__main__':
    main()
