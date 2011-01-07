#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang
import re
import vrml_loader,xml_loader, kxml_loader

def robotLoader(filename, load_mesh_bool):
    re_ext = re.compile(r"\.(?P<EXT>[A-Za-z]+)$")
    m = re_ext.search(filename)
    if not m:
        raise Exception("Couldn't find file extension of %s"%filename)
    ext = m.group('EXT')
    if ext in  ["vrml","wrl"]:
        return vrml_loader.load(filename,load_mesh_bool)

    elif ext == "xml":
        return xml_loader.load(filename)

    elif ext == "kxml":
        return kxml_loader.load(filename)


    else:
        raise Exception("Unknown extension %s"%ext)
