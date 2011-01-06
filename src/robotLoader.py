#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang
import re
import vrml_loader,xml_loader

def robotLoader(filename,MeshBool):
    if re.search(r"\.wrl$",filename):
        return vrml_loader.VRMLloader(filename,MeshBool)

    elif re.search(r"\.xml$",filename):
        return xml_loader.XMLloader(filename)
    return None
