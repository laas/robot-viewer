#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang
import re
import VRMLloader,XMLloader

def robotLoader(filename,MeshBool):
    if re.search(r"\.wrl$",filename):
        return VRMLloader.VRMLloader(filename,MeshBool)

    elif re.search(r"\.xml$",filename):
        return XMLloader.XMLloader(filename)
    return None
