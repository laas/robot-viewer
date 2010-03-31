#!/usr/bin/env python
import os
try:
    import simpleparse
except :
    cmd="""wget 'http://sourceforge.net/projects/simpleparse/files/simpleparse/2.1.1a2/SimpleParse-2.1.1a2.tar.gz/download' &&
	tar xvf SimpleParse-2.1.1a2.tar.gz &&
	cd SimpleParse-2.1.1a2 &&
	python setup.py install --prefix ${ROBOTPKG_BASE}"""

    os.system(cmd)

