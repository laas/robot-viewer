#! /usr/bin/env python

__author__ = "Duong Dang"
__version__ = "0.1"

import logging, sys, os
from BeautifulSoup import BeautifulSoup
class NullHandler(logging.Handler):
    def emit(self, record):
        pass

logger = logging.getLogger("vrml_specs_parser")
logger.addHandler(NullHandler())
logger.setLevel(logging.DEBUG)
import urllib2
import HTMLParser

def main():
    import optparse
    parser = optparse.OptionParser(
        usage='\n\t%prog [options]',
        version='%%prog %s' % __version__)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="be verbose")
    (options, args) = parser.parse_args(sys.argv[1:])
    spec_url = "http://graphcomp.com/info/specs/sgi/vrml/spec/part1/nodesRef.html"
    f = urllib2.urlopen(spec_url)
    soup = BeautifulSoup(f)
    h2s =  soup.findAll('h2')

    for h2 in h2s:
        node_name = h2.find('a').contents[0].strip()
        # print node_name
        pre = h2.nextSibling.nextSibling
        s = ''.join(pre.findAll(text=True))
        # print pre.name
        h = HTMLParser.HTMLParser()
        s = h.unescape(s)
        print s


if __name__ == '__main__':
    main()
