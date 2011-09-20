# -*- coding: utf-8 -*-

"""build_manpage command -- Generate man page from setup()"""

import datetime
from distutils.command.build import build
from distutils.core import Command
from distutils.errors import DistutilsOptionError
import optparse


class build_manpage(Command):

    description = 'Generate man page from setup().'

    user_options = [
        ('outputs=', 'O', 'output file'),
        ('parsers=', None, 'module path to optparser (e.g. mymod:func'),
        ('appnames=', None, 'appname'),
        ]

    def initialize_options(self):
        self.outputs = ""
        self.parsers= ""
        self.appnames = ""
        self.output = None
        self.parser = None
        self.appname = None


    def finalize_options(self):
        self.outputs = self.outputs.split()
        self.parsers = self.parsers.split()
        self.appnames = self.appnames.split()
        if self.outputs is []:
            raise DistutilsOptionError('\'output\' option is required')
        if self.parsers is []:
            raise DistutilsOptionError('\'parser\' option is required')

    def _markup(self, txt):
        return txt.replace('-', '\\-')

    def _write_header(self):
        appname = self.appname
        if not appname:
            appname = self.distribution.get_name()
        ret = []
        ret.append('.TH %s 1 %s\n' % (self._markup(appname),
                                      self._today.strftime('%Y\\-%m\\-%d')))

        description = self._parser.description
        #description = self.distribution.get_description()

        if description:
            name = self._markup('%s - %s' % (self._markup(appname),
                                             description.splitlines()[0]))
        else:
            name = self._markup(appname)
        ret.append('.SH NAME\n%s\n' % name)
        synopsis = self._parser.get_usage()
        if synopsis:
            synopsis = synopsis.replace('%s ' % appname, '')
            ret.append('.SH SYNOPSIS\n.B %s\n%s\n' % (self._markup(appname),
                                                      synopsis))
        #long_desc = self.distribution.get_long_description()
        #if long_desc:
        ret.append('.SH DESCRIPTION\n%s\n' % self._markup(description))
        return ''.join(ret)

    def _write_options(self):
        ret = ['.SH OPTIONS\n']
        ret.append(self._parser.format_option_help())
        return ''.join(ret)

    def _write_footer(self):
        ret = []
        appname = self.distribution.get_name()
        author = '%s <%s>' % (self.distribution.get_author(),
                              self.distribution.get_author_email())
        ret.append(('.SH AUTHORS\n.B %s\nwas written by %s.\n'
                    % (self._markup(appname), self._markup(author))))
        homepage = self.distribution.get_url()
        ret.append(('.SH DISTRIBUTION\nThe latest version of %s may '
                    'be downloaded from\n'
                    '.UR %s\n.UE\n'
                    % (self._markup(appname), self._markup(homepage),)))
        return ''.join(ret)

    def run(self):
        for i,output in enumerate(self.outputs):
            self.output = self.outputs[i]
            self.parser = self.parsers[i]
            self.appname = self.appnames[i]

            mod_name, func_name = self.parser.split(':')
            fromlist = mod_name.split('.')
            try:
                mod = __import__(mod_name, fromlist=fromlist)
                self._parser = getattr(mod, func_name)()
            except ImportError, err:
                raise
            self._parser.formatter = ManPageFormatter()
            self._parser.formatter.set_parser(self._parser)
            self.announce('Writing man page %s' % self.output)
            self._today = datetime.date.today()

            manpage = []
            manpage.append(self._write_header())
            manpage.append(self._write_options())
            manpage.append(self._write_footer())
            stream = open(self.output, 'w')
            stream.write(''.join(manpage))
            stream.close()


class ManPageFormatter(optparse.HelpFormatter):

    def __init__(self,
                 indent_increment=2,
                 max_help_position=24,
                 width=None,
                 short_first=1):
        optparse.HelpFormatter.__init__(self, indent_increment,
                                        max_help_position, width, short_first)

    def _markup(self, txt):
        return txt.replace('-', '\\-')

    def format_usage(self, usage):
        return self._markup(usage)

    def format_heading(self, heading):
        if self.level == 0:
            return ''
        return '.TP\n%s\n' % self._markup(heading.upper())

    def format_option(self, option):
        result = []
        opts = self.option_strings[option]
        result.append('.TP\n.B %s\n' % self._markup(opts))
        if option.help:
            help_text = '%s\n' % self._markup(self.expand_default(option))
            result.append(help_text)
        return ''.join(result)




