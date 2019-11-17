#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

"""
This reportconf.py file is executed by src/conftest.py if and only if
the user has pytest-html installed. It is used to customize the contents of 
the pytest-html report and control where it is written.
"""

import pytest
import matplotlib.pyplot as plt
import os
import sys
import time
import textwrap

make_report = ('--report' in sys.argv)
report_dir = 'tests/report/'  # relative to current folder
rel_fig_dir = 'assets/testFigs/'  # relative to report/
testFigsDir = report_dir + rel_fig_dir  # can assume this is run from basilisk/src/
if make_report:
    os.makedirs(testFigsDir)


def pytest_cmdline_preparse(config, args):
    # this is a pytest function we can use to adjust args
    # we want to convert --report into --html=<pathToReport>
    # we don't want the user to be able to call --html=<pathToReport> directly,
    # because we have hardcoded some paths to save figures etc and --report is cleaner
    for arg in args:
        if '--html' in arg:
            print('Please use --report with no arguments instead of --html and try again')
            quit()
    if '--report' in args:
        args.remove('--report')
        html_file = report_dir + 'BasiliskTestReport.html'
        args.extend(['--html', html_file])



def get_test_name(item):  # just get the name of the test from the item function
    return str(item.function).split(' ')[1]


def get_docstring(item):
    if item.function.__doc__:
        return '<span style="font-family:monospace;white-space:pre-wrap;word-wrap: break-word;">' \
               + textwrap.dedent(str(item.function.__doc__)).replace('``', '') + '</span>'
    else:
        return '<span style="font-family:monospace;white-space:pre-wrap;word-wrap: break-word;"> ' \
               'This test does not have a docstring </br></span>'


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """
        We (Basilisk) use this function to do two things:
        1) append the docstrings to the test log extras
        2) print test plots to the test log extras

        This is kept neat by inserting a table into the extras to separate the two things.
    """
    pytest_html = item.config.pluginmanager.getplugin('html')
    outcome = yield
    if item.config.option.htmlpath:  # don't save pictures etc. if not making a report

        report = outcome.get_result()
        extra = getattr(report, 'extra', [])

        # add the doc string
        extra.append(pytest_html.extras.html(get_docstring(item)))

        # save the figures
        dir_name = testFigsDir + get_test_name(item)
        i = 0
        if len(plt.get_fignums()) > 0:
            while i > -1:  # this loops makes a numbered directory per run of the same test to avoid overwrite
                dir_name_num = dir_name + '_' + str(i) + '/'
                if not os.path.exists(dir_name_num):
                    os.makedirs(dir_name_num)
                    break
                else:
                    i += 1
                    continue

            for f in plt.get_fignums():
                if not os.path.exists(dir_name_num):
                    time.sleep(0.02)
                filename = dir_name_num + 'figure_' + str(f) + '.svg'
                plt.figure(f).savefig(filename, transparent=True)
                plt.close(f)  # prevents saving same image multiple times
                img_src = 'assets' + filename.split('assets')[1]  # just want a relative (to report) path here
                extra.append(pytest_html.extras.html('<a href="' + img_src +'"><img src="' + img_src +
                                                     '" style="width:30em;float:left;"></a>'))
        else:
            extra.append(pytest_html.extras.html('<tt> This test has no images.<br></tt>'))
        extra.append(pytest_html.extras.html('<br style="clear:left;">'))
        report.extra = extra


def pytest_html_results_table_header(cells):
    # remove the "links" column from the report
    cells.pop()


def pytest_html_results_table_row(report, cells):
    # remove the "links column from the report
    cells.pop()


