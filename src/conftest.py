''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''

import pytest
import matplotlib.pyplot as plt
import os
import shutil
import sys
import subprocess
import time

reqs = subprocess.check_output([sys.executable, '-m', 'pip', 'freeze'])
installed_packages = [r.decode().split('==')[0] for r in reqs.split()]
make_report = False
if 'pytest-html' in installed_packages:
    make_report = ('--report' in sys.argv)
report_dir = 'tests/report/'  # relative to src/
rel_fig_dir = 'assets/testFigs/'  # relative to report/
testFigsDir = report_dir + rel_fig_dir  # can assume this is run from basilisk/src/

# remove the old report because we don't want stale data around
if os.path.exists(report_dir):
    shutil.rmtree(report_dir)
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
        print('HTML report file:', 'basilisk/src/' + html_file)
        args.extend(['--html', html_file])


def pytest_addoption(parser):
    parser.addoption("--show_plots", action="store_true",
                     help="test(s) shall display plots")
    parser.addoption("--report", action="store_true",  # --report is easier, more controlled than --html=<pathToReport>
                     help="whether or not to gen a pytest-html report. "
                          "The report is saved in src/tests/report")


@pytest.fixture(scope="module")
def show_plots(request):
    return request.config.getoption("--show_plots")


def get_test_name(item):  # just get the name of the test from the item function
    return str(item.function).split(' ')[1]


def get_docstring(item):
    if item.function.__doc__:
        return '<tt>' + str(item.function.__doc__).replace('\n', '</br>', ) + '</tt>'
    else:
        return '<tt> This test does not have a docstring </br></tt>'


def do_report():  # this is necessary to get make_report into the pytest_runtest_makereport hook
    return make_report


if make_report:  # only do this if the report is requested AND pytest-html is installed.
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
        if do_report:  # don't save pictures etc. if not making a report
            report = outcome.get_result()
            extra = getattr(report, 'extra', [])

            # make the table *within* the extras cell with a diff row per docstring/figures
            # the extra <div> throughout this function are to stop pytest_html from making everything its own div
            extra.append(pytest_html.extras.html('</div><table><tr><td><div>'))
            # add the doc string
            extra.append(pytest_html.extras.html(get_docstring(item)))
            extra.append(pytest_html.extras.html('</div></td></tr><tr><div>'))

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
                        time.sleep(.02)
                    filename = dir_name_num + 'figure_' + str(f) + '.png'
                    plt.figure(f).savefig(filename)
                    plt.close(f)  # close figures so test writers don't have to. prevents saving same image multiple times
                    img_src = 'assets' + filename.split('assets')[1]  # just want a relative (to report) path here
                    extra.append(pytest_html.extras.html('</div><td><div class="image"><a href="' + img_src +
                                                         '"><img src="' + img_src + '"/></a></div></td><div>'))
            else:
                extra.append(pytest_html.extras.html('<tt> This test has no images.</br></tt>'))
            extra.append(pytest_html.extras.html('</div></tr></table><div>'))  # this finishes off our fancy html table
            report.extra = extra


    def pytest_html_results_table_header(cells):
        # remove the "links" column from the report
        cells.pop()


    def pytest_html_results_table_row(report, cells):
        # remove the "links column from the report
        cells.pop()


