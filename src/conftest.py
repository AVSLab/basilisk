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
# where to put test figures relative to bsk src/ in order to gen pytest-html report with figs
testFigsDir = './assets/testFigs/'
if os.path.exists(testFigsDir):  # we specifically want to delete pre-existing figures
    shutil.rmtree(testFigsDir)
    os.makedirs(testFigsDir)
else:
    os.makedirs(testFigsDir)

def pytest_addoption(parser):
    parser.addoption("--show_plots", action="store_true",
                     help="test(s) shall display plots")


@pytest.fixture(scope="module")
def show_plots(request):
    return request.config.getoption("--show_plots")


def get_test_name(item):  # just get the name of the test from the item function
    ugly_name = str(item.function)
    less_ugly_name = ugly_name.strip('<function')
    beautiful_name = less_ugly_name.split(' at')[0]
    return beautiful_name


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """
        We use this function to do two things:
        1) append the docstrings to the test log
        2) print test plots to the test log

        This is kept neat by inserting a table into the extras to separate the two things.
    """
    pytest_html = item.config.pluginmanager.getplugin('html')
    outcome = yield
    report = outcome.get_result()
    extra = getattr(report, 'extra', [])

    extra.append(pytest_html.extras.html('</div><table><tr><td><div>'))
    # add the doc string
    if item.function.__doc__:
        docstring = '<tt>' + str(item.function.__doc__).replace('\n', '</br>', ) + '</tt>'
    else:
        docstring = '<tt> This test does not have a docstring </br></tt>'
    extra.append(pytest_html.extras.html(docstring))
    extra.append(pytest_html.extras.html('</div></td></tr><tr><td><div>'))

    # save the figures
    dir_name = testFigsDir + get_test_name(item)
    i = 0
    if len(plt.get_fignums()) > 0:
        while i > -1:
            dir_name_num = dir_name + '_' + str(i) + '/'
            if not os.path.exists(dir_name_num):
                os.mkdir(dir_name_num)
                break
            else:
                i += 1
                continue

        for f in plt.get_fignums():
            filename = dir_name_num + 'figure_' + str(f) + '.png'
            plt.figure(f).savefig(filename)
            plt.close(f)  # close figures so test writers don't have to
            extra.append(pytest_html.extras.image(filename))
    else:
        extra.append(pytest_html.extras.html('<tt> This test has no images.</br></tt>'))
    extra.append(pytest_html.extras.html('</div></td></tr></table><div>'))
    report.extra = extra

def pytest_html_results_table_header(cells):
    cells.pop()

def pytest_html_results_table_row(report, cells):
    cells.pop()


