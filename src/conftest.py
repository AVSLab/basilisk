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

import inspect
import os
import shutil
import subprocess
import sys

import pytest

def pytest_configure(config):
    config.addinivalue_line(
        "markers", "ciLinuxOnlySkip: skip test on Linux CI builds"
    )

def pytest_runtest_setup(item):
    if "ciLinuxOnlySkip" in item.keywords:
        if sys.platform.startswith("linux") and os.getenv("CI") == "true":
            pytest.skip("Skipped on Linux CI environment")

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
print(path)

# remove the old report because we don't want stale data around, even without pytest-html
# for more see reportconf.py
if os.path.exists('tests/report/'):
    shutil.rmtree('tests/report/')


def pytest_addoption(parser):
    parser.addoption("--show_plots", action="store_true",
                     help="test(s) shall display plots")
    parser.addoption("--report", action="store_true",  # --report is easier, more controlled than --html=<pathToReport>
                         help="whether or not to gen a pytest-html report. The report is saved in ./tests/report")


@pytest.fixture(scope="module")
def show_plots(request):
    return request.config.getoption("--show_plots")

# we don't want to reconfigure pytest per pytest-html unless we have it
# for more on this, see the reportconf.py file.
reqs = subprocess.check_output([sys.executable, '-m', 'pip', 'freeze'])
installed_packages = [r.decode().split('==')[0] for r in reqs.split()]

if ('--report' in sys.argv) and ('pytest-html' not in installed_packages):
    print('ERROR: you need to pip install pytest-html package to use the --report flag')
    quit()

if 'pytest-html' in installed_packages:
    exec(open(path + "/reportconf.py").read(), globals())
