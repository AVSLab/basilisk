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

#
# Basilisk Integrated Test
#
# Purpose:  Integrated test of the MonteCarlo module.  Runs multiple
#           scenarioAttitudeFeedbackRW with dispersed initial parameters
#

USE_DATESHADER = True
import sys, os, inspect
import pytest
try:
    import datashader
    import holoviews
    import pandas
    import bokeh
except ImportError:
    USE_DATESHADER = False

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioMonteCarloAttRW

@pytest.mark.parametrize("MCCases, datashader",
                         [(1, USE_DATESHADER),
                          (2, USE_DATESHADER)]) # Case 1 for normal MC runs, case 2 for running ICs

@pytest.mark.slowtest()
def test_MonteCarloSimulation(show_plots, MCCases, datashader):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    scenarioMonteCarloAttRW.run(True, MCCases , show_plots, datashader)
    return

