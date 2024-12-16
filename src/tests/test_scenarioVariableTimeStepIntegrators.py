#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Demonstration of how to setup and use different integrators in
#           Basilisk.  The simulation performs a 3-DOF orbit scenario.
# Author:   João Vaz Carneiro
# Creation Date:  Oct. 4, 2021
#

import inspect
import os
import sys

import pytest

import numpy as np
import numpy.testing as npt

from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import macros

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioVariableTimeStepIntegrators

@pytest.mark.scenarioTest
def test_scenarioIntegrators(show_plots):
    """This function is called by the pytest environment."""

    for integratorCase in ["rkf45", "rkf78"]:

        # each test method requires a single assert method to be called
        timeData, posData, velData, figureList = scenarioVariableTimeStepIntegrators.run(show_plots, integratorCase, 1e-8, 1e-12)

        analyticalPos = getAnalyticalSolution(posData[0,:], velData[0,:], timeData[-1] * macros.NANO2SEC)

        absTolerance = {
            "rkf45": 2000,  # m
            "rkf78": 100  # m
        }

        npt.assert_allclose(
            posData[-1, :],
            analyticalPos,
            rtol=0,
            atol=absTolerance[integratorCase],
            err_msg = "r_BN_N Vector, case: " + integratorCase
        )

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in list(figureList.items()):
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

def getAnalyticalSolution(r0, v0, dt):
    mu = simIncludeGravBody.gravBodyFactory().createEarth().mu
    oe = orbitalMotion.rv2elem(mu, r0, v0)

    E0 = orbitalMotion.f2E(oe.f, oe.e)
    M0 = orbitalMotion.E2M(E0, oe.e)
    Mf = M0 + np.sqrt(mu / oe.a / oe.a / oe.a) * dt
    Ef = orbitalMotion.M2E(Mf, oe.e)
    ff = orbitalMotion.E2f(Ef, oe.e)

    oe.f = ff

    rF, _ = orbitalMotion.elem2rv(mu, oe)
    return rF

if __name__ == "__main__":
    pytest.main([__file__, "--tb=native"])
