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


#
#   Unit Test Script
#   Module Name:        simpleSolarPanel
#   Author:             Andrew Harris
#   Creation Date:      July 17th 2019
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.architecture import messaging
from Basilisk.simulation import simpleSolarPanel
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import astroFunctions


@pytest.mark.parametrize("orbitDistance", [1000.*astroFunctions.AU, 1000.*1.52*astroFunctions.AU])
@pytest.mark.parametrize("eclipseValue", [0,1])
@pytest.mark.parametrize("scAttitude", [[0,0,0], rbk.C2MRP(rbk.euler3212C([0,np.radians(60.),0])), rbk.C2MRP(rbk.euler3212C([0,np.radians(90.),0]))])
def test_simpleSolarPanel(show_plots, orbitDistance, eclipseValue, scAttitude):
    """
    **Validation Test Description**

    Unit test for simpleSolarPanel. The unit test specifically covers:

    1. Shadowing: Does the panel correctly reflect shadowing from eclipse or attitude?

        This is tested by setting an eclipse value to either 0 or 1 and verifying that the panel produces power or not.

    2. Attitude dependence: Does the spacecraft power output correctly evaluate given the spacecraft attitude?

        This is tested at three values of the attitude corresponding to sun-facing, a 60 degree rotation away
        from the sun (half power), and a 90 degree rotation away from the sun (expected power).

    3. Orbit dependence: Does distance from the sun impact power generation?

        This is evaluated by testing the solar panel at both Earth (1AU) and Mars (1.52 AU) to see whether
        power production drops.

    Nominal power generation when face-on towards the Sun at earth is set to be 1372.5398 W, which assumes
    a 1m^2 solar panel that operates at perfect efficiency.
    """

    panelResults, panelMessage = run(show_plots, orbitDistance, eclipseValue, scAttitude)

    assert panelResults < 1, [panelMessage]

def run(showPlots, orbitDistance, eclipseValue, scAttitude):

    #   Test initialization
    testFailCount = 0                      
    testMessages = []                      
    unitTaskName = "unitTask"             
    unitProcessName = "TestProcess"

    #   Specify test-against parameter
    referencePower = astroFunctions.solarFluxEarth  # W/m^2 at Earth
    sunDistanceMult = pow(astroFunctions.AU*1000., 2.)/pow(orbitDistance, 2.)
    scAttMult = np.cos(abs(np.arccos(0.5 * (np.trace(rbk.MRP2C(scAttitude))-1.))))  # extract cos(prv) to determine the attitude angle vs the sun
    referenceMultiplier = 1.0 * eclipseValue * sunDistanceMult * scAttMult  # Nominally set to 1.0; modified by other vals

    #   Simulation set-up
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(1.0)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    #   Input message set-up
    eclipseMessage = messaging.EclipseMsgPayload()
    eclipseMessage.shadowFactor = eclipseValue  # Set it to be totally in shadow
    eclipseMsg = messaging.EclipseMsg().write(eclipseMessage)


    sunMessage = messaging.SpicePlanetStateMsgPayload()
    sunMessage.PlanetName = "Sun"
    sunMessage.PositionVector = [0, 0, 0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunMessage)

    scMessage = messaging.SCStatesMsgPayload()
    scMessage.r_BN_N = [-orbitDistance, 0, 0]
    scMessage.sigma_BN = scAttitude
    scMsg = messaging.SCStatesMsg().write(scMessage)

    #   Module set-up
    panel = simpleSolarPanel.SimpleSolarPanel()
    panel.setPanelParameters(np.array([1, 0, 0]), 1.0, 1.0)
    panel.stateInMsg.subscribeTo(scMsg)
    panel.sunEclipseInMsg.subscribeTo(eclipseMsg)
    panel.sunInMsg.subscribeTo(sunMsg)

    unitTestSim.AddModelToTask(unitTaskName, panel)
    
    dataLog = panel.nodePowerOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    #   Execute the sim for 1 second.
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    
    unitTestSim.ExecuteSimulation()

    powerData = dataLog.netPower

    tol = 1e-7

    if not unitTestSupport.isDoubleEqual(powerData[1], referencePower*referenceMultiplier, tol):
        testFailCount += 1
        testMessages.append('Error: simpleSolarPanel did not compute power correctly.')
    
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    print(test_simpleSolarPanel(False, 1000.*astroFunctions.AU, 1, rbk.C2MRP(rbk.euler3212C([0,np.radians(60.),0]))))
    print(test_simpleSolarPanel(False, 1000.*astroFunctions.AU, 1, [0,0,0]))
    print(test_simpleSolarPanel(False, 1.52*1000.*astroFunctions.AU, 1, [0, 0, 0]))
    print(test_simpleSolarPanel(False, 1000.*astroFunctions.AU, 0, [0,0,0]))
