
# ISC License
#
# Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import inspect
import os

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraftLocation
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("defaultPolarRadius", [False, True])
@pytest.mark.parametrize("defaultPlanet", [False, True])
@pytest.mark.parametrize("latitude", [55, 65, 115])
@pytest.mark.parametrize("maxRange", [-1, 3000.*1000])
@pytest.mark.parametrize("cone", [0, 1, -1])
def test_spacecraftLocation(show_plots, defaultPolarRadius, defaultPlanet, latitude, maxRange, cone):
    """
    Tests whether spacecraftLocation:

    1. defaults planet polar radius to equatorial radius if the polar radius is not set
    2. checks that the zero default planet states are used if the planet is not provided
    3. checks that the planet oblateness is accounted for
    4. checks if the optional sensor boresight axis is properly accounted for

    :return:
    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, defaultPolarRadius, defaultPlanet, latitude, maxRange, cone)
    assert testResults < 1, testMessage


def run(showplots, defaultPolarRadius, defaultPlanet, latitude, maxRange, cone):

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #   Initialize new atmosphere and drag model, add them to task
    module = spacecraftLocation.SpacecraftLocation()
    module.ModelTag = "scLocation"
    module.rEquator = orbitalMotion.REQ_EARTH * 1000.
    if not defaultPolarRadius:
        module.rPolar = orbitalMotion.REQ_EARTH * 1000. * 0.5
    if maxRange:
        module.maximumRange = maxRange
    if cone != 0:
        module.aHat_B = [0, 0, cone]
        module.theta = 80. * macros.D2R

    scSim.AddModelToTask(simTaskName, module)

    # create planet message
    planetPos = np.array([0.0, 0.0, 0.0])
    if not defaultPlanet:
        planet_message = messaging.SpicePlanetStateMsgPayload()
        planet_message.J20002Pfix = rbk.euler3(np.radians(-90.)).tolist()
        planetPos = np.array([orbitalMotion.AU * 1000, 0.0, 0.0])
        planet_message.PositionVector = planetPos
        planetMsg = messaging.SpicePlanetStateMsg().write(planet_message)
        module.planetInMsg.subscribeTo(planetMsg)

    # create primary spacecraft state message
    alt = 1000. * 1000      # meters
    scMsgData = messaging.SCStatesMsgPayload()
    r = orbitalMotion.REQ_EARTH * 1000. + alt
    scMsgData.r_BN_N = planetPos + np.array([r, 0.0, 0.0])
    scMsgData.sigma_BN = [1.0, 0.0, 0.0]
    scMsg = messaging.SCStatesMsg().write(scMsgData)
    module.primaryScStateInMsg.subscribeTo(scMsg)

    sc1MsgData = messaging.SCStatesMsgPayload()
    angle = np.radians(latitude)
    sc1MsgData.r_BN_N = planetPos + np.array([r * np.cos(angle), 0.0, r * np.sin(angle)])
    sc1Msg = messaging.SCStatesMsg().write(sc1MsgData)
    module.addSpacecraftToModel(sc1Msg)

    # Run the sim
    scSim.InitializeSimulation()

    # check polar planet radius default behavior
    if defaultPolarRadius and module.rPolar < 0:
        testFailCount += 1
        testMessages.append("FAILED: " + module.ModelTag + " Module failed default polar radius check.")

    scSim.TotalSim.SingleStepProcesses()

    accessMsg = module.accessOutMsgs[0].read()

    if latitude == 55 or (latitude == 65 and not defaultPolarRadius):
        trueAccess = 1
        if maxRange > 0:
            if accessMsg.slantRange > maxRange:
                trueAccess = 0
        if cone == 1:
            trueAccess = 0

        if accessMsg.hasAccess != trueAccess:
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed access test.")

        if accessMsg.slantRange <= 1e-6:
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed positive slant range test.")
    if (latitude == 65 and defaultPolarRadius) or latitude == 115:
        # should not have access
        if accessMsg.hasAccess != 0:
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed negative have access test.")

        if np.abs(accessMsg.slantRange) > 1e-6:
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed negative slant range test.")

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, ''.join(testMessages)]


def test_spacecraftLocationColinearAccess(show_plots):
    """
    Regression test for issues #644 / #946.

    Two spacecraft that are radially aligned (on the same ray from the planet
    center) at different altitudes have an unobstructed line of sight, so
    ``hasAccess`` must be 1.

    Before the #946 fix (commit ``d432682135``), ``rClose`` -- the closest point
    on the inter-spacecraft segment to the planet center -- was computed from the
    *unclamped* line parameter. For a radial alignment that parameter is negative,
    placing the "closest point" at the planet center (``rClose`` ~ 0), so the
    ``rClose.norm() > rEquator`` gate failed and access was wrongly denied.

    This geometry keeps every other access pathway neutral -- no planet message
    (identity orientation at the origin), a spherical planet so ``zScale == 1``,
    no range cap, no boresight cone, and no sun message -- which isolates the
    ``rClose`` computation. The existing :func:`test_spacecraftLocation` cases
    cannot catch this regression: their equal-radius geometry holds the line
    parameter at ~0.5, inside ``[0, 1]``, where the clamp is a no-op.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("simProcess")
    dynProcess.addTask(scSim.CreateNewTask("simTask", macros.sec2nano(1.)))

    module = spacecraftLocation.SpacecraftLocation()
    module.ModelTag = "scLocationColinear"
    module.rEquator = orbitalMotion.REQ_EARTH * 1000.  # [m]
    # rPolar left unset -> spherical planet (zScale = 1); no range cap, cone, or sun.
    scSim.AddModelToTask("simTask", module)

    # primary spacecraft on the +x axis, 1000 km altitude
    rPrimary = orbitalMotion.REQ_EARTH * 1000. + 1000.0e3  # [m]
    primaryMsgData = messaging.SCStatesMsgPayload()
    primaryMsgData.r_BN_N = [rPrimary, 0.0, 0.0]  # [m]
    primaryMsg = messaging.SCStatesMsg().write(primaryMsgData)
    module.primaryScStateInMsg.subscribeTo(primaryMsg)

    # other spacecraft farther out on the SAME ray -> unobstructed line of sight
    rOther = orbitalMotion.REQ_EARTH * 1000. + 2000.0e3  # [m]
    otherMsgData = messaging.SCStatesMsgPayload()
    otherMsgData.r_BN_N = [rOther, 0.0, 0.0]  # [m]
    otherMsg = messaging.SCStatesMsg().write(otherMsgData)
    module.addSpacecraftToModel(otherMsg)

    scSim.InitializeSimulation()
    scSim.TotalSim.SingleStepProcesses()

    accessMsg = module.accessOutMsgs[0].read()

    assert accessMsg.hasAccess == 1, (
        "radially-aligned spacecraft must have line-of-sight access (issues #644/#946); "
        "hasAccess=0 indicates the rClose clamp-ordering regression"
    )
    # for a pure radial separation the slant range is just the altitude difference
    np.testing.assert_allclose(accessMsg.slantRange, rOther - rPrimary, rtol=1e-6)


if __name__ == '__main__':
    run(False
        , False      # defaultPolarRadius
        , True     # defaultPlanet
        , 55        # true latitude angle (deg)
        , -7000.*1000 # max range
        , 1            # cone case, 0-> no cone, 1 -> [0, 1, 0], -1 -> [0, -1, 0]
        )
    test_spacecraftLocationColinearAccess(False)
