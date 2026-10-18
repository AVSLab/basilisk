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
#   Module Name:        meanOEFeedback
#   Author:             Hirotaka Kondo
#   Creation Date:      March 27, 2020
#

import numpy as np
import pytest
from Basilisk.architecture import bskLogging, messaging
from Basilisk.fswAlgorithms import meanOEFeedback  # import the module that is to be tested
# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("useClassicElem", [True, False])
@pytest.mark.parametrize("accuracy", [1e-6])

def test_meanOEFeedback(show_plots, useClassicElem, accuracy):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = meanOEFeedbackTestFunction(show_plots, useClassicElem, accuracy)
    assert testResults < 1, testMessage


def meanOEFeedbackTestFunction(show_plots, useClassicElem, accuracy):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    # Create a sim meanOEFeedback as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # process rate
    testProc = unitTestSim.CreateNewProcess(unitProcessName)  # create new process
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))  # create new task
    # Construct algorithm and associated C++ container
    module = meanOEFeedback.meanOEFeedback()
    module.ModelTag = "meanOEFeedback"  # update python name of test meanOEFeedback
    module.targetDiffOeMean = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    module.mu = orbitalMotion.MU_EARTH * 1e9  # [m^3/s^2]
    module.req = orbitalMotion.REQ_EARTH * 1e3  # [m]
    module.J2 = orbitalMotion.J2_EARTH      # []
    module.K = [1e7, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 1e7, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 1e7, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 1e7, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 1e7, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 1e7]
    if(useClassicElem):
        module.oeType = 0  # 0: classic
    else:
        module.oeType = 1  # 1: equinoctial
    # Add test meanOEFeedback to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)
    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    #
    # Chief Navigation Message
    #
    oe = orbitalMotion.ClassicElements()
    oe.a = 20000e3  # [m]
    oe.e = 0.1
    oe.i = 0.2
    oe.Omega = 0.3
    oe.omega = 0.4
    oe.f = 0.5
    (r_BN_N, v_BN_N) = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH*1e9, oe)
    chiefNavStateOutData = messaging.NavTransMsgPayload()  # Create a structure for the input message
    chiefNavStateOutData.timeTag = 0
    chiefNavStateOutData.r_BN_N = r_BN_N
    chiefNavStateOutData.v_BN_N = v_BN_N
    chiefNavStateOutData.vehAccumDV = [0, 0, 0]
    chiefInMsg = messaging.NavTransMsg().write(chiefNavStateOutData)

    #
    # Deputy Navigation Message
    #
    oe2 = orbitalMotion.ClassicElements()
    oe2.a = (1 + 0.0006) * 7000e3  # [m]
    oe2.e = 0.2 + 0.0005
    oe2.i = 0.0 + 0.0004
    oe2.Omega = 0.0 + 0.0003
    oe2.omega = 0.0 + 0.0002
    oe2.f = 0.0001
    (r_BN_N2, v_BN_N2) = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH*1e9, oe2)
    deputyNavStateOutData = messaging.NavTransMsgPayload()  # Create a structure for the input message
    deputyNavStateOutData.timeTag = 0
    deputyNavStateOutData.r_BN_N = r_BN_N2
    deputyNavStateOutData.v_BN_N = v_BN_N2
    deputyNavStateOutData.vehAccumDV = [0, 0, 0]
    deputyInMsg = messaging.NavTransMsg().write(deputyNavStateOutData)

    # Setup logging on the test meanOEFeedback output message so that we get all the writes to it
    dataLog = module.forceOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.chiefTransInMsg.subscribeTo(chiefInMsg)
    module.deputyTransInMsg.subscribeTo(deputyInMsg)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(testProcessRate)  # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    forceOutput = dataLog.forceRequestInertial

    # set the filtered output truth states
    if useClassicElem:
        trueVector = [[-849.57347406544340628897771239280701,
                       1849.77641265032843875815160572528839,
                       136.07817734479317550722043961286545]]
    else:
        trueVector = [[-1655.37188207880308254971168935298920,
                       1788.61776379042521512019447982311249,
                       52.54814237453938119415397522971034]]

    # compare the meanOEFeedback results to the truth values
    for i in range(0, len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(forceOutput[i], trueVector[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed "
                                + ".forceRequestInertial" + " unit test at t="
                                + str(dataLog.times()[i]*macros.NANO2SEC) + "sec\n")

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
        print("This test uses an accuracy value of " + str(accuracy))

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


def _equinoctial_force(eccentricity, inclination, j2, chief_anomaly, deputy_anomaly, deputy_offset=0.0):
    """Run two controller updates from supplied elliptic Cartesian states.

    Anomalies and inclination are in radians; ``deputy_offset`` changes the
    deputy semimajor axis in meters. The output is inertial force in newtons.
    """
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("equatorialProcess")
    step = macros.sec2nano(0.1)  # [ns]
    process.addTask(simulation.CreateNewTask("equatorialTask", step))
    module = meanOEFeedback.meanOEFeedback()
    module.ModelTag = "equatorialFeedback"
    module.oeType = 1
    module.mu = orbitalMotion.MU_EARTH * 1e9  # [m^3/s^2]
    module.req = orbitalMotion.REQ_EARTH * 1e3  # [m]
    module.J2 = j2
    module.K = (1e7 * np.eye(6)).ravel().tolist()  # [N*m/s]; normalized-element gain
    module.targetDiffOeMean = [0.0] * 6
    simulation.AddModelToTask("equatorialTask", module)
    messages = []
    for anomaly, offset in [(chief_anomaly, 0.0), (deputy_anomaly, deputy_offset)]:
        elements = orbitalMotion.ClassicElements()
        elements.a = 8e6 + offset  # [m]
        elements.e = eccentricity
        elements.i = inclination
        elements.Omega = 0.0  # [rad]
        elements.omega = 0.4  # [rad]
        elements.f = anomaly
        payload = messaging.NavTransMsgPayload()
        payload.r_BN_N, payload.v_BN_N = orbitalMotion.elem2rv(module.mu, elements)
        messages.append(messaging.NavTransMsg().write(payload))
    module.chiefTransInMsg.subscribeTo(messages[0])
    module.deputyTransInMsg.subscribeTo(messages[1])
    recorder = module.forceOutMsg.recorder()
    simulation.AddModelToTask("equatorialTask", recorder)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(step)
    simulation.ExecuteSimulation()
    return np.asarray(recorder.forceRequestInertial)


@pytest.mark.parametrize("eccentricity", [0.0, 0.1])
@pytest.mark.parametrize("inclination", [0.0, 1e-8])  # [rad]
@pytest.mark.parametrize("j2", [0.0, orbitalMotion.J2_EARTH])
def test_equinoctial_equatorial_identity(eccentricity, inclination, j2):
    """Identical equatorial states must command finite zero force in equinoctial mode."""
    force = _equinoctial_force(eccentricity, inclination, j2, 0.3, 0.3)
    assert np.all(np.isfinite(force))
    np.testing.assert_array_equal(force, np.zeros_like(force))


@pytest.mark.parametrize("inclination", [0.0, 1e-8, 0.7])  # [rad]
@pytest.mark.parametrize("anomaly", [0.3, -1e-9, np.pi - 1e-9])  # [rad]
def test_equinoctial_anomaly_revolutions(inclination, anomaly):
    """Equivalent chief/deputy states produce the same nonzero controller force."""
    force = _equinoctial_force(0.1, inclination, orbitalMotion.J2_EARTH, anomaly, anomaly + 0.0002, 100.0)
    wrapped = _equinoctial_force(0.1, inclination, orbitalMotion.J2_EARTH,
                                anomaly + 4.0 * np.pi, anomaly + 0.0002 - 6.0 * np.pi, 100.0)
    assert np.all(np.isfinite(force)) and np.all(np.isfinite(wrapped))
    assert np.linalg.norm(force) > 0.0
    np.testing.assert_allclose(wrapped, force, rtol=1e-8, atol=1e-8)  # [N] absolute tolerance


@pytest.mark.parametrize("j2", [-1e-3, np.nan, np.inf])
def test_equinoctial_invalid_j2(j2):
    """Accepting the zero-J2 identity must not admit invalid coefficients."""
    with pytest.raises(bskLogging.BasiliskError, match="J2 must be finite and nonnegative"):
        _equinoctial_force(0.1, 0.0, j2, 0.3, 0.3)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_meanOEFeedback(
        False,  # show_plots
        True,  # useClassicElem
        1e-6    # accuracy
    )


@pytest.mark.parametrize("inclination,message", [
    (np.arccos(1/np.sqrt(5)), "invalid mean-element mapping"),
    (np.pi-np.arccos(1/np.sqrt(5)), "invalid mean-element mapping"),
    (np.pi, "retrograde equator"),
])  # [rad]
def test_equinoctial_rejects_singular_domain(inclination, message):
    """An unsupported conversion must stop the controller instead of producing a NaN force."""
    with pytest.raises(bskLogging.BasiliskError, match=message):
        _equinoctial_force(0.1, inclination, orbitalMotion.J2_EARTH, 0.3, 0.3)
