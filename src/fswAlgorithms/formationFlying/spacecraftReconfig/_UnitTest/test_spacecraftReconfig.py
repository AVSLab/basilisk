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
#   Module Name:        spacecraftReconfig
#   Author:             Hirotaka Kondo
#   Creation Date:      March 27, 2020
#

import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import spacecraftReconfig  # import the module that is to be tested
# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import simHelpers
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import fswSetupThrusters
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("useRefAttitude", [True, False])
@pytest.mark.parametrize("accuracy", [1e-9])

def test_spacecraftReconfig(show_plots, useRefAttitude, accuracy):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = spacecraftReconfigTestFunction(show_plots, useRefAttitude, accuracy)
    assert testResults < 1, testMessage


def spacecraftReconfigTestFunction(show_plots, useRefAttitude, accuracy):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    # Create a sim spacecraftReconfig as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # process rate
    testProc = unitTestSim.CreateNewProcess(unitProcessName)  # create new process
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))  # create new task
    # Construct algorithm and associated C++ container
    module = spacecraftReconfig.spacecraftReconfig()
    module.ModelTag = "spacecraftReconfig"  # update python name of test spacecraftReconfig
    module.targetClassicOED = [0.0000, 0.0000, 0.0000, 0.0001, 0.0002, 0.0003]
    module.attControlTime = 400  # [s]
    module.mu = orbitalMotion.MU_EARTH * 1e9  # [m^3/s^2]
    # Add test spacecraftReconfig to runtime call list
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
    module.chiefTransInMsg.subscribeTo(chiefInMsg)
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
    module.deputyTransInMsg.subscribeTo(deputyInMsg)

    #
    # Deputy Vehicle Config Message
    #
    vehicleConfigInData = messaging.VehicleConfigMsgPayload()
    vehicleConfigInData.massSC = 500
    vehicleConfigMsg = messaging.VehicleConfigMsg().write(vehicleConfigInData)
    module.vehicleConfigInMsg.subscribeTo(vehicleConfigMsg)

    #
    # reference attitude message
    #
    if useRefAttitude:
        attRefInData = messaging.AttRefMsgPayload()
        attRefInData.sigma_RN = [1.0, 0.0, 0.0]
        attRefInData.omega_RN_N = [0.0, 0.0, 0.0]
        attRefInData.domega_RN_N = [0.0, 0.0, 0.0]
        attRefInMsg = messaging.AttRefMsg().write(attRefInData)
        module.attRefInMsg.subscribeTo(attRefInMsg)

    #
    # thruster configuration message
    #
    location = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    direction = [[1.0, 1.0, 1.0], [1.0, 1.0, 1.0]]  # get thrust in +z direction
    fswSetupThrusters.clearSetup()
    for i in range(len(location)):
        fswSetupThrusters.create(location[i], direction[i], 22.6)
    thrConfMsg = fswSetupThrusters.writeConfigMessage()
    module.thrustConfigInMsg.subscribeTo(thrConfMsg)

    # Setup logging on the test spacecraftReconfig output message so that we get all the writes to it
    dataLog = module.attRefOutMsg.recorder()
    moduleLog = module.logger("resetPeriod")
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    unitTestSim.AddModelToTask(unitTaskName, moduleLog)

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
    attOutput = dataLog.sigma_RN
    resetPeriod = simHelpers.addTimeColumn(moduleLog.times(), moduleLog.resetPeriod)
    # set the filtered output truth states
    if useRefAttitude:
        trueVector = [[1.0,0.0,0.0]]
    else:
        trueVector = [[0.38532697209248595,
                       -0.7016349090839732,
                       -0.4026194572440069]]
    trueResetPeriod = 28148.5466910579925752244889736
    # compare the spacecraftReconfig results to the truth values
    for i in range(0, len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(attOutput[i], trueVector[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed sigma_RN" + " unit test at t="
                                + str(attOutput[i, 0]*macros.NANO2SEC) + "sec\n")

    if (not unitTestSupport.isDoubleEqualRelative(resetPeriod[0,1], trueResetPeriod, accuracy)):
        testFailCount += 1
        testMessages.append("FAILED: " + module.ModelTag + " Module failed " + "resetPeriod")

    # The first update after reset starts the new plan from its current navigation sample.
    module.Reset(testProcessRate)
    module.UpdateState(2 * testProcessRate)
    assert module.tCurrent == 0.0  # [s]
    module.UpdateState(3 * testProcessRate)
    assert module.tCurrent == pytest.approx(testProcessRate * macros.NANO2SEC)  # [s]

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
        print("This test uses an accuracy value of " + str(accuracy))

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


@pytest.fixture
def burn_module():
    """Provide a module with live navigation, vehicle, and thruster inputs."""
    module = spacecraftReconfig.spacecraftReconfig()
    module.mu = orbitalMotion.MU_EARTH * 1e9  # [m^3/s^2]
    module.targetClassicOED = [0.0, 0.0001, 0.0002, -0.0001, -0.0002, -0.0003]  # [-, -, rad, rad, rad, rad]
    nav = messaging.NavTransMsgPayload()
    nav.r_BN_N = [7.0e6, 0.0, 0.0]  # [m]
    nav.v_BN_N = [0.0, 7500.0, 1000.0]  # [m/s]
    nav_msg = messaging.NavTransMsg().write(nav)
    module.chiefTransInMsg.subscribeTo(nav_msg)
    module.deputyTransInMsg.subscribeTo(nav_msg)
    thrusters = messaging.THRArrayConfigMsgPayload()
    thrusters.numThrusters = 2
    thruster = messaging.THRConfigMsgPayload()
    thruster.maxThrust = 22.6  # [N]
    thruster.tHatThrust_B = [1.0, 0.0, 0.0]  # [-]
    for index in range(thrusters.numThrusters):
        messaging.ThrustConfigArray_setitem(thrusters.thrusters, index, thruster)
    thruster_msg = messaging.THRArrayConfigMsg().write(thrusters)
    module.thrustConfigInMsg.subscribeTo(thruster_msg)
    vehicle = messaging.VehicleConfigMsgPayload()
    vehicle.massSC = 500.0  # [kg]
    vehicle_msg = messaging.VehicleConfigMsg().write(vehicle)
    module.vehicleConfigInMsg.subscribeTo(vehicle_msg)

    module.SelfInit()
    yield module


@pytest.mark.parametrize("burn_slot", [0, 1, 2])
@pytest.mark.parametrize("epoch", [0, macros.sec2nano(1.0)])  # [ns]
def test_fractional_burn_start(burn_module, burn_slot, epoch):
    """Each burn slot must command both thrusters on the first tick inside the burn window."""
    module = burn_module
    module.Reset(epoch)
    module.resetPeriod = 100.0  # [s]
    # Supply a known schedule so the ignition check is independent of orbit selection.
    schedule = module.burnArrayInfoOutMsgBuffer
    for index in range(3):
        burn = messaging.ReconfigBurnInfoMsgPayload()
        if index == burn_slot:
            burn.flag = 1  # pending
            burn.t = 2.4  # [s] since reset
            burn.thrustOnTime = 9.5  # [s] total across both thrusters
        else:
            burn.flag = 3  # skipped
        messaging.ReconfigBurnArray_setitem(schedule.burnArray, index, burn)

    command = messaging.THRArrayOnTimeCmdMsgReader()
    command.subscribeTo(module.onTimeOutMsg)
    module.UpdateState(epoch)
    assert not command.isWritten()  # 2.4 s remain, outside the 2.375 s half-burn
    next_tick = epoch + macros.sec2nano(0.1)  # [ns]
    module.UpdateState(next_tick)
    assert command.timeWritten() == next_tick  # 2.3 s remain, inside the half-burn
    assert list(command().OnTimeRequest[:2]) == [4.75, 4.75]  # [s]


@pytest.mark.parametrize("existing_plan", [False, True], ids=["delayed-first-update", "expired-plan"])
def test_burn_clock_after_long_gap(burn_module, existing_plan, capfd):
    """Plan from the first navigation sample, including after a long pause expires an old plan."""
    module = burn_module
    module.Reset(0)
    if existing_plan:
        module.UpdateState(0)
        plan = module.burnArrayInfoOutMsg.read()
        assert plan.burnArray.flag != 0

    update_time = macros.sec2nano(105 * 86400)  # [ns]
    step = macros.sec2nano(0.1)  # [ns]
    module.UpdateState(update_time)
    if existing_plan:
        # The expired plan is cleared on this update and rebuilt on the next one.
        plan = module.burnArrayInfoOutMsg.read()
        assert plan.burnArray.flag == 0
        update_time += step
        module.UpdateState(update_time)
    plan = module.burnArrayInfoOutMsg.read()
    assert plan.burnArray.flag != 0
    assert module.tCurrent == 0.0  # [s]
    module.UpdateState(update_time + step)
    assert module.tCurrent == pytest.approx(0.1)  # [s]
    assert "exceeds safe conversion limit" not in capfd.readouterr().err


@pytest.mark.parametrize("use_ref_attitude", [False, True])
def test_burn_clock_within_long_orbit(burn_module, use_ref_attitude):
    """Keep a future burn across a pause, then replan if its entire window was missed."""
    module = burn_module
    module.mu = orbitalMotion.MU_SUN * 1e9  # [m^3/s^2]
    nav = messaging.NavTransMsgPayload()
    nav.r_BN_N = [1.5e11, 0.0, 0.0]  # [m]
    nav.v_BN_N = [0.0, 28000.0, 4000.0]  # [m/s]
    nav_msg = messaging.NavTransMsg().write(nav)
    module.chiefTransInMsg.subscribeTo(nav_msg)
    module.deputyTransInMsg.subscribeTo(nav_msg)
    if use_ref_attitude:
        reference = messaging.AttRefMsg().write(messaging.AttRefMsgPayload())
        module.attRefInMsg.subscribeTo(reference)
    module.Reset(0)
    module.UpdateState(0)

    command = messaging.THRArrayOnTimeCmdMsgReader()
    command.subscribeTo(module.onTimeOutMsg)
    plan = module.burnArrayInfoOutMsg.read()
    first_burn = plan.burnArray
    elapsed = 105 * 86400 + 0.125  # [s]
    assert first_burn.flag == 1
    assert elapsed < first_burn.t
    module.UpdateState(macros.sec2nano(elapsed))
    assert module.tCurrent == elapsed
    assert not command.isWritten()

    expected_attitude = list(module.attRefOutMsg.read().sigma_RN)
    if use_ref_attitude:
        expected_attitude = [0.1, -0.2, 0.3]  # [-] nominal MRPs at reactivation
        reference.write(messaging.AttRefMsgPayload(sigma_RN=expected_attitude))
    resume = 200 * 86400  # [s]
    assert first_burn.t + first_burn.thrustOnTime < resume < module.resetPeriod
    module.UpdateState(macros.sec2nano(resume))
    assert not command.isWritten()
    assert list(module.attRefOutMsg.read().sigma_RN) == pytest.approx(expected_attitude)
    assert module.burnArrayInfoOutMsg.read().burnArray.flag == 0
    new_epoch = macros.sec2nano(resume + 1)  # [ns]
    module.UpdateState(new_epoch)
    new_plan = module.burnArrayInfoOutMsg.read()
    assert new_plan.burnArray.flag == 1
    assert module.tCurrent == 0.0  # [s]
    assert not command.isWritten()
    ignition = new_epoch + macros.sec2nano(new_plan.burnArray.t)  # [ns] inside the new window
    module.UpdateState(ignition)
    assert command.timeWritten() == ignition
    assert list(command().OnTimeRequest[:2]) == pytest.approx([new_plan.burnArray.thrustOnTime / 2] * 2)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_spacecraftReconfig(
        False,  # show_plots
        True,  # useRefAttitude
        1e-9    # accuracy
    )
