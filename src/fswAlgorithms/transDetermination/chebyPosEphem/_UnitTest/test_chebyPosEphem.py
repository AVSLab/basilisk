# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
import math
import os

import numpy
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
from Basilisk import __path__

bskPath = __path__[0]

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import chebyPosEphem
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities.pyswice_spk_utilities import spkRead
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

import matplotlib.pyplot as plt
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging

orbitPosAccuracy = 1.0
orbitVelAccuracy = 0.01


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
@pytest.mark.parametrize("function", ["sineCosine", "earthOrbitFit"])
def test_chebyPosFitAllTest(show_plots, function):
    """Module Unit Test"""
    testFunction = globals().get(function)

    if testFunction is None:
        raise ValueError(f"Function '{function}' not found in global scope")

    [testResults, testMessage] = testFunction(show_plots)
    assert testResults < 1, testMessage


def sineCosine(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    orbitRadius = 70000.0
    numCurvePoints = 365 * 3 + 1
    curveDurationDays = 365.0 * 3
    degChebCoeff = 21

    angleSpace = numpy.linspace(-3 * math.pi, 3 * math.pi, numCurvePoints)

    cosineValues = numpy.cos(angleSpace) * orbitRadius
    sineValues = numpy.sin(angleSpace) * orbitRadius
    oopValues = numpy.sin(angleSpace) + orbitRadius

    naif0012_path = get_path(DataFile.EphemerisData.naif0012)
    pyswice.furnsh_c(str(naif0012_path))
    et = pyswice.new_doubleArray(1)

    timeStringMid = "2019 APR 1 12:12:12.0 (UTC)"
    pyswice.str2et_c(timeStringMid, et)

    fitTimes = numpy.linspace(-1, 1, numCurvePoints)

    chebCosCoeff = numpy.polynomial.chebyshev.chebfit(
        fitTimes, cosineValues, degChebCoeff
    )
    chebSinCoeff = numpy.polynomial.chebyshev.chebfit(
        fitTimes, sineValues, degChebCoeff
    )
    cheboopCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, oopValues, degChebCoeff)

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    TotalSim = SimulationBaseClass.SimBaseClass()

    FSWUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    FSWUnitTestProc.addTask(
        TotalSim.CreateNewTask(unitTaskName, macros.sec2nano(8640.0))
    )

    chebyFitModel = chebyPosEphem.chebyPosEphem()
    chebyFitModel.ModelTag = "chebyFitModel"
    TotalSim.AddModelToTask(unitTaskName, chebyFitModel)

    totalList = numpy.array(chebCosCoeff).tolist()
    totalList.extend(numpy.array(chebSinCoeff).tolist())
    totalList.extend(numpy.array(cheboopCoeff).tolist())

    chebyFitModel.ephArray[0].posChebyCoeff = totalList
    chebyFitModel.ephArray[0].nChebCoeff = degChebCoeff + 1
    chebyFitModel.ephArray[0].ephemTimeMid = pyswice.doubleArray_getitem(et, 0)
    chebyFitModel.ephArray[0].ephemTimeRad = curveDurationDays / 2.0 * 86400.0

    clockCorrData = messaging.TDBVehicleClockCorrelationMsgPayload()
    clockCorrData.vehicleClockTime = 0.0
    clockCorrData.ephemerisTime = (
        chebyFitModel.ephArray[0].ephemTimeMid - chebyFitModel.ephArray[0].ephemTimeRad
    )
    clockInMsg = messaging.TDBVehicleClockCorrelationMsg().write(clockCorrData)
    chebyFitModel.clockCorrInMsg.subscribeTo(clockInMsg)

    xFitData = numpy.polynomial.chebyshev.chebval(fitTimes, chebCosCoeff)

    dataLog = chebyFitModel.posFitOutMsg.recorder()
    TotalSim.AddModelToTask(unitTaskName, dataLog)

    TotalSim.InitializeSimulation()
    TotalSim.ConfigureStopTime(int(curveDurationDays * 86400.0 * 1.0e9))
    TotalSim.ExecuteSimulation()

    posChebData = dataLog.r_BdyZero_N

    angleSpaceFine = numpy.linspace(-3 * math.pi, 3 * math.pi, numCurvePoints * 10 - 9)

    cosineValuesFine = numpy.cos(angleSpaceFine) * orbitRadius
    sineValuesFine = numpy.sin(angleSpaceFine) * orbitRadius
    oopValuesFine = numpy.sin(angleSpaceFine) + orbitRadius

    maxErrVec = [
        max(abs(posChebData[:, 0] - cosineValuesFine)),
        max(abs(posChebData[:, 1] - sineValuesFine)),
        max(abs(posChebData[:, 2] - oopValuesFine)),
    ]

    print("Sine Wave error: " + str(max(maxErrVec)))
    assert max(maxErrVec) < orbitPosAccuracy

    if testFailCount == 0:
        print("PASSED: " + " Sine and Cosine curve fit")
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, "".join(testMessages)]


def earthOrbitFit(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    # __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    numCurvePoints = 365 * 3 + 1
    curveDurationSeconds = 3 * 5950.0
    degChebCoeff = 23
    integFrame = "j2000"
    zeroBase = "Earth"

    dateSpice = "2015 February 10, 00:00:00.0 TDB"
    naif0012_path = get_path(DataFile.EphemerisData.naif0012)
    pyswice.furnsh_c(str(naif0012_path))
    et = pyswice.new_doubleArray(1)
    pyswice.str2et_c(dateSpice, et)
    etStart = pyswice.doubleArray_getitem(et, 0)
    etEnd = etStart + curveDurationSeconds

    de430_path = get_path(DataFile.EphemerisData.de430)
    naif0012_path = get_path(DataFile.EphemerisData.naif0012)
    de403masses_path = get_path(DataFile.EphemerisData.de_403_masses)
    pck00010_path = get_path(DataFile.EphemerisData.pck00010)
    hst_edited_path = get_path(DataFile.EphemerisData.hst_edited)
    pyswice.furnsh_c(str(de430_path))
    pyswice.furnsh_c(str(naif0012_path))
    pyswice.furnsh_c(str(de403masses_path))
    pyswice.furnsh_c(str(pck00010_path))
    pyswice.furnsh_c(str(hst_edited_path))

    hubblePosList = []
    hubbleVelList = []
    timeHistory = numpy.linspace(etStart, etEnd, numCurvePoints)

    for timeVal in timeHistory:
        stringCurrent = pyswice.et2utc_c(timeVal, "C", 4, 1024, "Yo")
        stateOut = spkRead(
            "HUBBLE SPACE TELESCOPE", stringCurrent, integFrame, zeroBase
        )
        hubblePosList.append(stateOut[0:3].tolist())
        hubbleVelList.append(stateOut[3:6].tolist())

    hubblePosList = numpy.array(hubblePosList)
    hubbleVelList = numpy.array(hubbleVelList)

    fitTimes = numpy.linspace(-1, 1, numCurvePoints)
    chebCoeff = numpy.polynomial.chebyshev.chebfit(
        fitTimes, hubblePosList, degChebCoeff
    )

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    TotalSim = SimulationBaseClass.SimBaseClass()

    FSWUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    FSWUnitTestProc.addTask(
        TotalSim.CreateNewTask(
            unitTaskName, macros.sec2nano(curveDurationSeconds / (numCurvePoints - 1))
        )
    )

    chebyFitModel = chebyPosEphem.chebyPosEphem()
    chebyFitModel.ModelTag = "chebyFitModel"
    TotalSim.AddModelToTask(unitTaskName, chebyFitModel)

    totalList = chebCoeff[:, 0].tolist()
    totalList.extend(chebCoeff[:, 1].tolist())
    totalList.extend(chebCoeff[:, 2].tolist())

    chebyFitModel.ephArray[0].posChebyCoeff = totalList
    chebyFitModel.ephArray[0].nChebCoeff = degChebCoeff + 1
    chebyFitModel.ephArray[0].ephemTimeMid = etStart + curveDurationSeconds / 2.0
    chebyFitModel.ephArray[0].ephemTimeRad = curveDurationSeconds / 2.0

    clockCorrData = messaging.TDBVehicleClockCorrelationMsgPayload()
    clockCorrData.vehicleClockTime = 0.0
    clockCorrData.ephemerisTime = (
        chebyFitModel.ephArray[0].ephemTimeMid - chebyFitModel.ephArray[0].ephemTimeRad
    )
    clockInMsg = messaging.TDBVehicleClockCorrelationMsg().write(clockCorrData)
    chebyFitModel.clockCorrInMsg.subscribeTo(clockInMsg)

    dataLog = chebyFitModel.posFitOutMsg.recorder()
    TotalSim.AddModelToTask(unitTaskName, dataLog)

    TotalSim.InitializeSimulation()
    TotalSim.ConfigureStopTime(int(curveDurationSeconds * 1.0e9))
    TotalSim.ExecuteSimulation()

    posChebData = dataLog.r_BdyZero_N
    velChebData = dataLog.v_BdyZero_N

    maxErrVec = [
        abs(max(posChebData[:, 0] - hubblePosList[:, 0])),
        abs(max(posChebData[:, 1] - hubblePosList[:, 1])),
        abs(max(posChebData[:, 2] - hubblePosList[:, 2])),
    ]
    maxVelErrVec = [
        abs(max(velChebData[:, 0] - hubbleVelList[:, 0])),
        abs(max(velChebData[:, 1] - hubbleVelList[:, 1])),
        abs(max(velChebData[:, 2] - hubbleVelList[:, 2])),
    ]
    print("Hubble Orbit Accuracy: " + str(max(maxErrVec)))
    print("Hubble Velocity Accuracy: " + str(max(maxVelErrVec)))
    assert (max(maxErrVec)) < orbitPosAccuracy
    assert (max(maxVelErrVec)) < orbitVelAccuracy
    plt.figure()
    plt.plot(
        dataLog.times() * 1.0e-9,
        velChebData[:, 0],
        dataLog.times() * 1.0e-9,
        hubbleVelList[:, 0],
    )

    if show_plots:
        plt.show()
        plt.close("all")

    if testFailCount == 0:
        print("PASSED: " + " Orbit curve fit")
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, "".join(testMessages)]


@pytest.mark.parametrize("reset_method", ["direct", "simulation"])
@pytest.mark.parametrize("old_count,new_count", [(4, 3), (10, 6), (6, 10), (6, 6), (6, 2), (6, 1)])
def test_chebyPosEphem_reset_rebuilds_velocity(reset_method, old_count, new_count):
    """Rebuild derivatives after changing coefficient counts, values, and time radii.

    Exercise every record and all three axes through both reset entry points.
    Compare position and velocity with NumPy's independent Chebyshev evaluator
    and derivative at multiple times, including a second reset without changes.
    """
    simulation = SimulationBaseClass.SimBaseClass()
    task_period = 1.0  # [s]
    simulation.CreateNewProcess("process").addTask(
        simulation.CreateNewTask("task", macros.sec2nano(task_period))
    )
    model = chebyPosEphem.chebyPosEphem()
    simulation.AddModelToTask("task", model)
    clock_data = messaging.TDBVehicleClockCorrelationMsgPayload()
    clock_message = messaging.TDBVehicleClockCorrelationMsg().write(clock_data)
    model.clockCorrInMsg.subscribeTo(clock_message)

    old_radius = 1000.0  # [s]
    new_radius = 500.0  # [s]
    record_spacing = 10000.0  # [s]
    coefficient_scale = 0.25  # [m]
    for record_index in range(chebyPosEphem.MAX_CHEB_RECORDS):
        record = model.ephArray[record_index]
        record.nChebCoeff = old_count
        record.ephemTimeMid = (record_index + 1) * record_spacing
        record.ephemTimeRad = old_radius
        coefficients = coefficient_scale * numpy.arange(1, 3 * old_count + 1)
        record.posChebyCoeff = coefficients.tolist()
    simulation.InitializeSimulation()

    coefficients = -coefficient_scale * numpy.arange(1, 3 * new_count + 1).reshape(3, new_count)
    for record_index in range(chebyPosEphem.MAX_CHEB_RECORDS):
        record = model.ephArray[record_index]
        record.nChebCoeff = new_count
        record.ephemTimeRad = new_radius
        record.posChebyCoeff = coefficients.ravel().tolist()

    for _ in range(2):
        if reset_method == "direct":
            model.Reset(0)
        else:
            simulation.InitializeSimulation()
        for record_index in range(chebyPosEphem.MAX_CHEB_RECORDS):
            record = model.ephArray[record_index]
            for scaled_time in [-0.75, 0.0, 0.6]:
                clock_data.ephemerisTime = record.ephemTimeMid + scaled_time * new_radius
                clock_message.write(clock_data)
                model.UpdateState(0)
                output = model.posFitOutMsg.read()
                position = [numpy.polynomial.chebyshev.chebval(scaled_time, axis) for axis in coefficients]
                velocity = [
                    numpy.polynomial.chebyshev.chebval(
                        scaled_time, numpy.polynomial.chebyshev.chebder(axis)
                    ) / new_radius
                    for axis in coefficients
                ]
                position_tolerance = 1.0e-13  # [m]
                velocity_tolerance = 1.0e-15  # [m/s]
                numpy.testing.assert_allclose(output.r_BdyZero_N, position, rtol=1.0e-13, atol=position_tolerance)
                numpy.testing.assert_allclose(output.v_BdyZero_N, velocity, rtol=1.0e-13, atol=velocity_tolerance)


@pytest.mark.parametrize("record_slots", [(0, 1), (1, 0), (1, 3)])
@pytest.mark.parametrize(
    "intervals,epoch,expected_record,expected_time",
    [
        # Interval endpoints and epochs are [s]; expected_time is normalized [-].
        pytest.param([(0.0, 1000.0), (1000.0, 2000.0)], -100.0, 0, -1.0, id="before-first"),
        pytest.param([(0.0, 1000.0), (1000.0, 2000.0)], 0.0, 0, -1.0, id="first-start"),
        pytest.param([(0.0, 1000.0), (1000.0, 2000.0)], 500.0, 0, 0.0, id="first-interior"),
        pytest.param([(0.0, 1000.0), (1000.0, 2000.0)], 1500.0, 1, 0.0, id="last-interior"),
        pytest.param([(0.0, 1000.0), (1000.0, 2000.0)], 2000.0, 1, 1.0, id="last-end"),
        pytest.param([(0.0, 1000.0), (1000.0, 2000.0)], 2500.0, 1, 1.0, id="after-last"),
        pytest.param([(0.0, 2000.0), (4000.0, 4500.0)], 2900.0, 0, 1.0, id="gap-near-left-end"),
        pytest.param([(0.0, 2000.0), (4000.0, 4500.0)], 3100.0, 1, -1.0, id="gap-near-right-end"),
        pytest.param([(0.0, 2000.0), (4000.0, 4500.0)], 3000.0, None, None, id="gap-tie"),
        pytest.param([(0.0, 1000.0), (1000.0, 2000.0)], 1000.0, None, None, id="shared-endpoint"),
        pytest.param([(0.0, 1000.0), (500.0, 1500.0)], 750.0, None, None, id="overlap"),
    ],
)
def test_chebyPosEphem_record_selection(record_slots, intervals, epoch, expected_record, expected_time):
    """Evaluate the containing record or nearest endpoint, regardless of storage order.

    Unequal interval widths distinguish endpoint distance from midpoint distance.
    Ties and overlapping coverage retain the first record in array order.
    Check both position and velocity against independently evaluated polynomials.
    """
    simulation = SimulationBaseClass.SimBaseClass()
    task_period = 1.0  # [s]
    simulation.CreateNewProcess("process").addTask(
        simulation.CreateNewTask("task", macros.sec2nano(task_period))
    )
    model = chebyPosEphem.chebyPosEphem()
    simulation.AddModelToTask("task", model)
    coefficients = numpy.array([
        [[1.0e6, 2.0e5, 3.0e4], [-2.0e6, 4.0e5, -5.0e4], [3.0e6, -6.0e5, 7.0e4]],
        [[5.0e6, -3.0e5, 8.0e4], [6.0e6, 7.0e5, -9.0e4], [-7.0e6, 8.0e5, 1.0e5]],
    ])  # [m]
    for record_index, slot in enumerate(record_slots):
        start, end = intervals[record_index]
        record = model.ephArray[slot]
        record.nChebCoeff = coefficients.shape[2]
        record.ephemTimeMid = (start + end) / 2.0
        record.ephemTimeRad = (end - start) / 2.0
        record.posChebyCoeff = coefficients[record_index].ravel().tolist()

    clock_data = messaging.TDBVehicleClockCorrelationMsgPayload()
    clock_data.vehicleClockTime = 10.0  # [s]
    clock_data.ephemerisTime = epoch + clock_data.vehicleClockTime
    clock_message = messaging.TDBVehicleClockCorrelationMsg().write(clock_data)
    model.clockCorrInMsg.subscribeTo(clock_message)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(0)
    simulation.ExecuteSimulation()

    if expected_record is None:
        expected_record = record_slots.index(min(record_slots))
        start, end = intervals[expected_record]
        expected_time = numpy.clip((2.0 * epoch - start - end) / (end - start), -1.0, 1.0)
    start, end = intervals[expected_record]
    radius = (end - start) / 2.0
    position = [
        numpy.polynomial.chebyshev.chebval(expected_time, axis)
        for axis in coefficients[expected_record]
    ]
    velocity = [
        numpy.polynomial.chebyshev.chebval(expected_time, numpy.polynomial.chebyshev.chebder(axis)) / radius
        for axis in coefficients[expected_record]
    ]
    output = model.posFitOutMsg.read()
    position_tolerance = 1.0e-8  # [m]
    velocity_tolerance = 1.0e-10  # [m/s]
    assert model.coeffSelector == record_slots[expected_record]
    numpy.testing.assert_allclose(output.r_BdyZero_N, position, rtol=1.0e-14, atol=position_tolerance)
    numpy.testing.assert_allclose(output.v_BdyZero_N, velocity, rtol=1.0e-14, atol=velocity_tolerance)
    assert output.timeTag == 0.0


def test_chebyPosEphem_requires_configured_record():
    """Reject evaluation when every record slot is unused."""
    model = chebyPosEphem.chebyPosEphem()
    clock_message = messaging.TDBVehicleClockCorrelationMsg().write(
        messaging.TDBVehicleClockCorrelationMsgPayload()
    )
    model.clockCorrInMsg.subscribeTo(clock_message)
    model.SelfInit()
    model.Reset(0)
    with pytest.raises(bskLogging.BasiliskError, match="at least one configured ephemeris record"):
        model.UpdateState(0)


if __name__ == "__main__":
    test_chebyPosFitAllTest(True)
