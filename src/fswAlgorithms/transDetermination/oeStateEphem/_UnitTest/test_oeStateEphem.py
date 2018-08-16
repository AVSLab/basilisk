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
import sys, os, inspect
import numpy
import pytest
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('fswAlgorithms')
from Basilisk import __path__
bskPath = __path__[0]


from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import oe_state_ephem
from Basilisk.simulation import sim_model
import ctypes
from Basilisk import pyswice
import matplotlib.pyplot as plt

orbitPosAccuracy = 10000.0
orbitVelAccuracy = 1.0

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def chebyPosFitAllTest(show_plots):
    [testResults, testMessage] = test_earthOrbitFit(show_plots)
    assert testResults < 1, testMessage

def test_earthOrbitFit(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    #__tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    numCurvePoints = 4*8640+1
    curveDurationSeconds = 4*86400
    degChebCoeff =6
    integFrame = "j2000"
    zeroBase = "Earth"
    centralBodyMu = 3.98574405096E14;

    dateSpice = "2015 April 10, 00:00:00.0 TDB"
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    et = pyswice.new_doubleArray(1)
    pyswice.str2et_c(dateSpice, et)
    etStart = pyswice.doubleArray_getitem(et, 0)
    etEnd = etStart + curveDurationSeconds

    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.furnsh_c(path + '/TDRSS.bsp')

    tdrssPosList = []
    tdrssVelList = []
    timeHistory = numpy.linspace(etStart, etEnd, numCurvePoints)
    posCArray = pyswice.new_doubleArray(3)
    velCArray = pyswice.new_doubleArray(3)
    orbEl = sim_model.classicElements()
    semiMajorArray = []
    eccArray = []
    incArray = []
    OmegaArray = []
    omegaArray = []
    timeArray = []
    mArray = []
    mPrev = 0.0
    mCount = 0

    for timeVal in timeHistory:
        stringCurrent = pyswice.et2utc_c(timeVal, 'C', 4, 1024, "Yo")
        stateOut = pyswice.spkRead('-221', stringCurrent, integFrame, zeroBase)
        for i in range(3):
            pyswice.doubleArray_setitem(posCArray, i, stateOut[i]*1000.0)
            pyswice.doubleArray_setitem(velCArray, i, stateOut[i+3]*1000.0)
        sim_model.rv2elem(centralBodyMu, posCArray, velCArray, orbEl)
        tdrssPosList.append([stateOut[0]*1000.0, stateOut[1]*1000.0, stateOut[2]*1000.0] )
        tdrssVelList.append([stateOut[3]*1000.0, stateOut[4]*1000.0, stateOut[5]*1000.0] )
        semiMajorArray.append(orbEl.a)
        eccArray.append(orbEl.e)
        incArray.append(orbEl.i)
        OmegaArray.append(orbEl.Omega)
        omegaArray.append(orbEl.omega)
        currentM = sim_model.E2M(sim_model.f2E(orbEl.f, orbEl.e), orbEl.e)
        if(currentM < mPrev):
            mCount += 1
        mArray.append(2*math.pi*mCount + currentM)
        mPrev = currentM

    tdrssPosList = numpy.array(tdrssPosList)
    tdrssVelList = numpy.array(tdrssVelList)

    fitTimes = numpy.linspace(-1, 1, numCurvePoints)
    chebSemCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, semiMajorArray, degChebCoeff)
    chebEccCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, eccArray, degChebCoeff)
    chebIncCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, incArray, degChebCoeff)
    chebOmegaCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, OmegaArray, degChebCoeff)
    chebomegaCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, omegaArray, degChebCoeff)
    chebMCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, mArray, degChebCoeff)
    ephemTimeMid = (etStart + etEnd)/2.0
    ephemTimeRad = (etEnd-etStart)/2.0

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    TotalSim = SimulationBaseClass.SimBaseClass()
    TotalSim.TotalSim.terminateSimulation()

    FSWUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    FSWUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, macros.sec2nano(curveDurationSeconds/(numCurvePoints-1))))

    oeStateModel = oe_state_ephem.OEStateEphemData()
    oeStateModelWrap = TotalSim.setModelDataWrap(oeStateModel)
    oeStateModelWrap.ModelTag = "oeStateModel"
    TotalSim.AddModelToTask(unitTaskName, oeStateModelWrap, oeStateModel)

    oeStateModel.stateFitOutMsgName = "veh_state_est"
    oeStateModel.clockCorrInMsgName = "vehicle_clock_ephem_corr"
    oeStateModel.muCentral = centralBodyMu

    oeStateModel.ephArray[0].semiMajorCoeff = chebSemCoeff.tolist()
    oeStateModel.ephArray[0].eccCoeff = chebEccCoeff.tolist()
    oeStateModel.ephArray[0].incCoeff = chebIncCoeff.tolist()
    oeStateModel.ephArray[0].argPerCoeff = chebomegaCoeff.tolist()
    oeStateModel.ephArray[0].meanAnomCoeff = chebMCoeff.tolist()
    oeStateModel.ephArray[0].RAANCoeff = chebOmegaCoeff.tolist()
    oeStateModel.ephArray[0].nChebCoeff = degChebCoeff+1
    oeStateModel.ephArray[0].ephemTimeMid = etStart + curveDurationSeconds/2.0
    oeStateModel.ephArray[0].ephemTimeRad = curveDurationSeconds/2.0

    clockCorrData = oe_state_ephem.TDBVehicleClockCorrelationFswMsg()
    clockCorrData.vehicleClockTime = 0.0
    clockCorrData.ephemerisTime = oeStateModel.ephArray[0].ephemTimeMid  - \
        oeStateModel.ephArray[0].ephemTimeRad

    TotalSim.TotalSim.CreateNewMessage(unitProcessName, oeStateModel.clockCorrInMsgName,
                                       clockCorrData.getStructSize(), 2, "TDBVehicleClockCorrelationMessage")
    TotalSim.TotalSim.WriteMessageData(oeStateModel.clockCorrInMsgName,
                                   clockCorrData.getStructSize(), 0, clockCorrData)

    TotalSim.TotalSim.logThisMessage(oeStateModel.stateFitOutMsgName)

    TotalSim.InitializeSimulation()
    TotalSim.ConfigureStopTime(int(curveDurationSeconds*1.0E9))
    TotalSim.ExecuteSimulation()

    posChebData = TotalSim.pullMessageLogData(oeStateModel.stateFitOutMsgName + ".r_BdyZero_N",
                                              range(3))
    velChebData = TotalSim.pullMessageLogData(oeStateModel.stateFitOutMsgName + ".v_BdyZero_N",
                                                  range(3))

    maxErrVec = [abs(max(posChebData[:,1] - tdrssPosList[:,0])),
        abs(max(posChebData[:,2] - tdrssPosList[:,1])),
        abs(max(posChebData[:,3] - tdrssPosList[:,2]))]
    maxVelErrVec = [abs(max(velChebData[:,1] - tdrssVelList[:,0])),
             abs(max(velChebData[:,2] - tdrssVelList[:,1])),
             abs(max(velChebData[:,3] - tdrssVelList[:,2]))]
    print "TDRSS Orbit Accuracy: " + str(max(maxErrVec))
    print "TDRSS Velocity Accuracy: " + str(max(maxVelErrVec))
    assert (max(maxErrVec)) < orbitPosAccuracy
    assert (max(maxVelErrVec)) < orbitVelAccuracy
    plt.figure()
    plt.plot(velChebData[:,0]*1.0E-9, velChebData[:,1], velChebData[:,0]*1.0E-9, tdrssVelList[:,0])

    if(show_plots):
        plt.show()
        plt.close('all')

    if testFailCount == 0:
        print "PASSED: " + " Orbit curve fit"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    chebyPosFitAllTest(False)
