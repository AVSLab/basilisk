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
import os, inspect
import numpy
import math
import pytest
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import oe_state_ephem
from Basilisk.simulation import sim_model
from Basilisk import pyswice
import matplotlib.pyplot as plt
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('fswAlgorithms')
from Basilisk import __path__
bskPath = __path__[0]

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

orbitPosAccuracy = 10000.0
orbitVelAccuracy = 1.0
unitTestSupport.writeTeXSnippet("tolerancePosValue", str(orbitPosAccuracy), path)
unitTestSupport.writeTeXSnippet("toleranceVelValue", str(orbitVelAccuracy), path)


@pytest.mark.parametrize('validChebyCurveTime', [True, False])
def test_chebyPosFitAllTest(show_plots, validChebyCurveTime):
    [testResults, testMessage] = chebyPosFitAllTest(show_plots, validChebyCurveTime)
    assert testResults < 1, testMessage


def chebyPosFitAllTest(show_plots, validChebyCurveTime):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    #__tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    numCurvePoints = 4*8640+1
    curveDurationSeconds = 4*86400
    logPeriod = curveDurationSeconds / (numCurvePoints - 1)
    degChebCoeff = 14
    integFrame = "j2000"
    zeroBase = "Earth"
    centralBodyMu = 3.98574405096E14

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
    rpArray = []
    eccArray = []
    incArray = []
    OmegaArray = []
    omegaArray = []
    fArray = []
    fPrev = 0.0
    fCount = 0

    for timeVal in timeHistory:
        stringCurrent = pyswice.et2utc_c(timeVal, 'C', 4, 1024, "Yo")
        stateOut = pyswice.spkRead('-221', stringCurrent, integFrame, zeroBase)
        for i in range(3):
            pyswice.doubleArray_setitem(posCArray, i, stateOut[i]*1000.0)
            pyswice.doubleArray_setitem(velCArray, i, stateOut[i+3]*1000.0)
        sim_model.rv2elem(centralBodyMu, posCArray, velCArray, orbEl)
        tdrssPosList.append([stateOut[0]*1000.0, stateOut[1]*1000.0, stateOut[2]*1000.0] )
        tdrssVelList.append([stateOut[3]*1000.0, stateOut[4]*1000.0, stateOut[5]*1000.0] )
        rpArray.append(orbEl.rPeriap)
        eccArray.append(orbEl.e)
        incArray.append(orbEl.i)
        OmegaArray.append(orbEl.Omega)
        omegaArray.append(orbEl.omega)
        currentf = orbEl.f
        if currentf < fPrev:
            fCount += 1
        fArray.append(2*math.pi*fCount + currentf)
        fPrev = currentf


    tdrssPosList = numpy.array(tdrssPosList)
    tdrssVelList = numpy.array(tdrssVelList)

    fitTimes = numpy.linspace(-1, 1, numCurvePoints)
    chebRpCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, rpArray, degChebCoeff)
    chebEccCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, eccArray, degChebCoeff)
    chebIncCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, incArray, degChebCoeff)
    chebOmegaCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, OmegaArray, degChebCoeff)
    chebomegaCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, omegaArray, degChebCoeff)
    chebfCoeff = numpy.polynomial.chebyshev.chebfit(fitTimes, fArray, degChebCoeff)

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()
    sim.TotalSim.terminateSimulation()

    FSWUnitTestProc = sim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    FSWUnitTestProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(logPeriod)))

    oeStateModel = oe_state_ephem.OEStateEphemData()
    oeStateModelWrap = sim.setModelDataWrap(oeStateModel)
    oeStateModelWrap.ModelTag = "oeStateModel"
    sim.AddModelToTask(unitTaskName, oeStateModelWrap, oeStateModel)

    oeStateModel.stateFitOutMsgName = "veh_state_est"
    oeStateModel.clockCorrInMsgName = "vehicle_clock_ephem_corr"
    oeStateModel.muCentral = centralBodyMu

    oeStateModel.ephArray[0].rPeriapCoeff = chebRpCoeff.tolist()
    oeStateModel.ephArray[0].eccCoeff = chebEccCoeff.tolist()
    oeStateModel.ephArray[0].incCoeff = chebIncCoeff.tolist()
    oeStateModel.ephArray[0].argPerCoeff = chebomegaCoeff.tolist()
    oeStateModel.ephArray[0].anomCoeff = chebfCoeff.tolist()
    oeStateModel.ephArray[0].RAANCoeff = chebOmegaCoeff.tolist()
    oeStateModel.ephArray[0].nChebCoeff = degChebCoeff + 1
    oeStateModel.ephArray[0].ephemTimeMid = etStart + curveDurationSeconds/2.0
    oeStateModel.ephArray[0].ephemTimeRad = curveDurationSeconds/2.0

    clockCorrData = oe_state_ephem.TDBVehicleClockCorrelationFswMsg()
    clockCorrData.vehicleClockTime = 0.0
    clockCorrData.ephemerisTime = oeStateModel.ephArray[0].ephemTimeMid - \
        oeStateModel.ephArray[0].ephemTimeRad

    sim.TotalSim.CreateNewMessage(unitProcessName, oeStateModel.clockCorrInMsgName,
                                  clockCorrData.getStructSize(), 2, "TDBVehicleClockCorrelationMessage")
    sim.TotalSim.WriteMessageData(oeStateModel.clockCorrInMsgName,
                                  clockCorrData.getStructSize(), 0, clockCorrData)

    sim.TotalSim.logThisMessage(oeStateModel.stateFitOutMsgName)

    if not validChebyCurveTime :
        sim.InitializeSimulation()
        # increase the run time by one logging period so that the sim time is outside the
        # valid chebychev curve duration
        sim.ConfigureStopTime(int((curveDurationSeconds + logPeriod) * 1.0E9))
        sim.ExecuteSimulation()
    else:
        sim.InitializeSimulation()
        sim.ConfigureStopTime(int(curveDurationSeconds*1.0E9))
        sim.ExecuteSimulation()

    posChebData = sim.pullMessageLogData(oeStateModel.stateFitOutMsgName + ".r_BdyZero_N",
                                         range(3))
    velChebData = sim.pullMessageLogData(oeStateModel.stateFitOutMsgName + ".v_BdyZero_N",
                                         range(3))

    if not validChebyCurveTime:
        lastLogidx = (curveDurationSeconds + logPeriod) / logPeriod - 1
        secondLastPos = posChebData[lastLogidx + 1, 1:] - tdrssPosList[lastLogidx, :]
        lastPos = posChebData[lastLogidx, 1:] - tdrssPosList[lastLogidx, :]
        if not numpy.array_equal(secondLastPos, lastPos):
            testFailCount += 1
            testMessages.append("FAILED: Expected Chebychev position to rail high or low " + str(secondLastPos) + " != " + str(lastPos) )

        secondLastVel = velChebData[lastLogidx + 1, 1:] - tdrssVelList[lastLogidx, :]
        lastVel = velChebData[lastLogidx, 1:] - tdrssVelList[lastLogidx, :]
        if not numpy.array_equal(secondLastVel, lastVel):
            testFailCount += 1
            testMessages.append("FAILED: Expected Chebychev velocity to rail high or low " + str(secondLastVel) + " != " + str(lastVel) )

    else:
        maxErrVec = [abs(max(posChebData[:, 1] - tdrssPosList[:, 0])),
            abs(max(posChebData[:, 2] - tdrssPosList[:, 1])),
            abs(max(posChebData[:,3] - tdrssPosList[:, 2]))]
        maxVelErrVec = [abs(max(velChebData[:, 1] - tdrssVelList[:, 0])),
                 abs(max(velChebData[:, 2] - tdrssVelList[:, 1])),
                 abs(max(velChebData[:, 3] - tdrssVelList[:, 2]))]

        if max(maxErrVec) >= orbitPosAccuracy:
            testFailCount += 1
            testMessages.append("FAILED: maxErrVec >= orbitPosAccuracy, TDRSS Orbit Accuracy: " + str(max(maxErrVec)))
        if max(maxVelErrVec) >= orbitVelAccuracy:
            testFailCount += 1
            testMessages.append("FAILED: maxVelErrVec >= orbitVelAccuracy, TDRSS Velocity Accuracy: " + str(max(maxVelErrVec)))

        plt.close("all")
        # plot the fitted and actual position coordinates
        plt.figure(1)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        for idx in range(1, 4):
            plt.plot(posChebData[:, 0]*macros.NANO2HOUR, posChebData[:, idx]/1000,
                     color=unitTestSupport.getLineColor(idx, 3),
                     linewidth=0.5,
                     label='$r_{fit,' + str(idx) + '}$')
            plt.plot(posChebData[:, 0]*macros.NANO2HOUR, tdrssPosList[:, idx-1]/1000,
                     color=unitTestSupport.getLineColor(idx, 3),
                     linestyle='dashed', linewidth=2,
                     label='$r_{true,' + str(idx) + '}$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Inertial Position [km]')

        # plot the fitted and actual velocity coordinates
        plt.figure(2)
        for idx in range(1, 4):
            plt.plot(velChebData[:, 0]*macros.NANO2HOUR, velChebData[:, idx]/1000,
                     color=unitTestSupport.getLineColor(idx, 3),
                     linewidth=0.5,
                     label='$v_{fit,' + str(idx) + '}$')
            plt.plot(velChebData[:, 0]*macros.NANO2HOUR, tdrssVelList[:, idx-1]/1000,
                     color=unitTestSupport.getLineColor(idx, 3),
                     linestyle='dashed', linewidth=2,
                     label='$v_{true,' + str(idx) + '}$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Velocity [km/s]')

        # plot the difference in position coordinates
        plt.figure(3)
        arrayLength = posChebData[:, 0].size
        for idx in range(1,4):
            plt.plot(posChebData[:, 0] * macros.NANO2HOUR, posChebData[:, idx]  - tdrssPosList[:, idx-1],
                     color=unitTestSupport.getLineColor(idx, 3),
                     linewidth=0.5,
                     label='$\Delta r_{' + str(idx) + '}$')
        plt.plot(velChebData[:, 0] * macros.NANO2HOUR, orbitPosAccuracy*numpy.ones(arrayLength),
                 color='r', linewidth=1)
        plt.plot(velChebData[:, 0] * macros.NANO2HOUR, -orbitPosAccuracy * numpy.ones(arrayLength),
                 color='r', linewidth=1)
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Position Difference [m]')

        # plot the difference in velocity coordinates
        plt.figure(4)
        arrayLength = velChebData[:, 0].size
        for idx in range(1,4):
            plt.plot(velChebData[:, 0] * macros.NANO2HOUR, velChebData[:, idx]  - tdrssVelList[:, idx-1],
                     color=unitTestSupport.getLineColor(idx, 3),
                     linewidth=0.5,
                     label='$\Delta v_{' + str(idx) + '}$')
        plt.plot(velChebData[:, 0] * macros.NANO2HOUR, orbitVelAccuracy*numpy.ones(arrayLength),
                 color='r', linewidth=1)
        plt.plot(velChebData[:, 0] * macros.NANO2HOUR, -orbitVelAccuracy * numpy.ones(arrayLength),
                 color='r', linewidth=1)
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Velocity Difference [m/s]')

    if show_plots:
        plt.show()
        plt.close('all')

    snippentName = "passFail" + str(validChebyCurveTime)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print "PASSED: " + oeStateModelWrap.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print "Failed: " + oeStateModelWrap.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    chebyPosFitAllTest(True, True)
