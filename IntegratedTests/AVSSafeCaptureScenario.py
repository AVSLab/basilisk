'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import AVSSim
import matplotlib.pyplot as plt
import ctypes
import math
import MessagingAccess
import sim_model
import numpy
import logging


def checkCSSEstAccuracy(DataCSSTruth, FSWsHat, CSSEstAccuracyThresh):
    i = 0
    j = 0
    accuracyFailCounter = 0
    while (i < FSWsHat.shape[0]):
        while (DataCSSTruth[j, 0] < FSWsHat[i, 0]):
            j += 1
        timePrevDelta = DataCSSTruth[j - 1, 0] - FSWsHat[i, 0]
        timeNowDelta = DataCSSTruth[j, 0] - FSWsHat[i, 0]
        indexUse = j - 1 if abs(timePrevDelta) < abs(timeNowDelta) else j
        dot_val = numpy.dot(DataCSSTruth[indexUse, 1:], FSWsHat[i, 1:])
        if (abs(dot_val) > 1.0):
            dot_val = 1.0
        angCheck = math.acos(dot_val)
        if (angCheck > CSSEstAccuracyThresh):
            errorString = "CSS accuracy failure for value of [deg]: "
            errorString += str(angCheck * 180.0 / math.pi)
            logging.error(errorString)
            accuracyFailCounter += 1
        i += 1
    return accuracyFailCounter


def checkSlewAccuracy(DataCSSTruth, FSWsHat, CSSEstAccuracyThresh,
                      slewFinishTime, desiredSunBdy):
    dtTruth = (DataCSSTruth[10, 0] - DataCSSTruth[9, 0]) * 1.0E-9
    counterStart = int(slewFinishTime / dtTruth)
    controlFailCounter = 0
    while (counterStart < DataCSSTruth.shape[0]):
        dot_val = numpy.dot(DataCSSTruth[counterStart, 1:], desiredSunBdy)
        if (dot_val > 1.0):
            dot_val = 1.0
        if (math.acos(dot_val) > CSSEstAccuracyThresh):
            controlFailCounter += 1
            errorString = "Safe mode control failure for value of [deg]: "
            errorString += str(math.acos(dot_val) * 180.0 / math.pi)
            logging.error(errorString)
        counterStart += 1
    return controlFailCounter


def executeAVSSafeCapture(TheAVSSim):
    TheAVSSim.TotalSim.logThisMessage("controlTorqueRaw", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("sun_safe_att_err", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("css_wls_est", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("solar_array_sun_bore", int(1E9))  # solar array boresight angles
    TheAVSSim.AddVectorForLogging('CSSConstelation.sensorList[0].sHatStr', 'double', 0, 2, int(1E9))

    TheAVSSim.InitializeSimulation()
    TheAVSSim.ConfigureStopTime(int(30 * 1E9))
    TheAVSSim.ExecuteSimulation()
    TheAVSSim.modeRequest = 'safeMode'
    print '\n Mode Request = ', TheAVSSim.modeRequest
    TheAVSSim.ConfigureStopTime(int(60 * 100 * 1E9))
    TheAVSSim.ExecuteSimulation()


if __name__ == "__main__":
    TheAVSSim = AVSSim.AVSSim()
    TheAVSSim.TotalSim.logThisMessage("acs_thruster_cmds", int(1E8))
    TheAVSSim.TotalSim.logThisMessage("inertial_state_output", int(1E9))
    TheAVSSim.TotalSim.logThisMessage("OrbitalElements", int(1E9))
    TheAVSSim.AddVariableForLogging('CSSWlsEst.numActiveCss', int(1E8))
    TheAVSSim.AddVariableForLogging('errorDeadband.error', int(1E8))
    TheAVSSim.AddVariableForLogging('simpleDeadband.wasControlOff', int(1E8))
    TheAVSSim.AddVariableForLogging('simpleDeadband.attError', int(1E8))
    TheAVSSim.AddVariableForLogging('simpleDeadband.rateError', int(1E8))

    TheAVSSim.VehOrbElemObject.CurrentElem.a = 188767262.18 * 1000.0
    TheAVSSim.VehOrbElemObject.CurrentElem.e = 0.207501
    TheAVSSim.VehOrbElemObject.CurrentElem.i = 0.0
    TheAVSSim.VehOrbElemObject.CurrentElem.Omega = 0.0
    TheAVSSim.VehOrbElemObject.CurrentElem.omega = 0.0
    TheAVSSim.VehOrbElemObject.CurrentElem.f = 70.0 * math.pi / 180.0
    # Convert those OEs to cartesian
    TheAVSSim.VehOrbElemObject.Elements2Cartesian()
    PosVec = ctypes.cast(TheAVSSim.VehOrbElemObject.r_N.__long__(),
                         ctypes.POINTER(ctypes.c_double))
    VelVec = ctypes.cast(TheAVSSim.VehOrbElemObject.v_N.__long__(),
                         ctypes.POINTER(ctypes.c_double))
    TheAVSSim.VehDynObject.PositionInit = sim_model.DoubleVector([PosVec[0], PosVec[1], PosVec[2]])
    TheAVSSim.VehDynObject.VelocityInit = sim_model.DoubleVector([VelVec[0], VelVec[1], VelVec[2]])

    executeAVSSafeCapture(TheAVSSim)

    DataCSSTruth = TheAVSSim.GetLogVariableData('CSSConstelation.sensorList[0].sHatStr')
    FSWsHat = TheAVSSim.pullMessageLogData("css_wls_est.sHatBdy", range(3))
    solarArrayMiss = TheAVSSim.pullMessageLogData("solar_array_sun_bore.missAngle")
    numCSSActive = TheAVSSim.GetLogVariableData('CSSWlsEst.numActiveCss')
    controlTorque = TheAVSSim.pullMessageLogData("controlTorqueRaw.torqueRequestBody", range(3))


    #boolControlOff = TheAVSSim.GetLogVariableData('errorDeadband.boolWasControlOff')

    CSSEstAccuracyThresh = 17.5 * math.pi / 180.0
    accuracyFailCounter = checkCSSEstAccuracy(DataCSSTruth, FSWsHat,
                                              CSSEstAccuracyThresh)

    slewFinishTime = 150.0
    desiredSunBdy = [0.0, 0.0, 1.0]
    controlFailCounter = checkSlewAccuracy(DataCSSTruth, FSWsHat, CSSEstAccuracyThresh,
                                           slewFinishTime, desiredSunBdy)

    plt.figure(1)
    plt.title('Sun direction unit vector')
    plt.plot(FSWsHat[:, 0] * 1.0E-9, FSWsHat[:, 1], 'b', DataCSSTruth[:, 0] * 1.0E-9, DataCSSTruth[:, 1], 'b--')
    plt.plot(FSWsHat[:, 0] * 1.0E-9, FSWsHat[:, 2], 'g', DataCSSTruth[:, 0] * 1.0E-9, DataCSSTruth[:, 2], 'g--')
    plt.plot(FSWsHat[:, 0] * 1.0E-9, FSWsHat[:, 3], 'r', DataCSSTruth[:, 0] * 1.0E-9, DataCSSTruth[:, 3], 'r--')
    plt.legend(['$\hat{x}_1$', '$x_1$', '$\hat{x}_2$', '$x_2$', '$\hat{x}_3$', '$x_3$'])
    plt.xlabel('time [s]')
    plt.ylabel('unit vector component value [-]')

    plt.figure(2)
    plt.plot(numCSSActive[:, 0] * 1.0E-9, numCSSActive[:, 1])
    plt.xlabel('time [s]')
    plt.ylabel('Total number')
    plt.title('Active CSS')

    plt.figure(3)
    plt.plot(solarArrayMiss[:, 0] * 1.0E-9, solarArrayMiss[:, 1] * 180 / math.pi)
    plt.title('Solar Array Miss Angle')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [deg]')

    plt.figure(4)
    plt.plot(controlTorque[:, 0] * 1.0E-9, controlTorque[:, 1], 'b')
    plt.plot(controlTorque[:, 0] * 1.0E-9, controlTorque[:, 2], 'g')
    plt.plot(controlTorque[:, 0] * 1.0E-9, controlTorque[:, 3], 'r')
    plt.legend(['$Lr_1$', '$Lr_2$', '$Lr_3$'])
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [N m]')
    plt.title('Control Torque')


    def errorDeadbandMode():
        dbError = TheAVSSim.GetLogVariableData('errorDeadband.error')
        plt.figure(5)
        plt.plot(dbError[:, 0] * 1.0E-9, dbError[:, 1] * 180 / math.pi)
        plt.axhline(TheAVSSim.errorDeadbandData.innerThresh, color='green')
        plt.title('FSW Estimated Error')
        plt.xlabel('Time [s]')
        plt.ylabel('Angle [deg]')
        plt.axhline(TheAVSSim.errorDeadbandData.innerThresh * 180.0 / math.pi, color='green')
        plt.axhline(TheAVSSim.errorDeadbandData.outerThresh * 180.0 / math.pi, color='red')
        plt.legend(['error', 'inner thresh', 'outer thresh', 'control (0=ON, 1=OFF)'])

    def simpleDeadbandMode():
        attError = TheAVSSim.GetLogVariableData('simpleDeadband.attError')
        rateError = TheAVSSim.GetLogVariableData('simpleDeadband.rateError')
        print 'sigma_BR = ', attError[:, 1]
        print 'omega_BR = ', rateError[:, 1]
        wasControlOff = TheAVSSim.GetLogVariableData('simpleDeadband.wasControlOff')
        plt.figure(6)
        plt.plot(attError[:, 0] * 1.0E-9, attError[:, 1] * 180 / math.pi, 'b')
        plt.plot(rateError[:, 0] * 1.0E-9, rateError[:, 1] * 180 / math.pi, 'dodgerblue')
        plt.axhline(TheAVSSim.simpleDeadbandData.innerAttThresh * 180.0 / math.pi, color='green')
        plt.axhline(TheAVSSim.simpleDeadbandData.innerRateThresh * 180.0 / math.pi, color='lawngreen')
        plt.axhline(TheAVSSim.simpleDeadbandData.outerAttThresh * 180.0 / math.pi, color='red')
        plt.axhline(TheAVSSim.simpleDeadbandData.outerRateThresh * 180.0 / math.pi, color='lightsalmon')
        controlScaleFactor = TheAVSSim.simpleDeadbandData.outerAttThresh * 180 / math.pi
        plt.plot(wasControlOff[:, 0] * 1.0E-9, wasControlOff[:, 1] * controlScaleFactor, 'k--')
        plt.title('FSW Estimated Error')
        plt.xlabel('Time [s]')
        plt.legend(['$\sigma$ [deg]','$\omega$ [deg/s]',
                    '$\sigma_{low}$', '$\omega_{low}$', '$\sigma_{up}$', '$\omega_{up}$',
                    'control (0=ON, 1=OFF)'])

    # Uncomment next line if the deadbanding you are using is errorDeadband and you want to see control performance
    #errorDeadbandMode()

    # Uncomment next line if the deadbanding you are using is simpleDeadband and you want to see control performance
    simpleDeadbandMode()

    plt.show()

    if (len(sys.argv) > 1):
        if (sys.argv[1] == 'True'):
            plt.show()

            # sys.exit(accuracyFailCounter + controlFailCounter)
