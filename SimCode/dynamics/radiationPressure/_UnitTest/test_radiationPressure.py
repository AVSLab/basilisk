'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
# RadiationPressure Unit Test
#
# Purpose:  Test the proper function of the Radiation Pressure Dynamics module.
#           This is done by comparing expected torques and forces to
#           what is sumulated
# Author:   Patrick Kenneally
# Creation Date:  Feb. 9, 2017
#

# @cond DOXYGEN_IGNORE
import sys
import os
import numpy as np
import math
import pytest
import inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')
# @endcond

#Import all of the modules that we are going to call in this simulation
import unitTestSupport
import matplotlib.pyplot as plt
import MessagingAccess
import SimulationBaseClass
import sim_model
import stateArchitecture
import spacecraftPlus
import radiation_pressure
import macros
import simIncludeGravity
import orbitalMotion
import spice_interface
import astroFunctions


# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
@pytest.mark.parametrize("modelType", ["cannonball"])
def test_unitRadiationPressure(show_plots, modelType):
    [testResults, testMessage] = unitRadiationPressure(show_plots, modelType)
    assert testResults < 1, testMessage


def unitRadiationPressure(show_plots, modelType):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0
    testMessages = []
    testTaskName = "unitTestTask"
    testProcessName = "unitTestProcess"
    testTaskRate = macros.sec2nano(0.1)
    simulationTime = 0.2
    r_N = [-16937711153.5, -138435806556.0, -60051616256.6]  # [m]
    sun_r_N = [507128401.716, 22652490.9092, -14854379.6232]  # [m]
    sigma_BN = [0.1, 0.2, -0.3]

    # Create a simulation container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # Ensure simulation is empty
    unitTestSim.TotalSim.terminateSimulation()
    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))

    srpDynEffector = radiation_pressure.RadiationPressure()
    srpDynEffector.ModelTag = "RadiationPressure"

    if modelType == "cannonball":
        srpDynEffector.setUseCannonballModel(True)
        srpDynEffector.area = 4
        srpDynEffector.coefficientReflection = 1.2
    elif modelType == "lookup":
        srpDynEffector.setUseCannonballModel(False)
        handler = radiation_pressure.SRPLookupTableHandler()
        handler.parseAndLoadXML(__file__ + "cube.xml")
        for i in xrange(0, len(handler.forceBLookup)):
            srpDynEffector.addForceLookupBEntry(unitTestSupport.np2EigenVectorXd(handler.forceBLookup[i, :]))
            srpDynEffector.addTorqueLookupBEntry(unitTestSupport.np2EigenVectorXd(handler.torqueBLookup[i, :]))
            srpDynEffector.addSHatLookupBEntry(unitTestSupport.np2EigenVectorXd(handler.sHatBLookup[i, :]))

    unitTestSim.AddModelToTask(testTaskName, srpDynEffector, None, 3)

    scPlusStateMsg = spacecraftPlus.SCPlusOutputStateData()
    scPlusStateMsgName = "inertial_state_output"
    unitTestSim.TotalSim.CreateNewMessage(testProcessName, scPlusStateMsgName, scPlusStateMsg.getStructSize(), 2)
    scPlusStateMsg.r_BN_N = r_N
    scPlusStateMsg.sigma_BN = sigma_BN
    unitTestSim.TotalSim.WriteMessageData(scPlusStateMsgName, scPlusStateMsg.getStructSize(), 1, scPlusStateMsg)

    sunSpiceMsg = spice_interface.SpicePlanetState()
    sunSpiceMsgName = "sun_planet_data"
    unitTestSim.TotalSim.CreateNewMessage(testProcessName, sunSpiceMsgName, sunSpiceMsg.getStructSize(), 2)
    sunSpiceMsg.PositionVector = sun_r_N
    unitTestSim.TotalSim.WriteMessageData(sunSpiceMsgName, sunSpiceMsg.getStructSize(), 1, sunSpiceMsg)

    # dynParamManager = stateArchitecture.DynParamManager()
    # dynParamManager.createProperty("hubSigma", sigma_BN)

    # Call linkInStates on srp module manually because it won't be called
    # by the dynamics engine because we are not connecting it to the engine
    # srpDynEffector.linkInStates(dynParamManager)

    # clear prior gravitational body and SPICE setup definitions
    # simIncludeGravity.clearSetup()
    # simIncludeGravity.addEarth()
    # earthGravBody = simIncludeGravity.gravBodyList[-1]
    # earthGravBody.isCentralBody = True
    # simIncludeGravity.addSun()
    # sunGravBody = simIncludeGravity.gravBodyList[-1]

    # attach gravity model to spaceCraftPlus
    # scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)
    #
    # # setup SPICE interface
    # simIncludeGravity.addSpiceInterface(splitPath[0], "2015 June 15, 00:00:00.0")
    # spiceInterface = simIncludeGravity.spiceObject

    # by default the SPICE object will use the solar system barycenter as the inertial origin
    # If the spacecraftPlus() output is desired relative to another celestial object, the zeroBase string
    # name of the SPICE object needs to be changed.
    # simIncludeGravity.spiceObject.zeroBase = 'Earth'

    # attach gravity model to spaceCraftPlus
    # scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)
    # unitTestSim.AddModelToTask(testTaskName, simIncludeGravity.spiceObject, None, 2)



    # set the simulation time
    # n = np.sqrt(earthGravBody.mu / oe.a / oe.a / oe.a)
    # period = 2. * np.pi / n

    # numDataPoints = 1
    # samplingTime = simulationTime / (numDataPoints - 1)
    samplingTime = simulationTime

    # unitTestSim.TotalSim.logThisMessage("sun_planet_data", samplingTime)
    # unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    unitTestSim.AddVariableForLogging(srpDynEffector.ModelTag + ".forceExternal_B",
                                      samplingTime, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(srpDynEffector.ModelTag + ".forceExternal_N",
                                      samplingTime, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(srpDynEffector.ModelTag + ".torqueExternalPntB_B",
                                      samplingTime, 0, 2, 'double')

    unitTestSim.InitializeSimulation()

    # posRef = scObject.dynManager.getStateObject("hubPosition")
    # velRef = scObject.dynManager.getStateObject("hubVelocity")
    # posRef.setState(unitTestSupport.np2EigenVectorXd(r_N))  # [m - r_BN_N
    # velRef.setState(unitTestSupport.np2EigenVectorXd(v_N))  # m - v_BN_N

    # sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    # omegaRef = scObject.dynManager.getStateObject("hubOmega")
    # sigmaRef.setState([[0.1], [0.2], [-0.3]])  # sigma_BN_B
    # omegaRef.setState([[0.001], [-0.001], [0.003]])  # rad/s omega_BN_B

    # Configure a simulation stop time time and execute the simulation run
    unitTestSim.ConfigureStopTime(simulationTime)
    unitTestSim.ExecuteSimulation()
    srpDynEffector.computeBodyForceTorque(unitTestSim.TotalSim.CurrentNanos)
    unitTestSim.TotalSim.SingleStepProcesses()
    unitTestSim.RecordLogVars()

    # Retrieve the logged data
    # sigmaData = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName + '.sigma_BN', range(3))
    # posData = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', range(3))
    # rSunData_N = unitTestSim.pullMessageLogData("sun_planet_data" + '.PositionVector', range(3))

    srpDataForce_B = unitTestSim.GetLogVariableData(srpDynEffector.ModelTag + ".forceExternal_B")
    srpDataForce_N = unitTestSim.GetLogVariableData(srpDynEffector.ModelTag + ".forceExternal_N")
    srpTorqueData = unitTestSim.GetLogVariableData(srpDynEffector.ModelTag + ".torqueExternalPntB_B")

    #
    #
    errTol = 10E-9
    if modelType == "cannonball":
        truthForceExternal_B = [-1.85556867482797E-05, 8.82911772465848E-06, -5.64107588371567E-06]
        truthForceExternal_N = [0, 0, 0]
        truthTorqueExternalPntB_B = [0, 0, 0]
        testFailCount, testMessages = unitTestSupport.compareVector(truthForceExternal_B,
                                                                    srpDataForce_B[2,1:],
                                                                    errTol,
                                                                    "Force_B",
                                                                    testFailCount,
                                                                    testMessages)


    # if modelType == "lookup":

    if testFailCount == 0:
        print "PASSED: " + modelType
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    unitRadiationPressure(False, "cannoball")
