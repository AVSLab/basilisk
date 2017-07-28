''' '''
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
# Eclipse Condition Unit Test
#
# Purpose:  Test the proper function of the Eclipse environment module.
#           This is done by comparing computed expected shadow factors in
#           particular eclipse conditions to what is simulated
# Author:   Patrick Kenneally
# Creation Date:  May. 31, 2017
#

# @cond DOXYGEN_IGNORE
import sys
import os
import pytest
import inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')
# @endcond

import SimulationBaseClass
import unitTestSupport
import spacecraftPlus
import macros
import spice_interface
import eclipse
import pyswice
import gravityEffector
import orbitalMotion as om
import coarse_sun_sensor
import numpy as np
import RigidBodyKinematics as rbk
from matplotlib import pyplot as plt
import simMessages


# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
# @pytest.mark.parametrize()
# def test_unitCoarseSunSensor(show_plots, eclipseCondition):
#     [testResults, testMessage] = unitCoarseSunSensor(show_plots, eclipseCondition)
#     assert testResults < 1, testMessage


# def unitCoarseSunSensor():
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
__tracebackhide__ = True

useConstellation = True

##################Single sensor tests######################
#--------variables will come from pytest parameterization later
visibilityFactor = 1. #no eclipse = 1., eclipse = 0., anywhere in between works.
fov = np.pi/2.
kelly = 0.0
scaleFactor = 1.
bias = 0.0
noiseStd = 0.0
albedoValue = 0.0
nHat_B = [1.,0.,0.]
sigmas = np.linspace(0., 2*np.pi, 5900)
for i in range(len(sigmas)): #convert rotation angle about 3rd axis to MRP
    sigmas[i] = np.tan(sigmas[i]/4.)
#---------

testFailCount = 0
testMessages = []
testTaskName = "unitTestTask"
testProcessName = "unitTestProcess"
testTaskRate = macros.sec2nano(0.1)

# Create a simulation container
unitTestSim = SimulationBaseClass.SimBaseClass()
# Ensure simulation is empty
unitTestSim.TotalSim.terminateSimulation()
testProc = unitTestSim.CreateNewProcess(testProcessName)
testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))

cssSinglePlain = coarse_sun_sensor.CoarseSunSensor()
cssSinglePlain.ModelTag = "cssSinglePlain"
cssSinglePlain.fov = fov
cssSinglePlain.KellyFactor = kelly
cssSinglePlain.scaleFactor = scaleFactor
cssSinglePlain.SenBias = bias
cssSinglePlain.SenNoiseStd = noiseStd
cssSinglePlain.OutputDataMsg = "cssSinglePlainOut"
cssSinglePlain.nHat_B = nHat_B
cssSinglePlain.sunEclipseInMsgName = "eclipseMsg"
cssSinglePlain.InputSunMsg = "sunMsg"
cssSinglePlain.InputStateMsg = "satelliteState"
unitTestSim.AddModelToTask(testTaskName,cssSinglePlain)

if useConstellation:
    cssA1 = coarse_sun_sensor.CoarseSunSensor(cssSinglePlain)
    cssA2 = coarse_sun_sensor.CoarseSunSensor(cssSinglePlain)
    cssA3 = coarse_sun_sensor.CoarseSunSensor(cssSinglePlain)
    cssA4 = coarse_sun_sensor.CoarseSunSensor(cssSinglePlain)
    cssB1 = coarse_sun_sensor.CoarseSunSensor(cssSinglePlain)
    cssB2 = coarse_sun_sensor.CoarseSunSensor(cssSinglePlain)
    cssB3 = coarse_sun_sensor.CoarseSunSensor(cssSinglePlain)
    cssB4 = coarse_sun_sensor.CoarseSunSensor(cssSinglePlain)

    cssA1.OutputDataMsg = "cssA1Out"
    cssA2.OutputDataMsg = "cssA2Out"
    cssA3.OutputDataMsg = "cssA3Out"
    cssA4.OutputDataMsg = "cssA4Out"
    cssB1.OutputDataMsg = "cssB1Out"
    cssB2.OutputDataMsg = "cssB2Out"
    cssB3.OutputDataMsg = "cssB3Out"
    cssB4.OutputDataMsg = "cssB4Out"


    #all sensors on a 45 degree, four sided pyramid mount
    cssA1.nHat_B = [1./np.sqrt(2.), 0., -1./np.sqrt(2.)]
    cssA2.nHat_B = [1./np.sqrt(2.), 1./np.sqrt(2.), 0.]
    cssA3.nHat_B = [1./np.sqrt(2.), 0.,  1./np.sqrt(2)]
    cssA4.nHat_B = [1./np.sqrt(2.), -1./np.sqrt(2.), 0.]

    #all except cssB4 given non-zero platform frame. B4 is not changed so that the default is tested.
    cssB1.setBodyToPlatformDCM(np.pi/2., np.pi/2., np.pi/2.)
    cssB2.setBodyToPlatformDCM(np.pi/2., np.pi/2., np.pi/2.)
    cssB3.setBodyToPlatformDCM(np.pi/2., np.pi/2., np.pi/2.)
    #cssB4 is not changed so that the default is tested to be identity

    cssB1.phi = -3.*np.pi/4.
    cssB1.theta = 0.
    cssB2.phi = 0.
    cssB2.theta = 0.
    cssB3.phi = 0.
    cssB3.theta = 0.
    cssB4.phi = 0.
    cssB4.theta = 0.

    cssB1.setUnitDirectionVectorWithPerturbation(cssB1.theta, cssB1.phi)
    cssB2.setUnitDirectionVectorWithPerturbation(cssB2.theta, cssB2.phi)
    print cssB1.dcm_PB
    cssB3.setUnitDirectionVectorWithPerturbation(cssB3.theta, cssB3.phi)
    cssB4.setUnitDirectionVectorWithPerturbation(cssB4.theta, cssB4.phi)
    
    constellationAList = [cssA1, cssA2, cssA3, cssA4]
    constellationA = coarse_sun_sensor.CSSConstellation()
    constellationA.ModelTag = "constellationA"
    constellationA.sensorList = coarse_sun_sensor.CSSVector(constellationAList)
    constellationA.outputConstellationMessage = "constellationA_Array_output"
    unitTestSim.AddModelToTask(testTaskName, constellationA)

    constellationBList = [cssB1, cssB2, cssB3, cssB4]
    constellationB = coarse_sun_sensor.CSSConstellation()
    constellationB.ModelTag = "constellationB"
    constellationB.sensorList = coarse_sun_sensor.CSSVector(constellationBList)
    constellationB.outputConstellationMessage = "constellationB_Array_output"
    unitTestSim.AddModelToTask(testTaskName, constellationB)

    unitTestSim.TotalSim.logThisMessage(constellationA.outputConstellationMessage, testTaskRate)
    unitTestSim.TotalSim.logThisMessage(constellationB.outputConstellationMessage, testTaskRate)


#Create dummy sun message
sunPositionMsg = simMessages.SpicePlanetStateSimMsg()
sunPositionMsg.PositionVector = [om.AU, 0.0, 0.0]
unitTestSupport.setMessage(unitTestSim.TotalSim,
                           testProcessName,
                           cssSinglePlain.InputSunMsg,
                           sunPositionMsg)

#Create dummy spacecraft message
satelliteStateMsg = simMessages.SCPlusStatesSimMsg()
satelliteStateMsg.r_BN_N = [0.0, 0.0, 0.0]
satelliteStateMsg.sigma_BN = [0.,0.,sigmas[0]]
unitTestSupport.setMessage(unitTestSim.TotalSim,
                           testProcessName,
                           cssSinglePlain.InputStateMsg,
                           satelliteStateMsg)
satelliteStateMsgSize = satelliteStateMsg.getStructSize()

#create dummy eclipse message
eclipseMsg = simMessages.EclipseSimMsg()
eclipseMsg.shadowFactor = visibilityFactor
unitTestSupport.setMessage(unitTestSim.TotalSim,
                           testProcessName,
                           cssSinglePlain.sunEclipseInMsgName,
                           eclipseMsg)

unitTestSim.InitializeSimulationAndDiscover()
unitTestSim.TotalSim.logThisMessage(cssSinglePlain.OutputDataMsg, macros.sec2nano(0.1))

# Execute the simulation for one time step
for i in range(len(sigmas)):
    satelliteStateMsg.sigma_BN = [0.,0.,sigmas[i]]
    unitTestSim.TotalSim.WriteMessageData(cssSinglePlain.InputStateMsg, satelliteStateMsgSize,
                                      unitTestSim.TotalSim.CurrentNanos + testTaskRate,
                                      satelliteStateMsg)
    unitTestSim.TotalSim.SingleStepProcesses()

cssOutput = unitTestSim.pullMessageLogData(cssSinglePlain.OutputDataMsg+".OutputData", range(1))
if useConstellation:
    constellationAdata = unitTestSim.pullMessageLogData(constellationA.outputConstellationMessage + ".CosValue", range(len(constellationAList)))
    constellationBdata = unitTestSim.pullMessageLogData(constellationB.outputConstellationMessage + ".CosValue", range(len(constellationBList)))

plt.figure(1)
plt.clf()
for i in range(4):
    sensorlabel = "cssA"+str(i+1)
    plt.plot(constellationAdata[:,0], constellationAdata[:,i+1], label=sensorlabel, linewidth=4-i)
plt.legend(loc='upper center')

plt.figure(2)
plt.clf()
for i in range(4):
    sensorlabel = "cssB"+str(i+1)
    plt.plot(constellationBdata[:,0], constellationBdata[:,i+1], label=sensorlabel, linewidth=4-i)
plt.legend(loc='upper center')

# plt.figure(3)
# plt.clf()
# plt.plot(cssOutput[:,0]*macros.NANO2MIN , cssOutput[:,1])

plt.show()

# plt.figure(1)
# plt.clf()

# plt.show()
########################end Single Sensor

#eclipseData_0 = unitTestSim.pullMessageLogData("eclipse_data_0.shadowFactor")

# errTol = 1E-12
# if eclipseCondition is "partial":
#     truthShadowFactor = 0.62310760206735027
#     if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
#         testFailCount += 1
#         testMessages.append("Shadow Factor failed for partial eclipse condition")
#
# elif eclipseCondition is "full":
#     truthShadowFactor = 0.0
#     if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
#         testFailCount += 1
#         testMessages.append("Shadow Factor failed for full eclipse condition")
#
# elif eclipseCondition is "none":
#     truthShadowFactor = 1.0
#     if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
#         testFailCount += 1
#         testMessages.append("Shadow Factor failed for none eclipse condition")
#
# if testFailCount == 0:
#     print "PASSED: " + eclipseCondition
# return fail count and join into a single string all messages in the list
# testMessage

# return [testFailCount, ''.join(testMessages)]

# if __name__ == "__main__":
#     unitCoarseSunSensor(False, "full")
