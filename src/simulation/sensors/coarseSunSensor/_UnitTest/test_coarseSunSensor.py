
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


#
# Coarse Sun Sensor Unit Test
#
# Purpose:  Test the proper function of the coarse sun sensor (css) module.
#           For basic functionality, results are compared to simple truth values calculated using np.cos().
#           For noise testing, noiseless truth values are subtracted from the output and the standard deviation is compared
#           to the input standard deviation.
#           For css constellation set up, two identical constellations are set up with different methods and compared to
#           each other
# Creation Date:  May. 31, 2017
#

import os

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import coarseSunSensor
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion as om
from Basilisk.utilities import unitTestSupport
from matplotlib import pyplot as plt

path = os.path.dirname(os.path.abspath(__file__))

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize(
    "useConstellation, visibilityFactor, fov, kelly, scaleFactor, bias, noiseStd, albedoValue, sunDistInput, minIn, maxIn, errTol, name,               zLevel, lineWide",
    [
        (False, 1.0, np.pi / 2., 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 10., 1e-10, "plain", 0, 5.),
        (False, 0.5, np.pi / 2., 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 10., 1e-10, "eclipse", -1, 5.),
        (False, 1.0, 3 * np.pi / 8., 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 10., 1e-10, "fieldOfView", -2, 5.),
        (False, 1.0, np.pi / 2., 0.15, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 10., 1e-10, "kellyFactor", 1, 5.),
        (False, 1.0, np.pi / 2., 0.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 10., 1e-10, "scaleFactor", 2, 5.),
        (False, 1.0, np.pi / 2., 0.0, 1.0, 0.5, 0.0, 0.0, 1.0, 0.0, 10., 1e-10, "bias", 3, 5.),
        (False, 1.0, np.pi / 2., 0.0, 1.0, 0.0, 0.125, 0.0, 1.0, -10., 10., 3e-2, "deviation", -5, 1.),
        # low tolerance for std deviation comparison
        (False, 1.0, np.pi / 2., 0.0, 1.0, 0.0, 0.0, 0.5, 1.0, 0.0, 10., 1e-10, "albedo", -4, 5.),
        (False, 1.0, np.pi / 2., 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.25, 0.75, 1e-10, "saturation", 5, 2.),
        (False, 1.0, np.pi / 2., 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 10.0, 1e-10, "sunDistance", 4, 3.),
        (False, 0.5, 3 * np.pi / 8., 0.15, 2.0, 0.5, 0.0, 0.5, 2.0, 0.0, 10., 1e-10, "cleanCombined", -3, 5.),
        (False, 0.5, 3 * np.pi / 8., 0.15, 2.0, 0.5, 0.125, 0.5, 2.0, -10., 10., 3e-2, "combined", -6, 1.),
        (True, 1.0, np.pi / 2., 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 10., 1e-10, "constellation", 0, 1.)
    ])
# provide a unique test method name, starting with test_
def test_coarseSunSensor(show_plots, useConstellation, visibilityFactor, fov, kelly, scaleFactor, bias, noiseStd,
                         albedoValue, sunDistInput, minIn, maxIn, errTol, name, zLevel, lineWide):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, useConstellation, visibilityFactor, fov, kelly, scaleFactor, bias,
                                     noiseStd, albedoValue, sunDistInput, minIn, maxIn, errTol, name, zLevel, lineWide)
    assert testResults < 1, testMessage

    __tracebackhide__ = True


def run(show_plots, useConstellation, visibilityFactor, fov, kelly, scaleFactor, bias, noiseStd, albedoValue,
        sunDistInput, minIn, maxIn, errTol, name, zLevel, lineWide):
    #
    #   Sim Setup
    #
    testFailCount = 0
    testMessages = []
    testTaskName = "unitTestTask"
    testProcessName = "unitTestProcess"
    testTaskRate = macros.sec2nano(0.1)

    # Create a simulation container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # Ensure simulation is empty
    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))

    #
    #   Input Message Setup
    #   Creates inputs from sun, spacecraft, and eclipse so that those modules don't have to be included
    # Create dummy sun message
    sunPositionMsg = messaging.SpicePlanetStateMsgPayload()
    sunPositionMsg.PositionVector = [om.AU * 1000. * sunDistInput, 0.0, 0.0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunPositionMsg)

    # Create dummy spacecraft message
    satelliteStateMsg = messaging.SCStatesMsgPayload()
    satelliteStateMsg.r_BN_N = [0.0, 0.0, 0.0]
    angles = np.linspace(0., 2 * np.pi, 360)
    sigmas = np.zeros(len(angles))
    truthVector = np.cos(angles)  # set truth vector initially, modify below based on inputs
    for i in range(len(sigmas)):  # convert rotation angle about 3rd axis to MRP
        sigmas[i] = np.tan(angles[i] / 4.)  # This is iterated through in the execution for loop
    satelliteStateMsg.sigma_BN = [0., 0., sigmas[0]]
    scMsg = messaging.SCStatesMsg().write(satelliteStateMsg)

    # Calculate sun distance factor
    r_Sun_Sc = [0.0, 0.0, 0.0]
    r_Sun_Sc[0] = sunPositionMsg.PositionVector[0] - satelliteStateMsg.r_BN_N[0]
    r_Sun_Sc[1] = sunPositionMsg.PositionVector[1] - satelliteStateMsg.r_BN_N[1]
    r_Sun_Sc[2] = sunPositionMsg.PositionVector[2] - satelliteStateMsg.r_BN_N[2]
    sunDist = np.linalg.norm(r_Sun_Sc)
    sunDistanceFactor = ((om.AU * 1000.0) ** 2) / (sunDist ** 2)

    # create dummy eclipse message
    eclipseMsg = messaging.EclipseMsgPayload()
    eclipseMsg.shadowFactor = visibilityFactor
    ecMsg = messaging.EclipseMsg().write(eclipseMsg)

    def setupCSS(CSS):
        CSS.fov = fov
        CSS.kellyFactor = kelly
        CSS.scaleFactor = scaleFactor
        CSS.senBias = bias
        CSS.senNoiseStd = noiseStd
        CSS.albedoValue = albedoValue
        CSS.minOutput = minIn
        CSS.maxOutput = maxIn
        CSS.nHat_B = np.array([1., 0., 0.])
        CSS.sunInMsg.subscribeTo(sunMsg)
        CSS.stateInMsg.subscribeTo(scMsg)
        CSS.sunEclipseInMsg.subscribeTo(ecMsg)

    #
    #   Single CSS Setup
    #   Sets up a single CSS with inputs from the pytest parameterization
    singleCss = coarseSunSensor.CoarseSunSensor()
    singleCss.ModelTag = "singleCss"
    setupCSS(singleCss)
    unitTestSim.AddModelToTask(testTaskName, singleCss)

    #
    #   CSS Constellation Setup
    #   Sets up two identical constellations (P1 and P2) but uses different methods to establish nHat_B for the sensors.
    if useConstellation:
        cssP11 = coarseSunSensor.CoarseSunSensor()
        cssP11.ModelTag = "cssP11"
        setupCSS(cssP11)
        cssP12 = coarseSunSensor.CoarseSunSensor()
        cssP12.ModelTag = "cssP12"
        setupCSS(cssP12)
        cssP13 = coarseSunSensor.CoarseSunSensor()
        cssP13.ModelTag = "cssP13"
        setupCSS(cssP13)
        cssP14 = coarseSunSensor.CoarseSunSensor()
        cssP14.ModelTag = "cssP14"
        setupCSS(cssP14)
        cssP21 = coarseSunSensor.CoarseSunSensor()
        cssP21.ModelTag = "cssP21"
        setupCSS(cssP21)
        cssP22 = coarseSunSensor.CoarseSunSensor()
        cssP22.ModelTag = "cssP22"
        setupCSS(cssP22)
        cssP23 = coarseSunSensor.CoarseSunSensor()
        cssP23.ModelTag = "cssP23"
        setupCSS(cssP23)
        cssP24 = coarseSunSensor.CoarseSunSensor()
        cssP24.ModelTag = "cssP24"
        setupCSS(cssP24)

        # all sensors on a 45 degree, four sided pyramid mount
        cssP11.nHat_B = [1. / np.sqrt(2.), 0., -1. / np.sqrt(2.)]
        cssP12.nHat_B = [1. / np.sqrt(2.), 1. / np.sqrt(2.), 0.]
        cssP13.nHat_B = [1. / np.sqrt(2.), 0., 1. / np.sqrt(2)]
        cssP14.nHat_B = [1. / np.sqrt(2.), -1. / np.sqrt(2.), 0.]

        # all except cssP24 given non-zero platform frame. B4 is not changed so that the default is tested.
        cssP21.setBodyToPlatformDCM(np.pi / 2., np.pi / 2., np.pi / 2.)
        cssP22.setBodyToPlatformDCM(np.pi / 2., np.pi / 2., np.pi / 2.)
        cssP23.setBodyToPlatformDCM(np.pi / 2., np.pi / 2., np.pi / 2.)
        # cssP24 is not changed so that the default is tested to be identity

        cssP21.phi = np.pi / 4.
        cssP21.theta = 0.
        cssP22.phi = np.pi / 4.
        cssP22.theta = np.pi / 2.
        cssP23.phi = np.pi / 4.
        cssP23.theta = np.pi
        cssP24.phi = np.pi / 6.  # remember, the cssP24 frame is the B frame. This angle is cancelled by a perturbation.
        cssP24.theta = -np.pi / 8.  # This angle is also provided with a perturbation to test to perturbation functionality.

        cssP21.setUnitDirectionVectorWithPerturbation(0., 0.)
        cssP22.setUnitDirectionVectorWithPerturbation(0., 0.)
        cssP23.setUnitDirectionVectorWithPerturbation(0., 0.)
        cssP24.setUnitDirectionVectorWithPerturbation(-np.pi / 8., -np.pi / 6.)

        constellationP1List = [cssP11, cssP12, cssP13,
                               cssP14]  # P1 is second platform, numbers following P2 are sensor numbers

        constellationP1 = coarseSunSensor.CSSConstellation()
        constellationP1.ModelTag = "constellationP1"
        for item in constellationP1List:
            constellationP1.appendCSS(item)
        unitTestSim.AddModelToTask(testTaskName, constellationP1)

        constellationP2List = [cssP21, cssP22, cssP23, cssP24]  # P2 is second platform, numbers following P2 are sensor numbers
        constellationP2 = coarseSunSensor.CSSConstellation()
        constellationP2.ModelTag = "constellationP2"
        for item in constellationP2List:
            constellationP2.appendCSS(item)
        unitTestSim.AddModelToTask(testTaskName, constellationP2)

        dataLogP1 = constellationP1.constellationOutMsg.recorder()
        dataLogP2 = constellationP2.constellationOutMsg.recorder()
        unitTestSim.AddModelToTask(testTaskName, dataLogP1)
        unitTestSim.AddModelToTask(testTaskName, dataLogP2)

    # log single CSS
    dataLogSingle = singleCss.cssDataOutMsg.recorder()
    unitTestSim.AddModelToTask(testTaskName, dataLogSingle)

    #
    #   Modify Truth Vector Appropriately
    #
    for i in range(len(truthVector)):
        if kelly > 0.0000000000001:  # only if kelly isn't actually zero
            truthVector[i] = truthVector[i] * (
                    1.0 - np.e ** (-truthVector[i] ** 2.0 / kelly))  # apply kelly factor, note: no albedo
        truthVector[i] = truthVector[i] * visibilityFactor * sunDistanceFactor  # account for eclipse effects
        truthVector[i] += albedoValue  # apply albedo
        truthVector[i] += bias  # apply bias
    for i in range(len(angles)):
        if angles[i] > fov and angles[i] < (2 * np.pi - fov):  # first, trim to fov
            truthVector[i] = 0.0
            truthVector[i] += albedoValue  # apply albedo
            truthVector[i] += bias
    truthVector = truthVector * scaleFactor
    for i in range(len(truthVector)):
        truthVector[i] = min([truthVector[i], maxIn])
        truthVector[i] = max([truthVector[i], minIn])

    #
    #   Initialize and run simulation one step at a time
    #
    unitTestSim.InitializeSimulation()

    # Execute the simulation for one time step
    for i in range(len(sigmas)):
        satelliteStateMsg.sigma_BN = [0.0, 0.0, sigmas[i]]
        scMsg.write(satelliteStateMsg, unitTestSim.TotalSim.CurrentNanos + testTaskRate)
        unitTestSim.TotalSim.SingleStepProcesses()

    #
    #   Constellation Outputs and plots
    #
    cssOutput = dataLogSingle.OutputData
    if useConstellation:
        constellationP1data = dataLogP1.CosValue
        constellationP2data = dataLogP2.CosValue

        plt.figure(1, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.clf()
        plt.subplot(2, 1, 1)
        for i in range(4):
            sensorlabel = "cssP1" + str(i + 1)
            plt.plot(dataLogP1.times() * macros.NANO2MIN, constellationP1data[:, i], label=sensorlabel,
                     linewidth=4 - i)
        plt.xlabel('Time [min]')
        plt.ylabel('P1 Output Values [-]')
        plt.legend(loc='upper center')

        plt.subplot(2, 1, 2)
        # plt.figure(2,figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        for i in range(4):
            sensorlabel = "cssP2" + str(i + 1)
            plt.plot(dataLogP2.times() * macros.NANO2MIN, constellationP2data[:, i], label=sensorlabel,
                     linewidth=4 - i)
        plt.xlabel('Time [min]')
        plt.ylabel('P2 Output Values [-]')
        plt.legend(loc='upper center')
        unitTestSupport.writeFigureLaTeX('constellationPlots',
                                         'Plot of first and second constellation outputs for comparision.\
                                          Note that the constellation starts pointing directly at the sun\
                                           and linearly rotates in time until it returns to a direct view.',
                                         plt, 'height=0.7\\textwidth, keepaspectratio', path)
    #
    #   Single CSS plotting
    #
    else:
        justTheNoise = cssOutput - truthVector  # subtract curve from noisy curve
        outputStd = np.std(justTheNoise)
        plt.figure(3, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.plot(dataLogSingle.times() * macros.NANO2MIN, cssOutput, label=name, zorder=zLevel, linewidth=lineWide)
        plt.legend()
        plt.xlabel('Time [min]')
        plt.ylabel('Output Value [-]')
        if name == "combined":
            unitTestSupport.writeFigureLaTeX('combinedPlot',
                                             'Plot of all cases of individual coarse sun sensor in comparison to\
                                              each other. Note that the incidence angle starts at direct and linearly\
                                               rotates in time until it returns to a direct view.',
                                             plt, 'height=0.7\\textwidth, keepaspectratio', path)

    if name == "constellation" and show_plots:  # Don't show plots until last run.
        plt.show()
        plt.close('all')

    #
    #   Compare output and truth vectors
    #
    if useConstellation:  # compare constellation P1 to constellation P2
        for i in range(0, np.shape(constellationP2data)[0]):
            if not unitTestSupport.isArrayEqualRelative(constellationP2data[i][:], constellationP1data[i][0:], 4,
                                                        errTol):
                testFailCount += 1
    elif noiseStd == 0.0:  # if a test without noise
        for i in range(0, np.shape(cssOutput)[0]):
            if cssOutput[i] == 0.0:
                if not unitTestSupport.isArrayZero([cssOutput[i]], 1, errTol):
                    testFailCount += 1
            else:
                if not unitTestSupport.isDoubleEqualRelative(cssOutput[i], truthVector[i], errTol):
                    testFailCount += 1
    else:  # if "combined" or "deviation"
        if not unitTestSupport.isDoubleEqualRelative(noiseStd * scaleFactor, outputStd, errTol):
            print(outputStd)
            print(noiseStd * scaleFactor)
            testFailCount += 1
            print("HPS: 2")

    if testFailCount == 0:
        colorText = 'ForestGreen'
        passFailMsg = ""  # "Passed: " + name + "."
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        passFailMsg = "Failed: " + name + "."
        testMessages.append(passFailMsg)
        testMessages.append(" | ")
        passedText = r'\textcolor{' + colorText + '}{' + "FAILED" + '}'

    # Write some snippets for AutoTex
    snippetName = name + "PassedText"
    snippetContent = passedText
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)

    snippetName = name + "PassFailMsg"
    snippetContent = passFailMsg
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)
    print("\n", passFailMsg)

    # write pytest parameters to AutoTex folder
    # "useConstellation, visibilityFactor, fov, kelly, scaleFactor, bias, noiseStd, albedoValue, errTol, name, zLevel, lineWide"
    useConstellationSnippetName = name + "UseConstellation"
    useConstellationSnippetContent = str(useConstellation)
    unitTestSupport.writeTeXSnippet(useConstellationSnippetName, useConstellationSnippetContent, path)

    visibilityFactorSnippetName = name + "VisibilityFactor"
    visibilityFactorSnippetContent = '{:1.2f}'.format(visibilityFactor)
    unitTestSupport.writeTeXSnippet(visibilityFactorSnippetName, visibilityFactorSnippetContent, path)

    fovSnippetName = name + "Fov"
    fovSnippetContent = '{:1.4f}'.format(fov)
    unitTestSupport.writeTeXSnippet(fovSnippetName, fovSnippetContent, path)

    kellySnippetName = name + "Kelly"
    kellySnippetContent = '{:1.2f}'.format(kelly)
    unitTestSupport.writeTeXSnippet(kellySnippetName, kellySnippetContent, path)

    scaleFactorSnippetName = name + "ScaleFactor"
    scaleFactorSnippetContent = '{:1.2f}'.format(scaleFactor)
    unitTestSupport.writeTeXSnippet(scaleFactorSnippetName, scaleFactorSnippetContent, path)

    biasSnippetName = name + "Bias"
    biasSnippetContent = '{:1.2f}'.format(bias)
    unitTestSupport.writeTeXSnippet(biasSnippetName, biasSnippetContent, path)

    noiseStdSnippetName = name + "NoiseStd"
    noiseStdSnippetContent = '{:1.3f}'.format(noiseStd)
    unitTestSupport.writeTeXSnippet(noiseStdSnippetName, noiseStdSnippetContent, path)

    albedoValueSnippetName = name + "AlbedoValue"
    albedoValueSnippetContent = '{:1.1f}'.format(albedoValue)
    unitTestSupport.writeTeXSnippet(albedoValueSnippetName, albedoValueSnippetContent, path)

    locationSnippetName = name + "Location"
    locationSnippetContent = '{:1.1f}'.format(sunDistInput)
    unitTestSupport.writeTeXSnippet(locationSnippetName, locationSnippetContent, path)

    saturationMaxSnippetName = name + "MaxSaturation"
    saturationMaxSnippetContent = '{:2.2f}'.format(maxIn)
    unitTestSupport.writeTeXSnippet(saturationMaxSnippetName, saturationMaxSnippetContent, path)

    saturationMinSnippetName = name + "MinSaturation"
    saturationMinSnippetContent = '{:2.2f}'.format(minIn)
    unitTestSupport.writeTeXSnippet(saturationMinSnippetName, saturationMinSnippetContent, path)

    errTolSnippetName = name + "ErrTol"
    errTolSnippetContent = '{:1.1e}'.format(errTol)
    unitTestSupport.writeTeXSnippet(errTolSnippetName, errTolSnippetContent, path)

    if testFailCount == 0:
        print("PASSED")
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    # run(True, False, 1.0, np.pi / 2., 0.0, 1.0, 0.0, 0.125, 0.0, 1.0, -10., 10., 1e-2, "deviation", -5, 1.)
    # run(True, False, 1.0, np.pi / 2., 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 10., 1e-10, "plain", 0, 5.)
    run(True, True, 1.0, np.pi / 2., 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 10., 1e-10, "constellation", 0, 1.)
    # run(True, False, 0.5, 3 * np.pi / 8., 0.15, 2.0, 0.5, 0.125, 0.5, 2.0, -10., 10., 1e-2, "combined", -6, 1.0)
