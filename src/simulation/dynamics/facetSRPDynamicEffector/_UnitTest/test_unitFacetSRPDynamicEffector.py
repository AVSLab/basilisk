
# ISC License
#
# Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# Purpose:  Test the facetSRPDynamicEffector module.
# Author:   Leah Kiner
# Creation Date:  Dec 18 2022
#


import os
import pytest
import inspect
import numpy as np
from Basilisk import __path__
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
import matplotlib
import matplotlib.pyplot as plt
from Basilisk.utilities import orbitalMotion, simIncludeGravBody
from Basilisk.simulation import spacecraft, facetSRPDynamicEffector
from Basilisk.utilities import macros, RigidBodyKinematics as rbk
from Basilisk.architecture import messaging
bskPath = __path__[0]
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')
matplotlib.rc('xtick', labelsize=12)
matplotlib.rc('ytick', labelsize=12)

def test_facetSRPDynamicEffector(show_plots):
    r"""
    **Validation Test Description**

    The unit test for this module ensures that the calculated Solar Radiation Pressure (SRP) force and torque acting
    on the spacecraft about the body-fixed point B is properly computed for a static spacecraft. The spacecraft
    geometry defined in this test consists of a cubic hub and two circular solar arrays. Six square facets represent
    the cubic hub and four circular facets describe the solar arrays. The SRP force and torque module values are
    checked with values computed in python both initially and at a time halfway through the simulation.

    **Test Parameters**

    Args:
        show_plots (bool): (True) Show plots, (False) Do not show plots

    **Description of Variables Being Tested**

    The initial module-computed SRP force value ``SRPDataForce_B`` is checked with the value computed in
    python ``forceTest1Val``. The initial module-computed SRP torque value ``SRPDataTorque_B`` is also checked
    with the value computed in python ``torqueTest1Val``. Similarly, these values halfway through the simulation
    are checked to match.
    """
    testResults = []
    testMessage = []

    srpRes, srpMsg = TestfacetSRPDynamicEffector(False)
    testMessage.append(srpMsg)
    testResults.append(srpRes)

    testSum = sum(testResults)
    snippetName = "unitTestPassFail"

    if testSum == 0:
        colorText = 'ForestGreen'
        print("PASSED")
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed")
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'

    unitTestSupport.writeTeXSnippet(snippetName, passedText, path)

    assert testSum < 1, testMessage

def checkFacetSRPForce(index, area, specCoeff, diffCoeff, normal_B, sigma_BN, scPos, sunPos):
    """This function calculates the per-facet SRP force acting on the spacecraft."""
    # Define required constants
    speedLight = 299792458.0
    AstU = 149597870700.0
    solarRadFlux = 1368.0

    # Compute dcm_BN using MRP transformation
    dcm_BN = rbk.MRP2C(sigma_BN)

    # Convert vectors from the N frame to the B frame
    r_BN_B = np.matmul(dcm_BN, scPos)
    r_SN_B = np.matmul(dcm_BN, sunPos)
    r_SB_B = r_SN_B - r_BN_B

    # Determine unit direction vector pointing from point B on the spacecraft to the Sun
    sHat = r_SB_B / np.linalg.norm(r_SB_B)

    # Calculate the incidence angle theta between the facet normal vector and the Sun-direction vector
    cosTheta = np.dot(sHat, normal_B)
    intermediate = np.cross(sHat, normal_B)
    sinTheta = np.linalg.norm(intermediate)
    theta = np.arctan2(sinTheta, cosTheta)

    # Calculate the facet projected area onto the plane whose normal vector is the Sun-direction vector
    projArea = area * np.cos(theta)

    # Calculate the solar radiation pressure acting on the facet
    numAU = AstU / np.linalg.norm(r_SB_B)
    SRPPressure = (solarRadFlux / speedLight) * numAU * numAU

    # Compute the SRP force acting on the facet only if the facet is illuminated by the Sun
    if projArea > 0:
        srp_force = -SRPPressure * projArea * np.cos(theta) * ( (1-specCoeff) * sHat + 2 * ( (diffCoeff / 3) + specCoeff * np.cos(theta)) * normal_B )
    else:
        srp_force = np.zeros([3,])

    return srp_force

def checkFacetSRPTorque(index, area, specCoeff, diffCoeff, normal_B, locationPntB_B, sigma_BN, scPos, sunPos):
    """This function calculates the per-facet SRP torque acting on the spacecraft."""
    # Define required constants
    speedLight = 299792458.0
    AstU = 149597870700.0
    solarRadFlux = 1368.0

    # Compute dcm_BN using MRP transformation
    dcm_BN = rbk.MRP2C(sigma_BN)

    # Convert vectors from the N frame to the B frame
    r_BN_B = np.matmul(dcm_BN, scPos)
    r_SN_B = np.matmul(dcm_BN, sunPos)
    r_SB_B = r_SN_B - r_BN_B

    # Determine unit direction vector pointing from point B on the spacecraft to the Sun
    sHat = r_SB_B / np.linalg.norm(r_SB_B)

    # Calculate the incidence angle theta between the facet normal vector and the Sun-direction vector
    cosTheta = np.dot(sHat, normal_B)
    intermediate = np.cross(sHat, normal_B)
    sinTheta = np.linalg.norm(intermediate)
    theta = np.arctan2(sinTheta, cosTheta)

    # Calculate the facet projected area onto the plane whose normal vector is the Sun-direction vector
    projArea = area * np.cos(theta)

    # Calculate the solar radiation pressure acting on the facet
    numAU = AstU / np.linalg.norm(r_SB_B)
    SRPPressure = (solarRadFlux / speedLight) * numAU * numAU

    # Compute the SRP force contribution from the facet only if the facet is illuminated by the Sun
    if projArea > 0:
        srp_force = -SRPPressure * projArea * np.cos(theta) * ( (1-specCoeff) * sHat + 2 * ( (diffCoeff / 3) + specCoeff * np.cos(theta)) * normal_B )
    else:
        srp_force = np.zeros([3, ])

    # Calculate the SRP torque contribution from the facet
    srp_torque = np.cross(locationPntB_B, srp_force)

    return srp_torque


def TestfacetSRPDynamicEffector(show_plots):
    """Call this routine directly to run the unit test."""
    # Init test support variables
    testFailCount = 0
    testMessages = []

    # Create the simulation task and process
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep_Sec = 0.1
    simulationTimeStep_NS = macros.sec2nano(simulationTimeStep_Sec)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep_NS))

    # Create the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraft"

    # Add the Earth and Sun as gravitational bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    sun = gravFactory.createSun()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    earthIdx = 0
    sunIdx = 1
    earthStateMsg = messaging.SpicePlanetStateMsgPayload()
    earthStateMsg.PositionVector = [0.0, -149598023 * 1000, 0.0]
    earthStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    earthMsg = messaging.SpicePlanetStateMsg().write(earthStateMsg)
    gravFactory.gravBodies['earth'].planetBodyInMsg.subscribeTo(earthMsg)
    sunStateMsg = messaging.SpicePlanetStateMsgPayload()
    sunStateMsg.PositionVector = [0.0, 0.0, 0.0]
    sunStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunStateMsg)
    gravFactory.gravBodies['sun'].planetBodyInMsg.subscribeTo(sunMsg)

    # Create an instance of the facetSRPEffector module to be tested
    newSRP = facetSRPDynamicEffector.FacetSRPDynamicEffector()
    newSRP.ModelTag = "FacetSRP"

    # Connect the Sun's ephemeris message to the module
    newSRP.sunInMsg.subscribeTo(sunMsg)

    # Add the SRP dynamic effector to the spacecraft object
    scObject.addDynamicEffector(newSRP)

    # Define the spacecraft geometry for this test
    try:
        # Define the facet surface areas
        area1 = 1.5*1.5  # [m^2]
        area2 = np.pi*(0.5*7.5)*(0.5*7.5)  # [m^2]
        facetAreas = [area1, area1, area1, area1, area1, area1, area2, area2, area2, area2]

        # Define the facet normals in B frame components
        facetNormal1 = np.array([1.0, 0.0, 0.0])
        facetNormal2 = np.array([0.0, 1.0, 0.0])
        facetNormal3 = np.array([-1.0, 0.0, 0.0])
        facetNormal4 = np.array([0.0, -1.0, 0.0])
        facetNormal5 = np.array([0.0, 0.0, 1.0])
        facetNormal6 = np.array([0.0, 0.0, -1.0])
        facetNormal7 = np.array([0.0, 1.0, 0.0])
        facetNormal8 = np.array([0.0, -1.0, 0.0])
        facetNormal9 = np.array([0.0, 1.0, 0.0])
        facetNormal10 = np.array([0.0, -1.0, 0.0])
        normals_B = [facetNormal1, facetNormal2, facetNormal3, facetNormal4, facetNormal5, facetNormal6, facetNormal7, facetNormal8, facetNormal9, facetNormal10]

        # Define the facet center of pressure locations with respect to point B in B frame components
        facetLoc1 = np.array([0.75, 0.0, 0.0])  # [m]
        facetLoc2 = np.array([0.0, 0.75, 0.0])  # [m]
        facetLoc3 = np.array([-0.75, 0.0, 0.0])  # [m]
        facetLoc4 = np.array([0.0, -0.75, 0.0])  # [m]
        facetLoc5 = np.array([0.0, 0.0, 0.75])  # [m]
        facetLoc6 = np.array([0.0, 0.0, -0.75])  # [m]
        facetLoc7 = np.array([4.5, 0.0, 0.75])  # [m]
        facetLoc8 = np.array([4.5, 0.0, 0.75])  # [m]
        facetLoc9 = np.array([-4.5, 0.0, 0.75])  # [m]
        facetLoc10 = np.array([-4.5, 0.0, 0.75])  # [m]
        locationsPntB_B = [facetLoc1, facetLoc2, facetLoc3, facetLoc4, facetLoc5, facetLoc6, facetLoc7, facetLoc8, facetLoc9, facetLoc10]

        # Define the facet optical coefficients
        specCoeff = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        diffCoeff = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        # Populate the scGeometry structure with the facet information
        for i in range(len(facetAreas)):
            newSRP.addFacet(facetAreas[i], specCoeff[i], diffCoeff[i], normals_B[i], locationsPntB_B[i])
    except:
        testFailCount += 1
        testMessages.append("ERROR: FacetDrag unit test failed while setting facet parameters.")
        return testFailCount, testMessages

    # Set up the spacecraft orbit
    oe = orbitalMotion.ClassicElements()
    r_eq = 6371*1000.0  # [m]
    rN = np.array([r_eq+2000.0, -149598023 * 1000, 0.0])  # [m]
    vN = np.array([0.0, 7.90854, 0.0])  # [m/s]
    sig_BN = np.array([0.0, 0.0, 0.0])

    # Initialize spacecraft states with the initialization variables
    scObject.hub.r_CN_NInit = rN  # [m] r_CN_N
    scObject.hub.v_CN_NInit = vN  # [m] v_CN_N
    scObject.hub.sigma_BNInit = sig_BN

    # Set up data logging
    scPosDataLog = scObject.scStateOutMsg.recorder()
    sunPosDataLog = gravFactory.gravBodies['sun'].planetBodyInMsg.recorder()

    # Add the BSK objects to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, newSRP)
    scSim.AddModelToTask(simTaskName, scPosDataLog)
    scSim.AddModelToTask(simTaskName, sunPosDataLog)

    # Set the simulation time
    simTimeSec = 10.0  # [s]
    simulationTime = macros.sec2nano(simTimeSec)

    # Initialize the simulation
    scSim.InitializeSimulation()

    # Add the data for logging
    scSim.AddVariableForLogging(newSRP.ModelTag + ".forceExternal_B", simulationTimeStep_NS, 0, 2, 'double')
    scSim.AddVariableForLogging(newSRP.ModelTag + ".torqueExternalPntB_B", simulationTimeStep_NS, 0, 2, 'double')

    # Configure the simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Retrieve the logged data
    timespan = scPosDataLog.times()
    r_BN_N = scPosDataLog.r_BN_N
    sigma_BN = scPosDataLog.sigma_BN
    r_SN_N = sunPosDataLog.PositionVector
    SRPDataForce_B = scSim.GetLogVariableData(newSRP.ModelTag + ".forceExternal_B")
    SRPDataTorque_B = scSim.GetLogVariableData(newSRP.ModelTag + ".torqueExternalPntB_B")

    # Store the logged data for plotting
    srpForce_B_plotting = SRPDataForce_B
    srpForce_B_plotting = np.delete(srpForce_B_plotting, 0, axis=1)
    srpTorque_B_plotting = SRPDataTorque_B
    srpTorque_B_plotting = np.delete(srpTorque_B_plotting, 0, axis=1)

    # Plotting results
    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, r_BN_N[:, 0], label=r'$r_{\mathcal{B}/\mathcal{N}} \cdot \hat{n}_1$')
    plt.plot(timespan * 1e-9, r_BN_N[:, 1], label=r'$r_{\mathcal{B}/\mathcal{N}} \cdot \hat{n}_2$')
    plt.plot(timespan * 1e-9, r_BN_N[:, 2], label=r'$r_{\mathcal{B}/\mathcal{N}} \cdot \hat{n}_3$')
    plt.xlabel(r'Time (s)')
    plt.ylabel(r'${}^\mathcal{N} r_{\mathcal{B}/\mathcal{N}}$ (m)')
    plt.legend()

    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, srpForce_B_plotting[:, 0], label=r'$F \cdot \hat{b}_1$')
    plt.plot(timespan * 1e-9, srpForce_B_plotting[:, 1], label=r'$F \cdot \hat{b}_2$')
    plt.plot(timespan * 1e-9, srpForce_B_plotting[:, 2], label=r'$F \cdot \hat{b}_3$')
    plt.xlabel(r'Time (s)')
    plt.ylabel(r'${}^\mathcal{B} F_{SRP, B}$ (N)')
    plt.legend()

    plt.figure()
    plt.clf()
    plt.plot(timespan * 1e-9, srpTorque_B_plotting[:, 0], label=r'$L \cdot \hat{b}_1$')
    plt.plot(timespan * 1e-9, srpTorque_B_plotting[:, 1], label=r'$L \cdot \hat{b}_2$')
    plt.plot(timespan * 1e-9, srpTorque_B_plotting[:, 2], label=r'$L \cdot \hat{b}_3$')
    plt.xlabel(r'Time (s)')
    plt.ylabel(r'${}^\mathcal{B} L_{SRP, B}$ (Nm)')
    plt.legend()

    if show_plots:
        plt.show()
    plt.close("all")

    # Compare the retrieved data to the expected values computed in python
    accuracy = 1e-15
    forceTest1Val = np.zeros([3,])
    forceTest2Val = np.zeros([3,])
    torqueTest1Val = np.zeros([3, ])
    torqueTest2Val = np.zeros([3, ])
    index2 = int(0.5*simTimeSec / simulationTimeStep_Sec)

    for i in range(len(facetAreas)):
        forceTest1Val += checkFacetSRPForce(i, facetAreas[i], specCoeff[i], diffCoeff[i], normals_B[i], sigma_BN[1], r_BN_N[1], r_SN_N[1])
        forceTest2Val += checkFacetSRPForce(i, facetAreas[i], specCoeff[i], diffCoeff[i], normals_B[i], sigma_BN[index2], r_BN_N[index2],
                                       r_SN_N[index2])
        torqueTest1Val += checkFacetSRPTorque(i, facetAreas[i], specCoeff[i], diffCoeff[i], normals_B[i], locationsPntB_B[i], sigma_BN[1],
                                            r_BN_N[1], r_SN_N[1])
        torqueTest2Val += checkFacetSRPTorque(i, facetAreas[i], specCoeff[i], diffCoeff[i], normals_B[i],
                                              locationsPntB_B[i], sigma_BN[index2],
                                              r_BN_N[index2], r_SN_N[index2])
    if not unitTestSupport.isArrayEqual(SRPDataForce_B[1, 1:4], forceTest1Val, 3, accuracy):
        testFailCount += 1
        testMessages.append("FAILED:  FacetSRPEffector failed unit test at t=" + str(
            SRPDataForce_B[1, 0] * macros.NANO2SEC) + "sec with a value difference of " + str(
            SRPDataForce_B[1, 1:] - forceTest1Val))
        print(SRPDataForce_B[1, 1:])
        print(forceTest1Val)

    if not unitTestSupport.isArrayEqual(SRPDataForce_B[index2, 1:4], forceTest2Val, 3, accuracy):
        testFailCount += 1
        testMessages.append("FAILED:  FacetSRPEffector failed unit test at t=" + str(
            SRPDataForce_B[index2, 0] * macros.NANO2SEC) + "sec with a value difference of " + str(
            SRPDataForce_B[index2, 1:] - forceTest2Val))
        print(SRPDataForce_B[index2, 1:])
        print(forceTest2Val)

    if not unitTestSupport.isArrayEqual(SRPDataTorque_B[1, 1:4], torqueTest1Val, 3, accuracy):
        testFailCount += 1
        testMessages.append("FAILED:  FacetSRPEffector failed unit test at t=" + str(
            SRPDataTorque_B[1, 0] * macros.NANO2SEC) + "sec with a value difference of " + str(
            SRPDataTorque_B[1, 1:] - torqueTest1Val))
        print(SRPDataTorque_B[1, 1:])
        print(torqueTest1Val)

    if not unitTestSupport.isArrayEqual(SRPDataTorque_B[index2, 1:4], torqueTest2Val, 3, accuracy):
        testFailCount += 1
        testMessages.append("FAILED:  FacetSRPEffector failed unit test at t=" + str(
            SRPDataTorque_B[index2, 0] * macros.NANO2SEC) + "sec with a value difference of " + str(
            SRPDataTorque_B[index2, 1:] - torqueTest2Val))
        print(SRPDataTorque_B[index2, 1:])
        print(torqueTest2Val)

    if testFailCount:
        print(testMessages)
    else:
        print("PASSED")

    return testFailCount, testMessages

if __name__=="__main__":
    # TestfacetSRPDynamicEffector
    TestfacetSRPDynamicEffector(
            True  # show plots
           )
