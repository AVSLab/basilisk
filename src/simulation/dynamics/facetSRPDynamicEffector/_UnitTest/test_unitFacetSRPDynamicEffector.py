
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
#   Unit Test Script
#   Module Name:        facetSRPDynamicEffector
#   Author:             Leah Kiner
#   Creation Date:      Dec 18 2022
#   Last Updated:       Dec 13 2023
#

import os

import matplotlib.pyplot as plt
import numpy as np
import pytest

from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import orbitalMotion
from Basilisk.simulation import facetSRPDynamicEffector
from Basilisk.simulation import spacecraft
from Basilisk.architecture import messaging

# Vary the articulated facet initial angles
@pytest.mark.parametrize("facetRotAngle1", [macros.D2R * -10.4, macros.D2R * 45.2, macros.D2R * 90.0, macros.D2R * 180.0])
@pytest.mark.parametrize("facetRotAngle2", [macros.D2R * -28.0, macros.D2R * 45.2, macros.D2R * -90.0, macros.D2R * 180.0])
def test_facetSRPTestFunction(show_plots, facetRotAngle1, facetRotAngle2):
    r"""
    **Validation Test Description**

    The unit test for this module ensures that the calculated Solar Radiation Pressure (SRP) force and torque acting
    on the spacecraft about the body-fixed point B is properly computed for either a static spacecraft or a spacecraft
    with any number of articulating facets. The spacecraft geometry defined in this test consists of a cubic hub and
    two circular solar arrays. Six static square facets represent the cubic hub and four articulated circular facets
    describe the articulating solar arrays. To validate the module functionality, the final SRP force simulation value
    is checked with the true value computed in python.

    **Test Parameters**

    Args:

        show_plots (bool): (True) Show plots, (False) Do not show plots
        facetRotAngle1 (double): [rad] Articulation angle for facets 7 and 8 (solar panel 1)
        facetRotAngle2 (double): [rad] Articulation angle for facets 9 and 10 (solar panel 2)
    """
    
    [testResults, testMessage] = facetSRPTestFunction(show_plots, facetRotAngle1, facetRotAngle2)

    assert testResults < 1, testMessage

def facetSRPTestFunction(show_plots, facetRotAngle1, facetRotAngle2):
    testFailCount = 0                                           # Zero the unit test result counter
    testMessages = []                                           # Create an empty array to store the test log messages

    # Set up the simulation, task, and process
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitProcessName = "simProcess"
    dynProcess = unitTestSim.CreateNewProcess(unitProcessName)
    simulationTimeStep = macros.sec2nano(0.1)
    unitTaskName = "simTask"
    dynProcess.addTask(unitTestSim.CreateNewTask(unitTaskName, simulationTimeStep))

    # Create the Sun
    gravFactory = simIncludeGravBody.gravBodyFactory()
    sun = gravFactory.createSun()
    sun.isCentralBody = True
    mu = sun.mu

    # Set custom Sun Spice data
    sunStateMsg = messaging.SpicePlanetStateMsgPayload()
    sunStateMsg.PositionVector = [0.0, 0.0, 0.0]
    sunStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunStateMsg)
    gravFactory.gravBodies['sun'].planetBodyInMsg.subscribeTo(sunMsg)

    # Create the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scObject"

    oe = orbitalMotion.ClassicElements()
    oe.a = 149597870700.0  # [m]
    oe.e = 0.5
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements

    scObject.hub.r_CN_NInit = rN  # [m] Spacecraft inertial position
    scObject.hub.v_CN_NInit = vN  # [m] Spacecraft inertial velocity
    scObject.hub.sigma_BNInit = np.array([0.0, 0.0, 0.0])
    scObject.hub.omega_BN_BInit = np.array([0.0, 0.0, 0.0])
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Create the articulated facet angle messages
    facetRotAngle1MessageData = messaging.HingedRigidBodyMsgPayload()
    facetRotAngle1MessageData.theta = facetRotAngle1
    facetRotAngle1MessageData.thetaDot = 0.0
    facetRotAngle1Message = messaging.HingedRigidBodyMsg().write(facetRotAngle1MessageData)
    
    facetRotAngle2MessageData = messaging.HingedRigidBodyMsgPayload()
    facetRotAngle2MessageData.theta = facetRotAngle2
    facetRotAngle2MessageData.thetaDot = 0.0
    facetRotAngle2Message = messaging.HingedRigidBodyMsg().write(facetRotAngle2MessageData)

    # Create an instance of the facetSRPDynamicEffector module to be tested
    srpEffector = facetSRPDynamicEffector.FacetSRPDynamicEffector()
    srpEffector.ModelTag = "srpEffector"
    numFacets = 10  # Total number of spacecraft facets
    numArticulatedFacets = 4  # Number of articulated facets
    srpEffector.numFacets = numFacets
    srpEffector.numArticulatedFacets = numArticulatedFacets
    srpEffector.sunInMsg.subscribeTo(sunMsg)
    srpEffector.addArticulatedFacet(facetRotAngle1Message)
    srpEffector.addArticulatedFacet(facetRotAngle1Message)
    srpEffector.addArticulatedFacet(facetRotAngle2Message)
    srpEffector.addArticulatedFacet(facetRotAngle2Message)
    scObject.addDynamicEffector(srpEffector)
    unitTestSim.AddModelToTask(unitTaskName, srpEffector)

    # Set up the srpEffector spacecraft geometry data structure
    try:
        # Define facet areas
        area1 = 1.5 * 1.5
        area2 = np.pi * (0.5 * 7.5) * (0.5 * 7.5)
        facetAreas = [area1, area1, area1, area1, area1, area1, area2, area2, area2, area2]

        # Define the facet normal vectors in B frame components
        facetNormals_B = [np.array([1.0, 0.0, 0.0]),
                          np.array([0.0, 1.0, 0.0]),
                          np.array([-1.0, 0.0, 0.0]),
                          np.array([0.0, -1.0, 0.0]),
                          np.array([0.0, 0.0, 1.0]),
                          np.array([0.0, 0.0, -1.0]),
                          np.array([0.0, 1.0, 0.0]),
                          np.array([0.0, -1.0, 0.0]),
                          np.array([0.0, 1.0, 0.0]),
                          np.array([0.0, -1.0, 0.0])]

        # Define facet center of pressure locations relative to point B
        locationsPntB_B = [np.array([0.75, 0.0, 0.0]),
                       np.array([0.0, 0.75, 0.0]),
                       np.array([-0.75, 0.0, 0.0]),
                       np.array([0.0, -0.75, 0.0]),
                       np.array([0.0, 0.0, 0.75]),
                       np.array([0.0, 0.0, -0.75]),
                       np.array([4.5, 0.0, 0.75]),
                       np.array([4.5, 0.0, 0.75]),
                       np.array([-4.5, 0.0, 0.75]),
                       np.array([-4.5, 0.0, 0.75])]

        # Define facet articulation axes in B frame components
        rotAxes_B = [np.array([0.0, 0.0, 0.0]),
                     np.array([0.0, 0.0, 0.0]),
                     np.array([0.0, 0.0, 0.0]),
                     np.array([0.0, 0.0, 0.0]),
                     np.array([0.0, 0.0, 0.0]),
                     np.array([0.0, 0.0, 0.0]),
                     np.array([1.0, 0.0, 0.0]),
                     np.array([1.0, 0.0, 0.0]),
                     np.array([-1.0, 0.0, 0.0]),
                     np.array([-1.0, 0.0, 0.0])]

        # Define facet optical coefficients
        specCoeff = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        diffCoeff = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        # Populate the srpEffector spacecraft geometry structure with the facet information
        for i in range(numFacets):
            srpEffector.addFacet(facetAreas[i], specCoeff[i], diffCoeff[i], facetNormals_B[i], locationsPntB_B[i], rotAxes_B[i])
    except:
        testFailCount += 1
        testMessages.append("ERROR: FacetSRP unit test failed while setting facet parameters.")
        return testFailCount, testMessages

    # Set up data logging
    scPosDataLog = scObject.scStateOutMsg.recorder()
    sunPosDataLog = gravFactory.gravBodies['sun'].planetBodyInMsg.recorder()
    srpDataLog = srpEffector.logger(["forceExternal_B", "torqueExternalPntB_B"], simulationTimeStep)
    unitTestSim.AddModelToTask(unitTaskName, scPosDataLog)
    unitTestSim.AddModelToTask(unitTaskName, sunPosDataLog)
    unitTestSim.AddModelToTask(unitTaskName, srpDataLog)

    # Execute the simulation
    unitTestSim.InitializeSimulation()
    simulationTime = macros.sec2nano(10.0)
    unitTestSim.ConfigureStopTime(simulationTime)
    unitTestSim.ExecuteSimulation()

    # Retrieve the logged data
    timespan = scPosDataLog.times() * macros.NANO2SEC  # [s]
    r_BN_N = scPosDataLog.r_BN_N  # [m]
    sigma_BN = scPosDataLog.sigma_BN
    r_SN_N = sunPosDataLog.PositionVector  # [m]
    srpForce_B = srpDataLog.forceExternal_B  # [N]
    srpTorque_B = srpDataLog.torqueExternalPntB_B  # [Nm]

    # Plot spacecraft inertial position
    plt.figure()
    plt.clf()
    plt.plot(timespan, r_BN_N[:, 0], label=r'$r_{\mathcal{B}/\mathcal{N}} \cdot \hat{n}_1$')
    plt.plot(timespan, r_BN_N[:, 1], label=r'$r_{\mathcal{B}/\mathcal{N}} \cdot \hat{n}_2$')
    plt.plot(timespan, r_BN_N[:, 2], label=r'$r_{\mathcal{B}/\mathcal{N}} \cdot \hat{n}_3$')
    plt.xlabel(r'Time (s)')
    plt.ylabel(r'${}^N r_{\mathcal{B}/\mathcal{N}}$ (m)')
    plt.legend()

    # Plot SRP force
    plt.figure()
    plt.clf()
    plt.plot(timespan, srpForce_B[:, 0], label=r'$F_{SRP} \cdot \hat{b}_1$')
    plt.plot(timespan, srpForce_B[:, 1], label=r'$F_{SRP} \cdot \hat{b}_2$')
    plt.plot(timespan, srpForce_B[:, 2], label=r'$F_{SRP} \cdot \hat{b}_3$')
    plt.xlabel('Time (s)')
    plt.ylabel(r'${}^B F_{SRP}$ (N)')
    plt.legend()

    # Plot SRP torque
    plt.figure()
    plt.clf()
    plt.plot(timespan, srpTorque_B[:, 0], label=r'$L_{SRP} \cdot \hat{b}_1$')
    plt.plot(timespan, srpTorque_B[:, 1], label=r'$L_{SRP} \cdot \hat{b}_2$')
    plt.plot(timespan, srpTorque_B[:, 2], label=r'$L_{SRP} \cdot \hat{b}_3$')
    plt.xlabel('Time (s)')
    plt.ylabel(r'${}^B L_{SRP}$ (Nm)')
    plt.legend()

    if show_plots:
        plt.show()
    plt.close("all")

    # Validate the results by comparing the last srp force and torque simulation values with the predicted values
    accuracy = 1e-12
    test_val = np.zeros([3,])
    for i in range(len(facetAreas)):
        test_val += checkFacetSRPForce(i, facetRotAngle1, facetRotAngle2, facetAreas[i], specCoeff[i], diffCoeff[i], facetNormals_B[i], rotAxes_B[i], sigma_BN[-1], r_BN_N[-1], r_SN_N[-1])

    if not unitTestSupport.isArrayEqual(srpForce_B[-1, :], test_val, 3, accuracy):
        testFailCount += 1
        testMessages.append("FAILED:  FacetSRPEffector failed unit test at t=" + str(
            timespan[-1]) + "sec with a value difference of " + str(
            srpForce_B[-1, :] - test_val))

    if testFailCount:
        print(testMessages)
    else:
        print("PASSED")

    return testFailCount, testMessages

def checkFacetSRPForce(index, facetRotAngle1, facetRotAngle2, area, specCoeff, diffCoeff, facetNormal, facetRotAxis, sigma_BN, scPos, sunPos):
    # Define required constants
    speedLight = 299792458.0  # [m/s] Speed of light
    AstU = 149597870700.0  # [m] Astronomical unit
    solarRadFlux = 1368.0  # [W/m^2] Solar radiation flux at 1 AU

    # Compute dcm_BN
    dcm_BN = rbk.MRP2C(sigma_BN)

    # Compute Sun direction relative to point B in B frame components
    r_BN_B = np.matmul(dcm_BN, scPos)  # [m]
    r_SN_B = np.matmul(dcm_BN, sunPos)  # [m]
    r_SB_B = r_SN_B - r_BN_B  # [m]

    # Determine unit direction vector pointing from sc to the Sun
    sHat = r_SB_B / np.linalg.norm(r_SB_B)

    # Rotate the articulated facet normal vectors
    if (index == 6 or index == 7):
        prv_BB0 = facetRotAngle1 * facetRotAxis
        dcm_BB0 = rbk.PRV2C(prv_BB0)
        facetNormal = np.matmul(dcm_BB0, facetNormal)
    if (index == 8 or index == 9):
        prv_BB0 = facetRotAngle2 * facetRotAxis
        dcm_BB0 = rbk.PRV2C(prv_BB0)
        facetNormal = np.matmul(dcm_BB0, facetNormal)

    # Determine the facet projected area
    cosTheta = np.dot(sHat, facetNormal)
    projArea = area * cosTheta

    # Determine the SRP pressure at the current sc location
    numAU = AstU / np.linalg.norm(r_SB_B)
    SRPPressure = (solarRadFlux / speedLight) * numAU * numAU

    # Compute the SRP force acting on the facet
    if projArea > 0:
        srp_force = -SRPPressure * projArea * ((1-specCoeff) * sHat + 2 * ( (diffCoeff / 3) + specCoeff * cosTheta) * facetNormal)
    else:
        srp_force = np.zeros([3,])

    return srp_force

if __name__=="__main__":
    facetSRPTestFunction(
        True,  # show plots
        macros.D2R * -10.0,  # facetRotAngle1
        macros.D2R * 45.0,  # facetRotAngle2
    )
