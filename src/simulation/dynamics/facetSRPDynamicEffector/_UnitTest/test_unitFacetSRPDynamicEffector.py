
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
#   Last Updated:       Dec 2 2024
#

import matplotlib.pyplot as plt
import numpy as np
import os
import pytest
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import orbitalMotion
from Basilisk.simulation import facetSRPDynamicEffector
from Basilisk.simulation import spacecraft
from Basilisk.architecture import messaging

# Vary the articulated facet initial angles
@pytest.mark.parametrize("facetRotAngle1", [macros.D2R * -10.4, macros.D2R * 45.2, macros.D2R * 90.0, macros.D2R * 180.0])
@pytest.mark.parametrize("facetRotAngle2", [macros.D2R * -28.0, macros.D2R * 45.2, macros.D2R * -90.0, macros.D2R * 180.0])
def test_facetSRPDynamicEffector(show_plots, facetRotAngle1, facetRotAngle2):
    r"""
    **Verification Test Description**

    The unit test for this module ensures that the calculated Solar Radiation Pressure (SRP) force and torque acting
    on the spacecraft about the body-fixed point B is properly computed for either a static spacecraft or a spacecraft
    with any number of articulating facets. The spacecraft geometry defined in this test consists of a cubic hub and
    two circular solar arrays. Six static square facets represent the cubic hub and four articulated circular facets
    describe the two articulating solar arrays. To verify the module functionality, the final SRP force and torque
    simulation values are checked with the truth values computed in python.

    **Test Parameters**

    Args:
        show_plots (bool): (True) Show plots, (False) Do not show plots
        facetRotAngle1 (double): [rad] Articulation angle for facets 7 and 8 (solar panel 1)
        facetRotAngle2 (double): [rad] Articulation angle for facets 9 and 10 (solar panel 2)
    """

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

    # Create the spacecraft object and set the spacecraft orbit
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
    oe = orbitalMotion.rv2elem(mu, rN, vN)
    scObject.hub.r_CN_NInit = rN  # [m] Spacecraft inertial position
    scObject.hub.v_CN_NInit = vN  # [m] Spacecraft inertial velocity
    scObject.hub.sigma_BNInit = np.array([0.0, 0.0, 0.0])
    scObject.hub.omega_BN_BInit = np.array([0.0, 0.0, 0.0])
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Create the articulated facet angle messages
    facetRotAngle1MessageData = messaging.HingedRigidBodyMsgPayload()
    facetRotAngle1MessageData.theta = facetRotAngle1  # [rad]
    facetRotAngle1MessageData.thetaDot = 0.0  # [rad]
    facetRotAngle1Message = messaging.HingedRigidBodyMsg().write(facetRotAngle1MessageData)

    facetRotAngle2MessageData = messaging.HingedRigidBodyMsgPayload()
    facetRotAngle2MessageData.theta = facetRotAngle2  # [rad]
    facetRotAngle2MessageData.thetaDot = 0.0  # [rad]
    facetRotAngle2Message = messaging.HingedRigidBodyMsg().write(facetRotAngle2MessageData)

    # Create an instance of the facetSRPDynamicEffector module to be tested
    srpEffector = facetSRPDynamicEffector.FacetSRPDynamicEffector()
    srpEffector.ModelTag = "srpEffector"
    numFacets = 10  # Total number of spacecraft facets
    numArticulatedFacets = 4  # Number of articulated facets
    srpEffector.setNumFacets(numFacets)
    srpEffector.setNumArticulatedFacets(numArticulatedFacets)
    srpEffector.sunInMsg.subscribeTo(sunMsg)
    srpEffector.addArticulatedFacet(facetRotAngle1Message)
    srpEffector.addArticulatedFacet(facetRotAngle1Message)
    srpEffector.addArticulatedFacet(facetRotAngle2Message)
    srpEffector.addArticulatedFacet(facetRotAngle2Message)
    scObject.addDynamicEffector(srpEffector)
    unitTestSim.AddModelToTask(unitTaskName, srpEffector)

    # Set up the srpEffector spacecraft geometry data structure
    # Define facet areas
    area1 = 1.5 * 1.5
    area2 = np.pi * (0.5 * 7.5) * (0.5 * 7.5)
    facetAreaList = [area1, area1, area1, area1, area1, area1, area2, area2, area2, area2]

    # Define the initial facet attitudes relative to B frame
    prv_F01B = (macros.D2R * -90.0) * np.array([0.0, 0.0, 1.0])
    prv_F02B = (macros.D2R * 0.0) * np.array([0.0, 0.0, 1.0])
    prv_F03B = (macros.D2R * 90.0) * np.array([0.0, 0.0, 1.0])
    prv_F04B = (macros.D2R * 180.0) * np.array([0.0, 0.0, 1.0])
    prv_F05B = (macros.D2R * 90.0) * np.array([1.0, 0.0, 0.0])
    prv_F06B = (macros.D2R * -90.0) * np.array([1.0, 0.0, 0.0])
    prv_F07B = (macros.D2R * 0.0) * np.array([0.0, 0.0, 1.0])
    prv_F08B = (macros.D2R * 180.0) * np.array([0.0, 0.0, 1.0])
    prv_F09B = (macros.D2R * 0.0) * np.array([0.0, 0.0, 1.0])
    prv_F010B = (macros.D2R * 180.0) * np.array([0.0, 0.0, 1.0])
    facetDcm_F0BList = [rbk.PRV2C(prv_F01B),
                        rbk.PRV2C(prv_F02B),
                        rbk.PRV2C(prv_F03B),
                        rbk.PRV2C(prv_F04B),
                        rbk.PRV2C(prv_F05B),
                        rbk.PRV2C(prv_F06B),
                        rbk.PRV2C(prv_F07B),
                        rbk.PRV2C(prv_F08B),
                        rbk.PRV2C(prv_F09B),
                        rbk.PRV2C(prv_F010B)]

    # Define the facet normal vectors in F frame components
    facetNHat_FList = [np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0])]

    # Define facet articulation axes in F frame components
    facetRotHat_FList = [np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([1.0, 0.0, 0.0]),
                         np.array([-1.0, 0.0, 0.0]),
                         np.array([-1.0, 0.0, 0.0]),
                         np.array([1.0, 0.0, 0.0])]

    # Define facet center of pressure locations relative to point B
    facetR_CopB_BList = [np.array([0.75, 0.0, 0.0]),
                         np.array([0.0, 0.75, 0.0]),
                         np.array([-0.75, 0.0, 0.0]),
                         np.array([0.0, -0.75, 0.0]),
                         np.array([0.0, 0.0, 0.75]),
                         np.array([0.0, 0.0, -0.75]),
                         np.array([4.5, 0.0, 0.75]),
                         np.array([4.5, 0.0, 0.75]),
                         np.array([-4.5, 0.0, 0.75]),
                         np.array([-4.5, 0.0, 0.75])]

    # Define facet optical coefficients
    facetDiffuseCoeffList = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    facetSpecularCoeffList = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    # Populate the srpEffector spacecraft geometry structure with the facet information
    for i in range(numFacets):
        srpEffector.addFacet(facetAreaList[i],
                             facetDcm_F0BList[i],
                             facetNHat_FList[i],
                             facetRotHat_FList[i],
                             facetR_CopB_BList[i],
                             facetDiffuseCoeffList[i],
                             facetSpecularCoeffList[i])

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
    srpForce_BSim = srpDataLog.forceExternal_B  # [N]
    srpTorque_BSim = srpDataLog.torqueExternalPntB_B  # [Nm]

    # Plot the spacecraft inertial position
    plt.figure()
    plt.clf()
    plt.plot(timespan, r_BN_N[:, 0], label=r'$r_{\mathcal{B}/\mathcal{N}} \cdot \hat{n}_1$')
    plt.plot(timespan, r_BN_N[:, 1], label=r'$r_{\mathcal{B}/\mathcal{N}} \cdot \hat{n}_2$')
    plt.plot(timespan, r_BN_N[:, 2], label=r'$r_{\mathcal{B}/\mathcal{N}} \cdot \hat{n}_3$')
    plt.title("Spacecraft Inertial Position Components")
    plt.xlabel(r'Time (s)')
    plt.ylabel(r'${}^N r_{\mathcal{B}/\mathcal{N}}$ (m)')
    plt.legend()

    # Plot SRP force
    plt.figure()
    plt.clf()
    plt.plot(timespan, srpForce_BSim[:, 0], label=r'$F_{SRP} \cdot \hat{b}_1$')
    plt.plot(timespan, srpForce_BSim[:, 1], label=r'$F_{SRP} \cdot \hat{b}_2$')
    plt.plot(timespan, srpForce_BSim[:, 2], label=r'$F_{SRP} \cdot \hat{b}_3$')
    plt.title("SRP Force Components")
    plt.xlabel('Time (s)')
    plt.ylabel(r'${}^B F_{SRP}$ (N)')
    plt.legend()

    # Plot SRP torque
    plt.figure()
    plt.clf()
    plt.plot(timespan, srpTorque_BSim[:, 0], label=r'$L_{SRP} \cdot \hat{b}_1$')
    plt.plot(timespan, srpTorque_BSim[:, 1], label=r'$L_{SRP} \cdot \hat{b}_2$')
    plt.plot(timespan, srpTorque_BSim[:, 2], label=r'$L_{SRP} \cdot \hat{b}_3$')
    plt.title("SRP Torque Components")
    plt.xlabel('Time (s)')
    plt.ylabel(r'${}^B L_{SRP}$ (Nm)')
    plt.legend()

    if show_plots:
        plt.show()
    plt.close("all")

    # Verify the results by comparing the last srp force and torque simulation values with the calculated truth values
    srpForce_BTruth = np.zeros([3,])
    srpTorque_BTruth = np.zeros([3,])
    for i in range(len(facetAreaList)):
        srpForce_BFacet, srpTorque_BFacet = computeFacetSRPForceTorque(i,
                                                                       facetRotAngle1,
                                                                       facetRotAngle2,
                                                                       facetAreaList[i],
                                                                       facetDcm_F0BList[i],
                                                                       facetNHat_FList[i],
                                                                       facetRotHat_FList[i],
                                                                       facetR_CopB_BList[i],
                                                                       facetDiffuseCoeffList[i],
                                                                       facetSpecularCoeffList[i],
                                                                       sigma_BN[-1],
                                                                       r_BN_N[-1],
                                                                       r_SN_N[-1])
        srpForce_BTruth += srpForce_BFacet
        srpTorque_BTruth += srpTorque_BFacet

    for idx in range(3):
        np.testing.assert_allclose(srpForce_BSim[-1, idx],
                                   srpForce_BTruth[idx],
                                   atol=1e-12,
                                   verbose=True)
        np.testing.assert_allclose(srpTorque_BSim[-1, idx],
                                   srpTorque_BTruth[idx],
                                   atol=1e-12,
                                   verbose=True)

def computeFacetSRPForceTorque(index,
                               facetRotAngle1,
                               facetRotAngle2,
                               facetArea,
                               facetDcm_F0B,
                               facetNHat_F,
                               facetRotHat_F,
                               facetR_CopB_B,
                               facetDiffuseCoeff,
                               facetSpecularCoeff,
                               sigma_BN,
                               r_BN_N,
                               r_SN_N):
    # Define required constants
    speedLight = 299792458.0  # [m/s] Speed of light
    AstU = 149597870700.0  # [m] Astronomical unit
    solarRadFlux = 1368.0  # [W/m^2] Solar radiation flux at 1 AU

    # Compute Sun direction relative to point B in B frame components
    dcm_BN = rbk.MRP2C(sigma_BN)
    r_BN_B = np.matmul(dcm_BN, r_BN_N)  # [m]
    r_SN_B = np.matmul(dcm_BN, r_SN_N)  # [m]
    r_SB_B = r_SN_B - r_BN_B  # [m]

    # Determine unit direction vector pointing from sc to the Sun
    sHat = r_SB_B / np.linalg.norm(r_SB_B)

    # Rotate the articulated facet normal vector
    dcm_FF0 = np.eye(3)
    if (index == 6 or index == 7):
        prv_FF0 = facetRotAngle1 * facetRotHat_F
        dcm_FF0 = rbk.PRV2C(prv_FF0)
    if (index == 8 or index == 9):
        prv_FF0 = facetRotAngle2 * facetRotHat_F
        dcm_FF0 = rbk.PRV2C(prv_FF0)

    # Compute the facet normal vector in the B frame
    dcm_FB = np.matmul(dcm_FF0, facetDcm_F0B)
    facetNHat_B = np.matmul(dcm_FB.transpose(), facetNHat_F)

    # Determine the facet projected area
    cosTheta = np.dot(sHat, facetNHat_B)
    projArea = facetArea * cosTheta

    # Determine the SRP pressure at the current sc location
    numAU = AstU / np.linalg.norm(r_SB_B)
    SRPPressure = (solarRadFlux / speedLight) * numAU * numAU

    # Compute the SRP force acting on the facet
    if projArea > 0:
        srpForce_BTruth = -SRPPressure * projArea * ((1-facetSpecularCoeff) * sHat + 2 * ( (facetDiffuseCoeff / 3) + facetSpecularCoeff * cosTheta) * facetNHat_B)
        srpTorque_BTruth = np.cross(facetR_CopB_B, srpForce_BTruth)
    else:
        srpForce_BTruth = np.zeros([3,])
        srpTorque_BTruth = np.zeros([3,])

    return srpForce_BTruth, srpTorque_BTruth

if __name__=="__main__":
    test_facetSRPDynamicEffector(
        True,  # show plots
        macros.D2R * -10.0,  # [rad] facetRotAngle1
        macros.D2R * 45.0,  # [rad] facetRotAngle2
    )
