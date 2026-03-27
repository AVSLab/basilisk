
# ISC License
#
# Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#
#
# Purpose:  Test the facetDrag module.
# Author:   Andrew Harris
# Creation Date:  May 16 2019
#


import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk

# import simulation related support
from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.simulation import facetDragDynamicEffector
from Basilisk.simulation import simpleNav
from Basilisk.simulation import zeroWindModel
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import simIncludeGravBody


test_drag = [([1.0, 1.0], np.array([2.0, 2.0]), [np.array([1, 0, 0]), np.array([0, 1, 0])], [np.array([0.1, 0, 0]), np.array([0, 0.1, 0])]),
             ([1.0, 1.0], np.array([2.0, 2.0]), [np.array([1, 0, 0]), np.array([0, 1, 0])], [np.array([0.3, 0, 0]), np.array([0, 0.3, 0])]),
             ([1.0, 2.0], np.array([2.0, 4.0]), [np.array([1, 0, 0]), np.array([0, 1, 0])], [np.array([0.1, 0, 0]), np.array([0, 0.1, 0])]),
             ([1.0, 1.0], np.array([2.0, 2.0]), [np.array([1, 0, 0]), np.array([0, 1, 0])], [np.array([0.1, 0, 0]), np.array([0, 0, 0.1])]),
             ([1.0, 1.0], np.array([2.0, 2.0]), [np.array([1, 0, 0]), np.array([0, 0, 1])], [np.array([0.1, 0, 0]), np.array([0, 0, 0.1])]),
             ([1.0, 1.0], np.array([2.0, 2.0]), [np.array([0, 0, -1]), np.array([0, -1, 0])], [np.array([0, 0, 0.1]), np.array([0, 0.1, 0])]),
]

@pytest.mark.parametrize("scAreas, scCoeff, B_normals, B_locations", test_drag)
def test_DragCalculation(scAreas, scCoeff, B_normals, B_locations):

    ##   Simulation initialization
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()

    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(5.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    ##   Initialize new atmosphere and drag model, add them to task
    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    newAtmo.ModelTag = "ExpAtmo"
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsg)

    newDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    newDrag.ModelTag = "FacetDrag"
    newDrag.atmoDensInMsg.subscribeTo(newAtmo.envOutMsgs[0])

    scObject.addDynamicEffector(newDrag)

    try:
        for i in range(0,len(scAreas)):
            newDrag.addFacet(scAreas[i], scCoeff[i], B_normals[i], B_locations[i])
    except:
        pytest.fail("ERROR: FacetDrag unit test failed while setting facet parameters.")

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()

    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    #   setup orbit and simulation time
    oe = orbitalMotion.ClassicElements()

    r_eq = 6371*1000.0  # [m]
    refBaseDens = 1.217  # [kg/m^3]
    refScaleHeight = 8500.0  # [m]

    #   Set base density, equitorial radius, scale height in Atmosphere
    newAtmo.baseDensity = refBaseDens
    newAtmo.scaleHeight = refScaleHeight
    newAtmo.planetRadius = r_eq

    rN = np.array([r_eq+200.0e3,0,0])  # [m]
    vN = np.array([0,7.788e3,0])  # [m/s]
    sig_BN = np.array([0,0,0])  # [-] MRP
    #   initialize Spacecraft States with the initialization variables
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N
    scObject.hub.sigma_BNInit = sig_BN

    simulationTime = macros.sec2nano(5.)
    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 10

    # add BSK objects to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, newAtmo)
    scSim.AddModelToTask(simTaskName, newDrag)

    # setup logging
    dataLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, dataLog)
    atmoLog = newAtmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask(simTaskName, atmoLog)
    newDragLog = newDrag.logger(["forceExternal_B", "torqueExternalPntB_B"])
    scSim.AddModelToTask(simTaskName, newDragLog)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   Retrieve logged data
    dragDataForce_B = unitTestSupport.addTimeColumn(newDragLog.times(), newDragLog.forceExternal_B)
    dragTorqueData = unitTestSupport.addTimeColumn(newDragLog.times(), newDragLog.torqueExternalPntB_B)
    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N
    attData = dataLog.sigma_BN
    densData = atmoLog.neutralDensity
    np.set_printoptions(precision=16)

    def checkFacetDragForce(dens, area, coeff, facet_dir, sigma_BN, inertial_vel):
        dcm = rbk.MRP2C(sigma_BN)
        vMag = np.linalg.norm(inertial_vel)
        v_hat_B = dcm.dot(inertial_vel) / vMag
        projArea = area * (facet_dir.dot(v_hat_B))
        if projArea > 0:
            drag_force = -0.5 * dens * projArea * coeff * vMag**2.0 * v_hat_B
        else:
            drag_force = np.zeros([3,])
        return drag_force


    #   Compare to expected values

    test_val_force = np.zeros([3,])
    test_val_torque = np.zeros([3,])
    for i in range(len(scAreas)):
        val_force_i = checkFacetDragForce(densData[i], scAreas[i], scCoeff[i], B_normals[i], attData[1], velData[1])
        test_val_force += val_force_i
        test_val_torque += np.cross(B_locations[i], val_force_i)

    assert len(densData) > 0, "FAILED:  ExpAtmo failed to pull any logged data"
    np.testing.assert_allclose(dragDataForce_B[1,1:4], test_val_force, atol = 1e-06)
    np.testing.assert_allclose(dragTorqueData[1,1:4], test_val_torque, atol = 1e-06)


test_shadow = [([1.0, 1.0], np.array([2.0, 2.0]), [np.array([0, 0, -1]), np.array([0, -1, 0])], [np.array([0, 0, 0.1]), np.array([0, 0.1, 0])]),
               ([1.0, 1.0], np.array([2.0, 4.0]), [np.array([0, 0, -1]), np.array([0, -1, 0])], [np.array([0, 0, 0.1]), np.array([0, 0.1, 0])]),
               ([1.0, 1.0], np.array([2.0, 2.0]), [np.array([0, 0, -1]), np.array([0, -1, 0])], [np.array([0, 0, 0.4]), np.array([0, 0.4, 0])]),
]

@pytest.mark.parametrize("scAreas, scCoeff, B_normals, B_locations", test_shadow)
def test_ShadowCalculation(scAreas, scCoeff, B_normals, B_locations):

    ##   Simulation initialization
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()

    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    simpleNavObj = simpleNav.SimpleNav()
    simpleNavObj.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    ##   Initialize new atmosphere and drag model, add them to task
    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    newAtmo.ModelTag = "ExpAtmo"
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsg)

    newDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    newDrag.ModelTag = "FacetDrag"
    newDrag.atmoDensInMsg.subscribeTo(newAtmo.envOutMsgs[0])

    scObject.addDynamicEffector(newDrag)

    try:
        for ind in range(0,len(scAreas)):
            newDrag.addFacet(scAreas[ind], scCoeff[ind], B_normals[ind], B_locations[ind])
    except:
        pytest.fail("ERROR: FacetDrag unit test failed while setting facet parameters.")

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()

    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    #   setup orbit and simulation time
    oe = orbitalMotion.ClassicElements()

    r_eq = 6371*1000.0  # [m]
    refBaseDens = 1.217  # [kg/m^3]
    refScaleHeight = 8500.0  # [m]

    #   Set base density, equitorial radius, scale height in Atmosphere
    newAtmo.baseDensity = refBaseDens
    newAtmo.scaleHeight = refScaleHeight
    newAtmo.planetRadius = r_eq

    rN = np.array([r_eq+200.0e3,0,0])  # [m]
    vN = np.array([0,7.788e3,0])  # [m/s]
    sig_BN = np.array([0,0,0])  # [-] MRP

    #   initialize Spacecraft States with the initialization variables
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N
    scObject.hub.sigma_BNInit = sig_BN

    simulationTime = macros.sec2nano(10.)
    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 10

    # add BSK objects to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, newAtmo)
    scSim.AddModelToTask(simTaskName, newDrag)

    # setup logging
    dataLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, dataLog)
    atmoLog = newAtmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask(simTaskName, atmoLog)
    newDragLog = newDrag.logger(["forceExternal_B", "torqueExternalPntB_B"])
    scSim.AddModelToTask(simTaskName, newDragLog)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   Retrieve logged data
    dragDataForce_B = unitTestSupport.addTimeColumn(newDragLog.times(), newDragLog.forceExternal_B)
    dragTorqueData = unitTestSupport.addTimeColumn(newDragLog.times(), newDragLog.torqueExternalPntB_B)
    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N
    attData = dataLog.sigma_BN
    densData = atmoLog.neutralDensity
    np.set_printoptions(precision=16)

    assert len(densData) > 0, "FAILED:  ExpAtmo failed to pull any logged data"
    for ind in range(1,len(densData)):
        np.testing.assert_allclose(dragDataForce_B[ind,1:4], [0, 0, 0], atol = 1e-11)
        np.testing.assert_allclose(dragTorqueData[ind,1:4], [0, 0, 0], atol = 1e-11)

def facetDragComp(dens, areas, coeffs, normals, vel, att):
    """Reference facet drag force in body frame using inertial velocity."""
    dcm = rbk.MRP2C(att)
    vMag = np.linalg.norm(vel)
    v_hat_B = dcm.dot(vel) / vMag
    force = np.zeros(3)
    for area, coeff, normal in zip(areas, coeffs, normals):
        proj = normal.dot(v_hat_B) * area
        if proj > 0:
            force += -0.5 * dens * proj * coeff * vMag**2 * v_hat_B
    return force


def setup_basic_facet_drag_sim(rN=None, vN=None, sigmaBN=None,
                                areas=None, coeffs=None, normals=None, locations=None):
    simTaskName = "simTask"
    simProcessName = "simProcess"

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(1.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    newAtmo.ModelTag = "ExpAtmo"
    newAtmo.baseDensity = 1.217  # [kg/m^3]
    newAtmo.scaleHeight = 8500.0  # [m]
    newAtmo.planetRadius = 6371.0 * 1000.0  # [m]
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsg)

    newDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    newDrag.ModelTag = "FacetDrag"
    newDrag.atmoDensInMsg.subscribeTo(newAtmo.envOutMsgs[0])

    if areas is None:
        areas = [1.0]
        coeffs = [2.2]
        normals = [np.array([1, 0, 0])]
        locations = [np.array([0, 0, 0])]
    for area, coeff, normal, loc in zip(areas, coeffs, normals, locations):
        newDrag.addFacet(area, coeff, normal, loc)

    scObject.addDynamicEffector(newDrag)

    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, newAtmo)
    scSim.AddModelToTask(simTaskName, newDrag)

    if rN is None:
        rN = np.array([7000e3, 0.0, 0.0])  # [m]
    if vN is None:
        vN = np.array([0.0, 7500.0, 0.0])  # [m/s]
    if sigmaBN is None:
        sigmaBN = np.array([0.0, 0.0, 0.0])  # [-] MRP

    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN
    scObject.hub.sigma_BNInit = sigmaBN

    dataLog = scObject.scStateOutMsg.recorder()
    atmoLog = newAtmo.envOutMsgs[0].recorder()
    dragLog = newDrag.logger("forceExternal_B")
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, atmoLog)
    scSim.AddModelToTask(simTaskName, dragLog)

    return scSim, scObject, newDrag, dataLog, atmoLog, dragLog, areas, coeffs, normals


def test_facetDragInertialVelocity():
    """Verify drag uses inertial velocity when no wind message is linked.

    Velocity has a component along the default facet normal [1, 0, 0] to ensure
    non-zero projected area and a non-trivial reference force.
    """
    rN = np.array([7000e3, 0.0, 0.0])  # [m]
    vN = np.array([3000.0, 7000.0, 0.0])  # [m/s]
    sigmaBN = np.array([0.0, 0.0, 0.0])  # [-] MRP

    scSim, _, _, dataLog, atmoLog, dragLog, areas, coeffs, normals = setup_basic_facet_drag_sim(
        rN=rN, vN=vN, sigmaBN=sigmaBN
    )

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()

    velData = dataLog.v_BN_N
    attData = dataLog.sigma_BN
    densData = atmoLog.neutralDensity
    dragForce = dragLog.forceExternal_B

    refForce = facetDragComp(densData[-1], areas, coeffs, normals, velData[-1], attData[-1])
    assert np.allclose(dragForce[-1, :], refForce, atol=1e-13)



def test_facetDragAtmosphereRelativeVelocityWhenWindLinked():
    """Test that drag uses atmosphere-relative velocity when wind message is linked.

    Two simulation steps are needed: step 1 populates windInData via ReadInputs();
    step 2's computeForceTorque uses that cached wind velocity.
    The force at step 2 is compared against facetDragComp using step-1 cached
    density/wind and step-2 velocity/attitude.
    """
    simTaskName = "simTask"
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.CreateNewProcess("sim").addTask(scSim.CreateNewTask(simTaskName, macros.sec2nano(1.0)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    newAtmo.ModelTag = "ExpAtmo"
    newAtmo.baseDensity = 1.217  # [kg/m^3]
    newAtmo.scaleHeight = 8500.0  # [m]
    newAtmo.planetRadius = 6371e3  # [m]
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsg)

    windMdl = zeroWindModel.ZeroWindModel()
    windMdl.ModelTag = "wind"
    windMdl.addSpacecraftToModel(scObject.scStateOutMsg)
    planetStateMsg = messaging.SpicePlanetStateMsg().write(messaging.SpicePlanetStateMsgPayload())
    windMdl.planetPosInMsg.subscribeTo(planetStateMsg)
    omega_earth = np.array([0.0, 0.0, orbitalMotion.OMEGA_EARTH])  # [rad/s]
    windMdl.setPlanetOmega_N(omega_earth)

    newDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    newDrag.ModelTag = "FacetDrag"
    newDrag.addFacet(1.0, 2.2, np.array([0, 1, 0]), np.array([0, 0, 0.1]))  # area [m^2], Cd [-], normal [-], CoP_B [m]
    newDrag.atmoDensInMsg.subscribeTo(newAtmo.envOutMsgs[0])
    newDrag.windVelInMsg.subscribeTo(windMdl.envOutMsgs[0])
    scObject.addDynamicEffector(newDrag)

    scObject.hub.r_CN_NInit = np.array([7000e3, 0.0, 0.0])  # [m]
    scObject.hub.v_CN_NInit = np.array([0.0, 7600.0, 0.0])  # [m/s]
    scObject.hub.sigma_BNInit = np.array([0.0, 0.0, 0.0])  # [-] MRP

    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, newAtmo)
    scSim.AddModelToTask(simTaskName, windMdl)
    scSim.AddModelToTask(simTaskName, newDrag)

    scSim.InitializeSimulation()

    windLog = windMdl.envOutMsgs[0].recorder()
    scLog = scObject.scStateOutMsg.recorder()
    atmoLog = newAtmo.envOutMsgs[0].recorder()
    dragLog = newDrag.logger("forceExternal_B")
    scSim.AddModelToTask(simTaskName, windLog)
    scSim.AddModelToTask(simTaskName, scLog)
    scSim.AddModelToTask(simTaskName, atmoLog)
    scSim.AddModelToTask(simTaskName, dragLog)

    # 2 steps: step 1 loads wind/atmo into cache; step 2 uses cache in computeForceTorque
    scSim.ConfigureStopTime(macros.sec2nano(2.0))
    scSim.ExecuteSimulation()

    # Wind velocity at step 2 should match co-rotation at step-2 position
    v_air_step2 = np.array(windLog.v_air_N[-1])
    r_step2 = np.array(scLog.r_CN_N[-1])
    np.testing.assert_allclose(v_air_step2, np.cross(omega_earth, r_step2), rtol=1e-10)

    # Force at step 2 uses cached step-1 wind/density with step-2 velocity/attitude
    dens_step1 = atmoLog.neutralDensity[0]
    wind_step1 = np.array(windLog.v_air_N[0])
    v_step2 = np.array(scLog.v_BN_N[-1])
    sigma_step2 = np.array(scLog.sigma_BN[-1])
    drag_force = np.array(dragLog.forceExternal_B[-1])
    expected_force = facetDragComp(dens_step1, [1.0], [2.2], [np.array([0, 1, 0])],
                                   v_step2 - wind_step1, sigma_step2)
    np.testing.assert_allclose(drag_force, expected_force, atol=1e-10)


if __name__=="__main__":
    scAreas = [1.0, 1.0]
    scCoeff = np.array([2.0, 2.0])
    B_normals = [np.array([0, 0, -1]), np.array([0, -1, 0])]
    B_locations = [np.array([0, 0, 0.1]), np.array([0, 0.1, 0])]
    test_DragCalculation(scAreas, scCoeff, B_normals, B_locations)
    test_ShadowCalculation(scAreas, scCoeff, B_normals, B_locations)
