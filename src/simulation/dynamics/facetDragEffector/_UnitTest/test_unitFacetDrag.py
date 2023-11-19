
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
from Basilisk.simulation import spacecraft
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.simulation import facetDragDynamicEffector
from Basilisk.simulation import simpleNav
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

    r_eq = 6371*1000.0
    refBaseDens = 1.217
    refScaleHeight = 8500.0

    #   Set base density, equitorial radius, scale height in Atmosphere
    newAtmo.baseDensity = refBaseDens
    newAtmo.scaleHeight = refScaleHeight
    newAtmo.planetRadius = r_eq

    rN = np.array([r_eq+200.0e3,0,0])
    vN = np.array([0,7.788e3,0])
    sig_BN = np.array([0,0,0])
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

    r_eq = 6371*1000.0
    refBaseDens = 1.217
    refScaleHeight = 8500.0

    #   Set base density, equitorial radius, scale height in Atmosphere
    newAtmo.baseDensity = refBaseDens
    newAtmo.scaleHeight = refScaleHeight
    newAtmo.planetRadius = r_eq

    rN = np.array([r_eq+200.0e3,0,0])
    vN = np.array([0,7.788e3,0])
    sig_BN = np.array([0,0,0])

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

if __name__=="__main__":
    scAreas = [1.0, 1.0]
    scCoeff = np.array([2.0, 2.0])
    B_normals = [np.array([0, 0, -1]), np.array([0, -1, 0])]
    B_locations = [np.array([0, 0, 0.1]), np.array([0, 0.1, 0])]
    test_DragCalculation(scAreas, scCoeff, B_normals, B_locations)
    test_ShadowCalculation(scAreas, scCoeff, B_normals, B_locations)
