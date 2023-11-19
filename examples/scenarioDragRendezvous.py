#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


r"""
Overview
--------

This script sets up a formation flying scenario with two spacecraft. The deputy spacecraft attempts to rendezvous with the
chief using attitude-driven differential drag using the strategy outlined in `this paper <https://arc.aiaa.org/doi/10.2514/1.G004521>`_.

This script is found in the folder ``src/examples`` and executed by using::

      python3 scenarioDragRendezvous

The simulation layout is shown in the following illustration. Two spacecraft are orbiting the earth at
close distance. Perturbations from atmospheric drag, provided by :ref:`exponentialAtmosphere` and
:ref:`facetDragDynamicEffector`,  are implemented by default; the :math:`J_2` gravity perturbation
can also be included. Each spacecraft sends a :ref:`simpleNav`
output message of type :ref:`NavAttMsgPayload` continuously to a :ref:`hillStateConverter` module,
which is assumed to be a part of the deputy (maneuvering)
spacecraft's flight software stack. The :ref:`hillStateConverter` module then writes
a :ref:`hillRelStateMsgPayload`, which is read by the :ref:`hillToAttRef` module implementing
the differential drag attitude guidance law. 

.. image:: /_images/static/scenarioDragRendezvousDiagram.png
   :align: center


Illustration of Simulation Results
----------------------------------

::

        0.0, #   altitude offset (m)
        0.1, #  True anomaly offset (deg)
        1, #    Density multiplier (non-dimensional)
        ctrlType='lqr',
        useJ2=False

In this case, the deputy spacecraft attempts to catch up to a reference set ten kilometers ahead of it
along-track using a static LQR control law, without considering the impact of :math:`J_2` perturbations.
The resulting relative attitude and in-plane Hill trajectory are shown below.


.. image:: /_images/Scenarios/scenarioDragRendezvous_relativeAtt.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDragRendezvous_hillTraj.svg
   :align: center

Time trajectories of the in-plane Hill components of the Deputy are shown here:

.. image:: /_images/Scenarios/scenarioDragRendezvous_hillX.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDragRendezvous_hillY.svg
   :align: center
"""

import os
import time

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.fswAlgorithms import hillStateConverter, hillToAttRef, hillPoint
from Basilisk.simulation import spacecraft, facetDragDynamicEffector, simpleNav, exponentialAtmosphere
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


#   Declare some linearized drag HCW dynamics
drag_state_dynamics = [[0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
                     [4.134628279603025589e-06,0.000000000000000000e+00,0.000000000000000000e+00,-7.178791202675993545e-10,2.347943292785702706e-03,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,-2.347943292785702706e-03,-1.435758240535198709e-09,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00]]
drag_ctrl_effects = [[0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,4.1681080996120926e-05],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00]
                     ]
drag_sens_effects = [[0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,-7.178791202675151888e-10,0.000000000000000000e+00,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,-1.435758240535030378e-09,0.000000000000000000e+00],
                     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00]]
drag_R_inv = [[1e-8,0,0],
              [0,1e-8,0],
              [0,0,1e-8]]
#   Static LQR gain
path = os.path.dirname(os.path.abspath(__file__))
dataFileName = os.path.join(path, "dataForExamples", "static_lqr_controlGain.npz")
lqr_gain_set = np.load(dataFileName)['arr_0.npy']

fileName = os.path.basename(os.path.splitext(__file__)[0])

def setup_spacecraft_plant(rN, vN, modelName):
    """
    Convenience function to set up a spacecraft and its associated dragEffector and simpleNav module.

    Args:
        rN (float(3)): Inertial position vector
        vN (float(3)): Inertial velocity vector
        modelName (string): String specifying the spacecraft name

    """

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = modelName
    scObject.hub.mHub = 6.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    I = [10., 0., 0.,
         0., 9., 0.,
         0., 0., 8.]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN

    scNav = simpleNav.SimpleNav()
    scNav.ModelTag = modelName+'_navigator'
    scNav.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    dragArea = 0.3*0.2
    dragCoeff = 2.2
    normalVector = -rbk.euler3(np.radians(45.)).dot(np.array([0,-1,0]))
    panelLocation = [0,0,0]
    dragEffector = facetDragDynamicEffector.FacetDragDynamicEffector()
    dragEffector.ModelTag = modelName+'_dragEffector'
    dragEffector.addFacet(dragArea, dragCoeff, normalVector, panelLocation)
    scObject.addDynamicEffector(dragEffector)

    return scObject, dragEffector, scNav


def drag_simulator(altOffset, trueAnomOffset, densMultiplier, ctrlType='lqr', useJ2=False):
    """
    Basilisk simulation of a two-spacecraft rendezvous using relative-attitude driven differential drag. Includes
    both static gain and desensitized time-varying gain options and the option to use simulated attitude control or
    direct reference inputs.

    Args:
        altOffset - double - deputy altitude offset from the chief ('x' hill direction), meters
        trueAnomOffset - double - deputy true anomaly difference from the chief ('y' direction), degrees
    """

    startTime = time.time()
    print(f"Starting process execution for altOffset = {altOffset}, trueAnomOffset={trueAnomOffset}, densMultiplier={densMultiplier} with {ctrlType} controls...")

    scSim = SimulationBaseClass.SimBaseClass()

    #   Configure simulation container and timestep
    simProcessName = "simProcess"
    dynTaskName = "dynTask"
    fswTaskName = "fswTask"
    simProcess = scSim.CreateNewProcess(simProcessName, 2)
    dynTimeStep = macros.sec2nano(60.0)  # Timestep to evaluate dynamics at
    fswTimeStep = macros.sec2nano(60.0)  # Timestep to evaluate FSW at
    simProcess.addTask(scSim.CreateNewTask(dynTaskName, dynTimeStep))
    simProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    ##  Configure environmental parameters
    #   Gravity; includes 2-body plus J2.
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth'])
    gravBodies['earth'].isCentralBody = True
    if useJ2:
        gravBodies['earth'].useSphericalHarmonicsGravityModel(bskPath + '/supportData/LocalGravData/GGM03S.txt', 2)
    # timeInitString = '2021 MAY 04 07:47:48.965 (UTC)'
    # gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/'
    #                                           , timeInitString
    #                                           )
    # gravFactory.spiceObject.zeroBase = 'earth'

    #   Density
    atmosphere = exponentialAtmosphere.ExponentialAtmosphere()
    atmosphere.ModelTag = 'atmosphere'
    # atmosphere.planetPosInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    atmosphere.planetRadius = orbitalMotion.REQ_EARTH*1e3 + 300e3 #   m
    atmosphere.envMinReach = -300e3
    atmosphere.envMaxReach = +300e3
    atmosphere.scaleHeight = 8.0e3    # m
    atmosphere.baseDensity =  2.022E-14 * 1000 *  densMultiplier #    kg/m^3

    ##   Set up chief, deputy orbits:
    mu = gravFactory.gravBodies['earth'].mu
    chief_oe = orbitalMotion.ClassicElements()
    chief_oe.a = orbitalMotion.REQ_EARTH * 1e3 + 300e3 # meters
    chief_oe.e = 0.
    chief_oe.i = np.radians(45.)

    chief_oe.Omega = np.radians(20.0)
    chief_oe.omega = np.radians(30.)
    chief_oe.f = np.radians(20.)
    chief_rN, chief_vN = orbitalMotion.elem2rv(mu, chief_oe)

    dep_oe = orbitalMotion.ClassicElements()
    dep_oe.a = orbitalMotion.REQ_EARTH * 1e3 + 300e3 + (altOffset)  # meters
    dep_oe.e = 0.
    dep_oe.i = np.radians(45.)
    dep_oe.Omega = np.radians(20.0)
    dep_oe.omega = np.radians(30.)
    dep_oe.f = np.radians(20. - trueAnomOffset)
    dep_rN, dep_vN = orbitalMotion.elem2rv(mu, dep_oe)

    #   Initialize s/c dynamics, drag, navigation solutions
    chiefSc, chiefDrag, chiefNav = setup_spacecraft_plant(chief_rN, chief_vN, 'wiggum')
    depSc, depDrag, depNav = setup_spacecraft_plant(dep_rN,dep_vN, 'lou')

    #   Connect s/c to environment (gravity, density)
    chiefSc.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    depSc.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    atmosphere.addSpacecraftToModel(depSc.scStateOutMsg)
    depDrag.atmoDensInMsg.subscribeTo(atmosphere.envOutMsgs[-1])
    atmosphere.addSpacecraftToModel(chiefSc.scStateOutMsg)
    chiefDrag.atmoDensInMsg.subscribeTo(atmosphere.envOutMsgs[-1])

    #   Add all dynamics stuff to dynamics task
    scSim.AddModelToTask(dynTaskName, atmosphere,920)
    # scSim.AddModelToTask(dynTaskName, ephemConverter, 921)
    scSim.AddModelToTask(dynTaskName, chiefDrag,890)
    scSim.AddModelToTask(dynTaskName, depDrag,891)
    scSim.AddModelToTask(dynTaskName, chiefSc,810)
    scSim.AddModelToTask(dynTaskName, depSc,811)
    scSim.AddModelToTask(dynTaskName, chiefNav,800)
    scSim.AddModelToTask(dynTaskName, depNav,801)
    # scSim.AddModelToTask(dynTaskName, gravFactory.spiceObject, 999)

    ##  FSW setup
    #   Chief S/C
    #   hillPoint - set such that the chief attitude follows its hill frame.
    chiefAttRef = hillPoint.hillPoint()
    chiefAttRef.ModelTag = 'chief_att_ref'
    chiefAttRef.transNavInMsg.subscribeTo(chiefNav.transOutMsg)
    # chiefAttRefData.celBodyInMsg.subscribeTo(ephemConverter.ephemOutMsgs[-1]) #   We shouldn't need this because the planet is the origin
    chiefSc.attRefInMsg.subscribeTo(chiefAttRef.attRefOutMsg) #  Force the chief spacecraft to follow the hill direction

    depHillRef = hillPoint.hillPoint()
    depHillRef.ModelTag = 'dep_hill_ref'
    depHillRef.transNavInMsg.subscribeTo(depNav.transOutMsg)
    # chiefAttRefData.celBodyInMsg.subscribeTo(ephemConverter.ephemOutMsgs[-1]) #   We shouldn't need this because the planet is the origin
    #chiefSc.attRefInMsg.subscribeTo(chiefAttRefData.attRefOutMsg) #  Force the chief spacecraft to follow the hill direction

    # hillStateConverter
    hillStateNavObj = hillStateConverter.hillStateConverter()
    hillStateNavObj.ModelTag = "dep_hillStateNav"
    hillStateNavObj.depStateInMsg.subscribeTo(depNav.transOutMsg)
    hillStateNavObj.chiefStateInMsg.subscribeTo(chiefNav.transOutMsg)

    # hillToAtt guidance law w/ static gain
    depAttRef = hillToAttRef.hillToAttRef()
    depAttRef.ModelTag = 'dep_att_ref'
    depAttRef.gainMatrix = hillToAttRef.MultiArray(lqr_gain_set)
    #   Configure parameters common to relative attitude guidance modules
    depAttRef.hillStateInMsg.subscribeTo(hillStateNavObj.hillStateOutMsg)
    # depAttRef.attStateInMsg.subscribeTo(chiefNav.attOutMsg)
    depAttRef.attRefInMsg.subscribeTo(depHillRef.attRefOutMsg)
    depAttRef.relMRPMin = -0.2
    depAttRef.relMRPMax = 0.2
    #   Set the deputy spacecraft to directly follow the attRefMessage
    depSc.attRefInMsg.subscribeTo(depAttRef.attRefOutMsg)


    scSim.AddModelToTask(dynTaskName, chiefAttRef, 710)
    scSim.AddModelToTask(dynTaskName, hillStateNavObj,790)
    scSim.AddModelToTask(dynTaskName, depHillRef,789)
    scSim.AddModelToTask(dynTaskName, depAttRef, 700)
    # ----- log ----- #
    orbit_period = 2*np.pi/np.sqrt(mu/chief_oe.a**3)
    simulationTime = 40*orbit_period#106920.14366466808 
    simulationTime = macros.sec2nano(simulationTime)
    numDataPoints = 21384
    samplingTime = simulationTime // (numDataPoints - 1)

    #   Set up recorders and loggers
    chiefStateRec = chiefSc.scStateOutMsg.recorder()
    depStateRec = depSc.scStateOutMsg.recorder()
    hillStateRec = hillStateNavObj.hillStateOutMsg.recorder()
    depAttRec = depAttRef.attRefOutMsg.recorder()
    chiefAttRec = chiefAttRef.attRefOutMsg.recorder()
    chiefDragForceLog = chiefDrag.logger("forceExternal_B")
    depDragForceLog = depDrag.logger("forceExternal_B")
    atmoRecs = []
    for msg in atmosphere.envOutMsgs:
        atmoRec = msg.recorder()
        atmoRecs.append(atmoRec)

    scSim.AddModelToTask(dynTaskName, chiefStateRec, 700)
    scSim.AddModelToTask(dynTaskName, depStateRec, 701)
    scSim.AddModelToTask(dynTaskName, hillStateRec, 702)
    scSim.AddModelToTask(dynTaskName, depAttRec, 703)
    scSim.AddModelToTask(dynTaskName, chiefAttRec, 704)
    scSim.AddModelToTask(dynTaskName, chiefDragForceLog, 705)
    scSim.AddModelToTask(dynTaskName, depDragForceLog, 706)
    
    for ind,rec in enumerate(atmoRecs):
        scSim.AddModelToTask(dynTaskName, rec, 707+ind)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, [chiefSc, depSc],
                                              # saveFile=fileName,
                                              )
    
    # ----- execute sim ----- #
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    setupTimeStamp = time.time()
    setupTime = setupTimeStamp-startTime
    print(f"Sim setup complete in {setupTime} seconds, executing...")
    #scSim.ShowExecutionFigure(True)
    scSim.ExecuteSimulation()

    execTimeStamp = time.time()
    execTime = execTimeStamp - setupTimeStamp
    print(f"Sim complete in {execTime} seconds, pulling data...")
    # ----- pull ----- #
    results_dict = {}
    results_dict['chiefDrag_B'] = chiefDragForceLog.forceExternal_B
    results_dict['depDrag_B'] = depDragForceLog.forceExternal_B
    results_dict['dynTimeData'] = chiefStateRec.times()
    results_dict['fswTimeData'] = depAttRec.times()
    results_dict['wiggum.r_BN_N'] = chiefStateRec.r_BN_N 
    results_dict['wiggum.v_BN_N'] = chiefStateRec.v_BN_N
    results_dict['lou.r_BN_N'] = depStateRec.r_BN_N
    results_dict['lou.v_BN_N'] = depStateRec.v_BN_N
    results_dict['relState.r_DC_H'] = hillStateRec.r_DC_H
    results_dict['relState.v_DC_H'] = hillStateRec.v_DC_H
    results_dict['wiggum.sigma_BN'] = chiefStateRec.sigma_BN 
    results_dict['lou.sigma_BN'] = depStateRec.sigma_BN
    results_dict['depDensity'] = atmoRecs[0].neutralDensity
    results_dict['chiefDensity'] = atmoRecs[1].neutralDensity
    results_dict['mu'] = mu
    results_dict['dens_mult'] = densMultiplier
    pullTimeStamp = time.time()
    pullTime = pullTimeStamp - execTimeStamp
    overallTime = pullTimeStamp - startTime
    print(f"Data pulled in {pullTime} seconds. Overall time: {overallTime} seconds.")
    return results_dict


def run(show_plots, altOffset, trueAnomOffset, densMultiplier, ctrlType='lqr', useJ2=False):
    results = drag_simulator(altOffset, trueAnomOffset, densMultiplier, ctrlType=ctrlType, useJ2=useJ2)

    timeData = results['dynTimeData']
    fswTimeData = results['fswTimeData']
    pos = results['wiggum.r_BN_N']
    vel = results['wiggum.v_BN_N']
    chiefAtt = results['wiggum.sigma_BN']
    pos2 = results['lou.r_BN_N']
    vel2= results['lou.v_BN_N']
    depAtt = results['lou.sigma_BN']
    hillPos = results['relState.r_DC_H']
    hillVel = results['relState.v_DC_H']
    depDensity = results['depDensity']
    chiefDensity = results['chiefDensity']
    depDrag = results['depDrag_B']
    chiefDrag = results['chiefDrag_B']
    densData = results
    numDataPoints = len(timeData)
    mu = results['mu']
    rel_mrp_hist = np.empty([numDataPoints,3])

    for ind in range(0,numDataPoints):
        rel_mrp_hist[ind,:] = rbk.subMRP(depAtt[ind,:], chiefAtt[ind,:])

    figureList = {}
    
    #   Plots for general consumption
    plt.figure() 
    plt.plot(timeData[1:], hillPos[1:,0],label="r_1")
    plt.grid()
    plt.xlabel('Time')   
    plt.ylabel('Hill X Position (m)')
    pltName = fileName + "_hillX"
    figureList[pltName] = plt.figure(1)
    plt.figure()
    plt.plot(timeData[1:], hillPos[1:,1],label="r_2")
    plt.grid()
    plt.xlabel('Time')   
    plt.ylabel('Hill Y Position (m)')
    pltName = fileName + "_hillY"
    figureList[pltName] = plt.figure(2)
    

    plt.figure()
    plt.plot(timeData[1:], hillVel[1:,0],label="v_1")
    plt.grid()
    plt.xlabel('Time')   
    plt.ylabel('Hill X Velocity (m/s)')
    pltName = fileName + "_hilldX"
    figureList[pltName] = plt.figure(3)
    plt.figure()
    plt.plot(timeData[1:], hillVel[1:,1],label="v_2")
    plt.ylabel('Hill Y Velocity (m/s)')
    pltName = fileName + "_hilldy"
    figureList[pltName] = plt.figure(4)

    plt.figure()
    plt.semilogy(timeData[1:], chiefDensity[1:],label=r'Chief $\rho$')
    plt.semilogy(timeData[1:], depDensity[1:],label=r'Deputy $\rho$')
    plt.grid()
    plt.legend()
    plt.xlabel('Time')
    plt.ylabel('Density (kg/m3)')
    pltName = fileName + "_densities"
    figureList[pltName] = plt.figure(5)

    plt.figure()
    plt.plot(hillPos[1:,0],hillPos[1:,1])
    plt.grid()
    plt.xlabel('Hill X (m)')
    plt.ylabel('Hill Y (m)')
    pltName = fileName + "_hillTraj"
    figureList[pltName] = plt.figure(6)

    plt.figure()
    plt.plot(timeData, rel_mrp_hist)
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Relative MRP Value')
    pltName = fileName + "_relativeAtt"
    figureList[pltName] = plt.figure(7)

    #   Debug plots
    plt.figure()
    plt.plot(timeData[1:], depDrag[1:,0]-chiefDrag[1:,0],label="delta a_1")
    plt.plot(timeData[1:], depDrag[1:,1]-chiefDrag[1:,1],label="delta a_2")
    plt.plot(timeData[1:], depDrag[1:,2]-chiefDrag[1:,2],label="delta a_3")
    plt.grid()
    plt.legend()
    plt.xlabel('Time')   
    plt.ylabel('Relative acceleration due to drag, body frame (m/s)')

    plt.figure()
    plt.plot(timeData[1:], chiefDrag[1:,0],label="chief a_1")
    plt.plot(timeData[1:], chiefDrag[1:,1],label="chief a_2")
    plt.plot(timeData[1:], chiefDrag[1:,2],label="chief a_3")
    plt.grid()
    plt.legend()
    plt.xlabel('Time')   
    plt.ylabel('Relative acceleration due to drag, body frame (m/s)')

    plt.figure()
    plt.plot(timeData[1:], depDrag[1:,0],label="dep a_1")
    plt.plot(timeData[1:], depDrag[1:,1],label="dep a_2")
    plt.plot(timeData[1:], depDrag[1:,2],label="dep a_3")
    plt.grid()
    plt.legend()
    plt.xlabel('Time')   
    plt.ylabel('Relative acceleration due to drag, body frame (m/s)')
    

    if(show_plots):
        plt.show()
    plt.close("all")

    return figureList

if __name__ == "__main__":
    run(
        True,  # show_plots
        0.0, #   altitude offset (m)
        0.1, #  True anomaly offset (deg)
        1, #    Density multiplier (nondimensional)
        ctrlType='lqr',
        useJ2=False
    )
