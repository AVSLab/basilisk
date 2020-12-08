#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
chief using attitude-driven differential drag.

This script is found in the folder ``src/examples`` and executed by using::

      python3 scenarioDragRenzesvous

The simulation layout is shown in the following illustration. Two spacecraft are orbiting the earth at
close distance. Only :math:`J_2` gravity perturbation is included. Each spacecraft sends a :ref:`simple_nav`
output message of type :ref:`NavAttIntMsg` message at a certain period
to :ref:`meanOEFeedback`, where mean orbital element difference is calculated and necessary control force is output to
extForceTorque module.

.. image:: /_images/static/test_scenarioFormationMeanOEFeedback.svg
   :align: center


Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useClassicElem = True

In this case, target orbital element difference is set based on classical orbital element.
This resulting feedback control error is shown below.


.. image:: /_images/Scenarios/scenarioFormationMeanOEFeedback11.svg
   :align: center

::

    show_plots = True, useClassicElem = False

In this case, target orbital element difference is set based on equinoctial orbital element.
This resulting feedback control error is shown below.

.. image:: /_images/Scenarios/scenarioFormationMeanOEFeedback20.svg
   :align: center


"""

import os
import copy
import time

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import vizSupport

from Basilisk.simulation import spacecraftPlus, facetDragDynamicEffector, simple_nav, exponentialAtmosphere, vizInterface
from Basilisk.fswAlgorithms import hillStateConverter, hillToAttRef, hillPoint, linSensitivityProp, desenHillToAttRef
from Basilisk import __path__
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
#/home/andrew/dev/basilisk/src/examples/data/ddsen_gain.npz
#   Time-varying LQR gain
lqr_gain_set = np.load('./data/lqr_tv_gain.npz')['arr_0.npy']
lqr_gain_set = np.transpose(lqr_gain_set, (2,0,1))

#   Desensitized gain (no control sensitivities)
dsen_gain_set = np.load('./data/desensitized_gain.npz')['arr_0.npy']
dsen_gain_set = np.transpose(dsen_gain_set, (2,0,1))
#   Desensitized gain (includes control sensitivites)

ddsen_gain_set = np.load('./data/ctrl_desensitized_gain.npz')['arr_0.npy']
ddsen_gain_set = np.transpose(ddsen_gain_set, (2,0,1))

def setup_spacecraft_plant(rN, vN, modelName,):
    """
    Convenience function to set up a spacecraft and its associated dragEffector and simpleNav module.
    :param rN:
    :param vN:
    :return:
    """

    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = modelName
    scObject.scStateOutMsgName = modelName+'_inertial_states'
    scObject.scMassStateOutMsgName = modelName+'_mass_states'
    scObject.hub.mHub = 3.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    I = [1000., 0., 0.,
         0., 900., 0.,
         0., 0., 800.]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN

    scNav = simple_nav.SimpleNav()
    scNav.inputStateName = scObject.scStateOutMsgName
    scNav.outputTransName = modelName + '_trans_nav_state'
    scNav.outputAttName = modelName + '_att_nav_state'

    dragArea = 2.0
    dragCoeff = 2.2
    normalVector = rbk.euler3(np.radians(45.)).dot(np.array([0,1,0]))
    panelLocation = [0,0,0]
    dragEffector = facetDragDynamicEffector.FacetDragDynamicEffector()
    dragEffector.addFacet(dragArea, dragCoeff, normalVector, panelLocation)
    scObject.addDynamicEffector(dragEffector)

    return scObject, dragEffector, scNav


def drag_simulator(altOffset, trueAnomOffset, densMultiplier, ctrlType='lqr', makeViz=False):
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
    dynTimeStep = macros.sec2nano(0.1) #   Timestep to evaluate dynamics at
    fswTimeStep = macros.sec2nano(5.0) #   Timestep to evaluate FSW at
    simProcess.addTask(scSim.CreateNewTask(dynTaskName, dynTimeStep))
    simProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    ##  Configure environmental parameters
    #   Gravity; includes 2-body plus J2.
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth'])
    gravBodies['earth'].isCentralBody = True
    #simIncludeGravBody.loadGravFromFile(bskPath + '/supportData/LocalGravData/GGM03S.txt', gravBodies['earth'].spherHarm, 2)
    gravBodies['earth'].useSphericalHarmParams = False

    #   Density
    atmosphere = exponentialAtmosphere.ExponentialAtmosphere()
    atmosphere.ModelTag = 'atmosphere'
    atmosphere.planetRadius = 6378137.0#orbitalMotion.REQ_EARTH*1e3   #   m
    atmosphere.scaleHeight = 8.0e3    # m
    atmosphere.baseDensity = 1.020 * densMultiplier #    kg/m^3

    ##   Set up chief, deputy orbits:
    mu = gravFactory.gravBodies['earth'].mu
    chief_oe = orbitalMotion.ClassicElements()
    chief_oe.a = orbitalMotion.REQ_EARTH * 1e3 + 230e3 # meters
    chief_oe.e = 0.
    chief_oe.i = 45.
    chief_oe.Omega = 20.0 * macros.D2R
    chief_oe.omega = 30.0 * macros.D2R
    chief_oe.f = 20.*macros.D2R
    chief_rN, chief_vN = orbitalMotion.elem2rv(mu, chief_oe)

    dep_oe = orbitalMotion.ClassicElements()
    dep_oe.a = orbitalMotion.REQ_EARTH * 1e3 + 230e3 + (altOffset)  # meters
    dep_oe.e = 0.
    dep_oe.i = 45.
    dep_oe.Omega = 20.0 * macros.D2R
    dep_oe.omega = 30.0 * macros.D2R
    dep_oe.f = (20. - trueAnomOffset) * macros.D2R
    dep_rN, dep_vN = orbitalMotion.elem2rv(mu, dep_oe)

    #   Initialize s/c dynamics, drag, navigation solutions
    chiefSc, chiefDrag, chiefNav = setup_spacecraft_plant(chief_rN, chief_vN, 'wiggum')
    depSc, depDrag, depNav = setup_spacecraft_plant(dep_rN,dep_vN, 'lou')

    #   Connect s/c to environment (gravity, density)
    chiefSc.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))
    depSc.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

    atmosphere.addSpacecraftToModel(depSc.scStateOutMsgName)
    depDrag.atmoDensInMsgName = atmosphere.envOutMsgNames[-1]
    atmosphere.addSpacecraftToModel(chiefSc.scStateOutMsgName)
    chiefDrag.atmoDensInMsgName = atmosphere.envOutMsgNames[-1]

    #   Add all dynamics stuff to dynamics task
    scSim.AddModelToTask(dynTaskName, atmosphere,920)
    scSim.AddModelToTask(dynTaskName, chiefDrag,890)
    scSim.AddModelToTask(dynTaskName, depDrag,891)
    scSim.AddModelToTask(dynTaskName, chiefSc,810)
    scSim.AddModelToTask(dynTaskName, depSc,811)
    scSim.AddModelToTask(dynTaskName, chiefNav,800)
    scSim.AddModelToTask(dynTaskName, depNav,801)

    ##  FSW setup
    #   Chief S/C
    #   hillPoint - set to just point at a banked angle in the hill direction.
    chiefAttRefData = hillPoint.hillPointConfig()
    chiefAttRefWrap = scSim.setModelDataWrap(chiefAttRefData)
    chiefAttRefWrap.ModelTag = 'chief_att_ref'
    chiefAttRefData.outputDataName = 'chief_att_ref'
    chiefAttRefData.inputNavDataName = chiefNav.outputTransName
    chiefAttRefData.inputCelMessName = gravFactory.gravBodies['earth'].bodyInMsgName
    chiefSc.attRefInMsgName = chiefAttRefData.outputDataName #  Force the chief spacecraft to follow the hill direction

    # hillStateConverter
    hillStateNavData = hillStateConverter.HillStateConverterConfig()
    hillStateNavWrap = scSim.setModelDataWrap(hillStateNavData)
    hillStateNavWrap.ModelTag = "dep_hillStateNav"
    hillStateNavData.chiefStateInMsgName = chiefNav.outputTransName
    hillStateNavData.depStateInMsgName = depNav.outputTransName
    hillStateNavData.hillStateOutMsgName = 'dep_hill_nav'

    # linear sensitivity propagator
    sensProp = linSensitivityProp.LinSensProp()
    sensProp.depAttInMsgName = depNav.outputAttName 
    sensProp.chiefAttInMsgName = chiefNav.outputAttName
    sensProp.hillStateInMsgName = 'dep_hill_nav'
    sensProp.sensOutMsgName = 'dep_sens_nav'
    sensProp.C = hillToAttRef.MultiArray(drag_sens_effects)
    sensProp.A = hillToAttRef.MultiArray(drag_state_dynamics)
    if ctrlType=='ddesen':
        sensProp.D = hillToAttRef.MultiArray(drag_ctrl_effects)
    else:
        sensProp.D = hillToAttRef.MultiArray(np.zeros(np.array(drag_ctrl_effects).shape))  

    #   Configure module-specific control parameters
    if ctrlType == 'desen' or ctrlType=='ddesen':
        if ctrlType=='desen':
            gain_set = dsen_gain_set
        else:
            gain_set = ddsen_gain_set
            drag_R_inv = -np.linalg.inv(1e7*np.identity(3))

        #   desensitized relative attitude control w/ dynamic gain
        depAttRef = desenHillToAttRef.DesenHillToAttRef()
        depAttRef.ModelTag = 'dep_att_ref'
        depAttRef.sensInMsgName = sensProp.sensOutMsgName
        depAttRef.B = hillToAttRef.MultiArray(drag_ctrl_effects)
        depAttRef.Rinv = hillToAttRef.MultiArray(drag_R_inv)
        depAttRef.stateGainVec = hillToAttRef.MultiArray3d(gain_set[:,0:6,0:6])
        depAttRef.sensGainVec = hillToAttRef.MultiArray3d(gain_set[:,0:6,6:])

    elif ctrlType == 'tv_lqr':
        # hillToAtt guidance law w/ time-varying gain
        depAttRef = desenHillToAttRef.DesenHillToAttRef()
        depAttRef.ModelTag = 'dep_att_ref'
        depAttRef.sensInMsgName = sensProp.sensOutMsgName
        depAttRef.B = hillToAttRef.MultiArray(drag_ctrl_effects)
        
        drag_R_inv = -np.linalg.inv(1e7*np.identity(3))
        depAttRef.Rinv = hillToAttRef.MultiArray(drag_R_inv)
        depAttRef.stateGainVec = hillToAttRef.MultiArray3d(lqr_gain_set)
        depAttRef.sensGainVec = hillToAttRef.MultiArray3d(np.zeros(lqr_gain_set.shape))

    else:
        # hillToAtt guidance law w/ static gain
        depAttRef = hillToAttRef.HillToAttRef()
        depAttRef.ModelTag = 'dep_att_ref'
        depAttRef.gainMatrixVec = hillToAttRef.MultiArray3d(np.array([[[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,   0.00000000e+00,  0.00000000e+00],
                                                                       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,   0.00000000e+00, 0.00000000e+00],
                                                                       [ 1.28982586e-02, -9.99999987e-05,  0.00000000e+00,  1.90838094e-01,   5.58803922e+00,  0.00000000e+00]]]))

    #   Configure parameters common to relative attitude guidance modules
    depAttRef.hillStateInMsgName = hillStateNavData.hillStateOutMsgName
    depAttRef.attStateInMsgName = chiefNav.outputAttName
    depAttRef.attRefOutMsgName = 'dep_att_ref'
    depAttRef.relMRPMin = -0.2
    depAttRef.relMRPMax = 0.2
    #   Set the deputy spacecraft to directly follow the attRefMessage
    depSc.attRefInMsgName = depAttRef.attRefOutMsgName

    scSim.AddModelToTask(fswTaskName, chiefAttRefWrap, chiefAttRefData, 710)
    scSim.AddModelToTask(fswTaskName, hillStateNavWrap, hillStateNavData,790)
    scSim.AddModelToTask(fswTaskName, sensProp,780)
    scSim.AddModelToTask(fswTaskName, depAttRef,700)
    # ----- log ----- #
    orbit_period = 2*np.pi/np.sqrt(mu/chief_oe.a**3)
    simulationTime = 80*orbit_period#106920.14366466808 
    simulationTime = macros.sec2nano(simulationTime)
    numDataPoints = 21384
    samplingTime = simulationTime // (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(chiefSc.scStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(depSc.scStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(hillStateNavData.hillStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(sensProp.sensOutMsgName, samplingTime)
    for msg in atmosphere.envOutMsgNames:
        scSim.TotalSim.logThisMessage(msg, samplingTime)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    if makeViz:
        viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, simProcessName, gravBodies=gravFactory,
                                                  saveFile=fileName,
                                                  scName=[chiefSc.ModelTag, depSc.ModelTag])
        # delete any existing list of vizInterface spacecraft data
        viz.scData.clear()

        # create a chief spacecraft info container
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = chiefSc.ModelTag
        scData.numRW = 0
        scData.scPlusInMsgName = chiefSc.scStateOutMsgName
        # the following command is required as we are deviating from the default naming of using the Model.Tag
        viz.scData.push_back(scData)

        # create a chief spacecraft info container
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = depSc.ModelTag
        scData.numRW = 0
        scData.scPlusInMsgName = depSc.scStateOutMsgName
        # the following command is required as we are deviating from the default naming of using the Model.Tag
        viz.scData.push_back(scData)

    # ----- execute sim ----- #
    scSim.InitializeSimulationAndDiscover()
    scSim.ConfigureStopTime(simulationTime)
    setupTimeStamp = time.time()
    setupTime = setupTimeStamp-startTime
    print(f"Sim setup complete in {setupTime} seconds, executing...")

    scSim.ExecuteSimulation()

    execTimeStamp = time.time()
    execTime = execTimeStamp - setupTimeStamp
    print(f"Sim complete in {execTime} seconds, pulling data...")
    # ----- pull ----- #
    varNames = [chiefSc.scStateOutMsgName + '.r_BN_N', chiefSc.scStateOutMsgName + '.v_BN_N', chiefSc.scStateOutMsgName+'.sigma_BN',
                depSc.scStateOutMsgName + '.r_BN_N', depSc.scStateOutMsgName + '.v_BN_N',depSc.scStateOutMsgName+'.sigma_BN',
                hillStateNavData.hillStateOutMsgName+'.r_DC_H',hillStateNavData.hillStateOutMsgName+'.v_DC_H',
                sensProp.sensOutMsgName+'.r_DC_H', sensProp.sensOutMsgName+'.v_DC_H',
                ]
    varSizes = [list(range(3))]*len(varNames)
    types = ['double'] * len(varNames)
    results_dict = scSim.pullMultiMessageLogData(varNames, varSizes, types) #   use pullMultiMessages for O(n) pull scaling
    results_dict['dynTimeData'] = results_dict[chiefSc.scStateOutMsgName+'.r_BN_N'][:, 0]*macros.NANO2SEC/orbit_period
    results_dict['fswTimeData'] = results_dict[sensProp.sensOutMsgName+'.r_DC_H'][:, 0]*macros.NANO2SEC/orbit_period
    results_dict['mu'] = mu
    results_dict['dens_mult'] = densMultiplier
    pullTimeStamp = time.time()
    pullTime = pullTimeStamp - execTimeStamp
    overallTime = pullTimeStamp - startTime
    print(f"Data pulled in {pullTime} seconds. Overall time: {overallTime} seconds.")
    return results_dict


def run(show_plots, altOffset, trueAnomOffset, densMultiplier, ctrlType='lqr'):
    results = drag_simulator(altOffset, trueAnomOffset, densMultiplier, ctrlType=ctrlType, makeViz=True,)

    timeData = results['dynTimeData']
    fswTimeData = results['fswTimeData']
    pos = results['wiggum_inertial_states.r_BN_N']
    vel = results['wiggum_inertial_states.v_BN_N']
    chiefAtt = results['wiggum_inertial_states.sigma_BN']
    pos2 = results['lou_inertial_states.r_BN_N']
    vel2= results['lou_inertial_states.v_BN_N']
    depAtt = results['lou_inertial_states.sigma_BN']
    hillPos = results['dep_hill_nav.r_DC_H']
    hillVel = results['dep_hill_nav.v_DC_H']
    hillPosSens = results['dep_sens_nav.r_DC_H']
    hillVelSens = results['dep_sens_nav.v_DC_H']
    # depDensity = results[atmosphere.envOutMsgNames[0]+'.density']
    # chiefDensity = results[atmosphere.envOutMsgNames[1]+'.density']
    densData = results
    numDataPoints = len(timeData)
    mu = results['mu']

    rel_mrp_hist = np.empty([numDataPoints,3])

    for ind in range(0,numDataPoints):
        rel_mrp_hist[ind,:] = rbk.subMRP(depAtt[ind,1:4], chiefAtt[ind,1:4])

    #swTimeData, pos, vel, chiefAtt, pos2, vel2, depAtt, hillPos, hillVel, hillPosSens, hillVelSens, numDataPoints, mu,
    # ----- plot ----- #
    # classical oe (figure1)
    plt.figure(1)
    oed_cl = np.empty((len(pos[:, 0]), 6))
    for i in range(0, len(pos[:, 0])):
        # spacecraft 1 (chief)
        oe_cl_osc = orbitalMotion.rv2elem(mu, pos[i, 1:4], vel[i, 1:4])
        oe_cl_mean = orbitalMotion.ClassicElements()
        orbitalMotion.clMeanOscMap(orbitalMotion.REQ_EARTH*1e3, orbitalMotion.J2_EARTH, oe_cl_osc, oe_cl_mean, -1)
        # spacecraft 2 (deputy)
        oe2_cl_osc = orbitalMotion.rv2elem(mu, pos2[i, 1:4], vel2[i, 1:4])
        oe2_cl_mean = orbitalMotion.ClassicElements()
        orbitalMotion.clMeanOscMap(orbitalMotion.REQ_EARTH*1e3, orbitalMotion.J2_EARTH, oe2_cl_osc, oe2_cl_mean, -1)
        # calculate oed
        oed_cl[i, 0] = (oe2_cl_mean.a - oe_cl_mean.a)/oe_cl_mean.a  # delta a (normalized)
        oed_cl[i, 1] = oe2_cl_mean.e - oe_cl_mean.e  # delta e
        oed_cl[i, 2] = oe2_cl_mean.i - oe_cl_mean.i  # delta i
        oed_cl[i, 3] = oe2_cl_mean.Omega - oe_cl_mean.Omega  # delta Omega
        oed_cl[i, 4] = oe2_cl_mean.omega - oe_cl_mean.omega  # delta omega
        E_tmp = orbitalMotion.f2E(oe_cl_mean.f, oe_cl_mean.e)
        E2_tmp = orbitalMotion.f2E(oe2_cl_mean.f, oe2_cl_mean.e)
        oed_cl[i, 5] = orbitalMotion.E2M(
            E2_tmp, oe2_cl_mean.e) - orbitalMotion.E2M(E_tmp, oe_cl_mean.e)  # delta M
        for j in range(3, 6):
            while(oed_cl[i, j] > np.pi):
                oed_cl[i, j] = oed_cl[i, j] - 2*np.pi
            while(oed_cl[i, j] < -np.pi):
                oed_cl[i, j] = oed_cl[i, j] + 2*np.pi
    plt.plot(timeData, oed_cl[:, 0], label="da")
    plt.plot(timeData, oed_cl[:, 1], label="de")
    plt.plot(timeData, oed_cl[:, 2], label="di")
    plt.plot(timeData, oed_cl[:, 3], label="dOmega")
    plt.plot(timeData, oed_cl[:, 4], label="domega")
    plt.plot(timeData, oed_cl[:, 5], label="dM")
    plt.legend()
    plt.xlabel("time [orbit]")
    plt.ylabel("mean orbital element difference")
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)
    # equinoctial oe (figure2)
    plt.figure(2)
    oed_eq = np.empty((len(pos[:, 0]), 6))
    for i in range(0, len(pos[:, 0])):
        # spacecraft 1 (chief)
        oe_cl_osc = orbitalMotion.rv2elem(mu, pos[i, 1:4], vel[i, 1:4])
        oe_cl_mean = orbitalMotion.ClassicElements()
        orbitalMotion.clMeanOscMap(orbitalMotion.REQ_EARTH*1e3, orbitalMotion.J2_EARTH, oe_cl_osc, oe_cl_mean, -1)
        oe_eq_mean = orbitalMotion.EquinoctialElements()
        orbitalMotion.clElem2eqElem(oe_cl_mean, oe_eq_mean)
        # spacecraft 2 (deputy)
        oe2_cl_osc = orbitalMotion.rv2elem(mu, pos2[i, 1:4], vel2[i, 1:4])
        oe2_cl_mean = orbitalMotion.ClassicElements()
        orbitalMotion.clMeanOscMap(orbitalMotion.REQ_EARTH*1e3, orbitalMotion.J2_EARTH, oe2_cl_osc, oe2_cl_mean, -1)
        oe2_eq_mean = orbitalMotion.EquinoctialElements()
        orbitalMotion.clElem2eqElem(oe2_cl_mean, oe2_eq_mean)
        # calculate oed
        oed_eq[i, 0] = (oe2_eq_mean.a - oe_eq_mean.a)/oe_eq_mean.a  # delta a (normalized)
        oed_eq[i, 1] = oe2_eq_mean.P1 - oe_eq_mean.P1  # delta P1
        oed_eq[i, 2] = oe2_eq_mean.P2 - oe_eq_mean.P2  # delta P2
        oed_eq[i, 3] = oe2_eq_mean.Q1 - oe_eq_mean.Q1  # delta Q1
        oed_eq[i, 4] = oe2_eq_mean.Q2 - oe_eq_mean.Q2  # delta Q2
        oed_eq[i, 5] = oe2_eq_mean.l - oe_eq_mean.l  # delta l
        while(oed_eq[i, 5] > np.pi):
            oed_eq[i, 5] = oed_eq[i, 5] - 2*np.pi
        while(oed_eq[i, 5] < -np.pi):
            oed_eq[i, 5] = oed_eq[i, 5] + 2*np.pi
    plt.plot(timeData, oed_eq[:, 0], label="da")
    plt.plot(timeData, oed_eq[:, 1], label="dP1")
    plt.plot(timeData, oed_eq[:, 2], label="dP2")
    plt.plot(timeData, oed_eq[:, 3], label="dQ1")
    plt.plot(timeData, oed_eq[:, 4], label="dQ2")
    plt.plot(timeData, oed_eq[:, 5], label="dl")
    plt.legend()
    plt.xlabel("time [orbit]")
    plt.ylabel("mean orbital element difference")
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plt.figure()
    plt.plot(timeData, chiefAtt[:,1:4],label='Chief $\sigma_{BN}$')
    plt.plot(timeData, depAtt[:,1:4],label='Deputy $\sigma_{BN}$')
    plt.grid()
    plt.legend()
    plt.ylim([-1,1])
    plt.xlabel('Time')
    plt.ylabel('MRP Value')

    plt.figure()
    plt.plot(hillPos[:,1],hillPos[:,2])
    plt.grid()
    plt.legend()
    plt.xlabel('Hill X (m)')
    plt.ylabel('Hill Y (m)')

    plt.figure()
    plt.plot(fswTimeData, hillPosSens[:,1:3],label='Position Sensitivities')
    plt.grid()
    plt.legend()
    plt.xlabel('Time')
    plt.ylabel('Sensitivity value')

    plt.figure()
    plt.plot(fswTimeData, hillVelSens[:,1:3], label='Velocity Sensitivities')
    plt.grid()
    plt.legend()
    plt.xlabel('Time')
    plt.ylabel('Sensitivity value')

    plt.figure()
    plt.plot(timeData, rel_mrp_hist)
    plt.grid()
    plt.xlabel('Time')
    plt.ylabel('Relative MRP Value')

    if(show_plots):
        plt.show()
    plt.close("all")

if __name__ == "__main__":
    run(
        True,  # show_plots
        0.0, #   altitude offset (m)
        0.005, #  True anomaly offset (deg)
        1, #    Density multiplier (nondimensional)
        ctrlType='lqr'
    )
