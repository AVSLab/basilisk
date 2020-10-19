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

import numpy as np
import matplotlib.pyplot as plt


from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import vizSupport

from Basilisk.simulation import spacecraftPlus, facetDragDynamicEffector, simple_nav, exponentialAtmosphere
from Basilisk.fswAlgorithms import hillStateConverter, hillToAttRef, hillPoint
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

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
    scObject.hub.mHub = 12.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    I = [10., 0., 0.,
         0., 9., 0.,
         0., 0., 8.]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    scNav = simple_nav.SimpleNav()
    scNav.inputStateName = scObject.scStateOutMsgName
    scNav.outputTransName = modelName + '_trans_nav_state'
    scNav.outputAttName = modelName + '_att_nav_state'

    dragArea = 1.0
    dragCoeff = 1.0
    normalVector = [0,0.707,0.707]
    panelLocation = [0,0,0]

    dragEffector = facetDragDynamicEffector.FacetDragDynamicEffector()
    dragEffector.addFacet(dragArea, dragCoeff, normalVector, panelLocation)
    scObject.addDynamicEffector(dragEffector)

    return scObject, dragEffector, scNav


def run(show_plots, altOffset, trueAnomOffset):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        useClassicElem (bool): Determines if classic orbital element is used
    """
    scSim = SimulationBaseClass.SimBaseClass()

    #   Configure simulation container and timestep
    simProcessName = "simProcess"
    dynTaskName = "dynTask"
    fswTaskName = "dynTask"
    simProcess = scSim.CreateNewProcess(simProcessName, 2)
    dynTimeStep = macros.sec2nano(15.0) #   Timestep to evaluate dynamics at
    dynTimeStep = macros.sec2nano(15.0) #   Timestep to evaluate dynamics at
    simProcess.addTask(scSim.CreateNewTask(dynTaskName, dynTimeStep))

    ##  Configure environmental parameters
    #   Gravity; includes 2-body plus J2.
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth'])
    gravBodies['earth'].isCentralBody = True
    gravBodies['earth'].useSphericalHarmParams = True
    simIncludeGravBody.loadGravFromFile(bskPath + '/supportData/LocalGravData/GGM03S.txt', gravBodies['earth'].spherHarm, 2)

    #   Density
    atmosphere = exponentialAtmosphere.ExponentialAtmosphere()
    atmosphere.ModelTag = 'exponential'
    atmosphere.planetRadius = orbitalMotion.REQ_EARTH*1e3   #   m
    atmosphere.scaleHeight = 8e3    # m
    atmosphere.baseDensity = 1.020 #    kg/m^3

    ##   Set up chief, deputy orbits:
    mu = gravFactory.gravBodies['earth'].mu
    chief_oe = orbitalMotion.ClassicElements()
    chief_oe.a = orbitalMotion.REQ_EARTH * 1e3 + 230e3 # meters
    chief_oe.e = 0.
    chief_oe.i = 60.
    chief_oe.Omega = 00.0 * macros.D2R
    chief_oe.omega = 0.0 * macros.D2R
    chief_oe.f = 180.*macros.D2R
    chief_rN, chief_vN = orbitalMotion.elem2rv(mu, chief_oe)

    dep_oe = copy.deepcopy(chief_oe)
    dep_oe.a += altOffset
    dep_oe.f += trueAnomOffset
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
    scSim.AddModelToTask(dynTaskName, chiefSc)
    scSim.AddModelToTask(dynTaskName, depSc)
    scSim.AddModelToTask(dynTaskName, chiefDrag)
    scSim.AddModelToTask(dynTaskName, depDrag)
    scSim.AddModelToTask(dynTaskName, chiefNav)
    scSim.AddModelToTask(dynTaskName, depNav)
#    scSim.AddModelToTask(dynTaskName, gravFactory.spiceObject)

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

    # hillToAtt guidance law
    depAttRef = hillToAttRef.HillToAttRef()
    depAttRef.ModelTag = 'dep_att_ref'
    depAttRef.hillStateInMsg = hillStateNavData.hillStateOutMsgName
    depAttRef.attStateInMsg = chiefNav.outputAttName
    depAttRef.attRefOutMsgName = 'dep_att_ref'
    depAttRef.gainMatrixVec = [[ [0.00000000e+00],  [0.00000000e+00], [0.00000000e+00], [0.00000000e+00], [0.00000000e+00], [0.00000000e+00]],
                               [ [0.00000000e+00],  [0.00000000e+00], [0.00000000e+00], [0.00000000e+00], [0.00000000e+00], [0.00000000e+00]],
                               [ [1.28982586e-02], [-9.99999987e-05], [0.00000000e+00], [1.90838094e-01], [5.58803922e+00], [0.00000000e+00]]]
    depSc.attRefInMsgName = depAttRef.attRefOutMsgName

    scSim.AddModelToTask(fswTaskName, chiefAttRefWrap, chiefAttRefData)
    scSim.AddModelToTask(fswTaskName, hillStateNavWrap, hillStateNavData)
    scSim.AddModelToTask(fswTaskName, depAttRefWrap, defAttRefData)

    # ----- log ----- #
    orbit_period = 2*np.pi/np.sqrt(mu/oe.a**3)
    simulationTime = orbit_period*20
    simulationTime = macros.sec2nano(simulationTime)
    numDataPoints = 1000
    samplingTime = simulationTime // (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(scObject2.scStateOutMsgName, samplingTime)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, dynProcessName, gravBodies=gravFactory,
                                              # saveFile=fileName,
                                              scName=[scObject.ModelTag, scObject2.ModelTag])

    # ----- execute sim ----- #
    scSim.InitializeSimulationAndDiscover()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # ----- pull ----- #
    pos = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', list(range(3)))
    vel = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', list(range(3)))
    pos2 = scSim.pullMessageLogData(scObject2.scStateOutMsgName + '.r_BN_N', list(range(3)))
    vel2 = scSim.pullMessageLogData(scObject2.scStateOutMsgName + '.v_BN_N', list(range(3)))
    timeData = pos[:, 0]*macros.NANO2SEC/orbit_period

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
    pltName = fileName + "1" + str(int(useClassicElem))
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
    pltName = fileName + "2" + str(int(useClassicElem))
    figureList[pltName] = plt.figure(2)

    if(show_plots):
        plt.show()
    plt.close("all")

    return pos, vel, pos2, vel2, numDataPoints, figureList


if __name__ == "__main__":
    run(
        True,  # show_plots
        10.0e3, #   altitude offset
        0.01 #  True anomaly offset
    )
