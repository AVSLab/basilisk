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

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/w0XXmEm4Z4I" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

Discusses how to directly command the spacecraft orientation using the optional :ref:`attRefMsgPayload`
input message.  The benefit here is that the attitude dynamics is over-written to prescribe the orientation to
follow the given input message.  This can lead to must faster simulation times as larger integration
time steps can be taken.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioAttitudePrescribed.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.

.. image:: /_images/static/test_scenarioAttitudePrescribed.svg
   :align: center

In this scenario we use :ref:`attRefCorrection` to change the Hill pointing
reference message to align any desired set of body axes with the Hill frame.  The resulting output message
is connected to the optional :ref:`spacecraft` attitude input message to drive this orientation.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useAltBodyFrame = False

The default scenario shown has the ``useAltBodyFrame`` flag turned off.  This means that we seek
to align the body frame *B* with the Hill reference frame :math:`\cal R`.

.. image:: /_images/Scenarios/scenarioAttitudePrescribed10.svg
   :align: center

::

    show_plots = True, useAltBodyFrame = True

Here we apply a 90 degree rotation from the body frame B to the corrected body frame :math:`B_c`
and the spacecraft is prescribed to have a different orientation relative to the Hill frame.


.. image:: /_images/Scenarios/scenarioAttitudePrescribed11.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft() undergoing orbital motion subject to a
#           prescribed orientation.
# Author:   Hanspeter Schaub
# Creation Date:  July 1, 2021
#

import math
import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.fswAlgorithms import attRefCorrection
# import FSW Algorithm related support
from Basilisk.fswAlgorithms import hillPoint
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport

# import message declarations
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, useAltBodyFrame):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useAltBodyFrame (bool): Specify if the alternate body frame should be aligned with Hill frame.

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(3*60.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(60)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    #
    #   initialize Spacecraft States with initialization variables
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.1
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N


    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    #
    #   setup the FSW algorithm tasks
    #

    # setup hillPoint guidance module
    attGuidance = hillPoint.hillPoint()
    attGuidance.ModelTag = "hillPoint"
    attGuidance.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    # if you want to connect attGuidance.celBodyInMsg, then you need a planet ephemeris message of
    # type EphemerisMsgPayload.  In this simulation the input message is not connected to create an empty planet
    # ephemeris message which puts the earth at (0,0,0) origin with zero speed.
    scSim.AddModelToTask(simTaskName, attGuidance)

    # connect torque command to external torque effector
    if useAltBodyFrame:
        attRefCor = attRefCorrection.attRefCorrection()
        attRefCor.ModelTag = "attRefCor"
        scSim.AddModelToTask(simTaskName, attRefCor)
        attRefCor.sigma_BcB = [0.0, 0.0, math.tan(math.pi/8)]
        attRefCor.attRefInMsg.subscribeTo(attGuidance.attRefOutMsg)
        scObject.attRefInMsg.subscribeTo(attRefCor.attRefOutMsg)
    else:
        scObject.attRefInMsg.subscribeTo(attGuidance.attRefOutMsg)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, snAttLog)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              # , saveFile=fileName
                                              )

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataSigmaBN = snAttLog.sigma_BN

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeLineSet = snAttLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plt.figure(1)
    for idx in range(3):
        plt.plot(timeLineSet, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Inertial Attitude $\sigma_{B/N}$')
    figureList = {}
    pltName = fileName + "1" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(1)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        False  # useAltBodyFrame
    )
