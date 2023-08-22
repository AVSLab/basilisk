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

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/Fux_UsZ5FR0" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

Demonstrates how to use :ref:`spiceInterface` to load a Spice kernel with the Hubble orbital
information and use the corresponding :ref:`transRefMsgPayload` output message
to specify the spacecraft motion.  With this approach the spacecraft attitude is still
dynamic and can achieve the same inertial pointing goal as setup in
:ref:`scenarioAttitudeFeedback`.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioSpiceSpacecraft.py

The simulation layout is
shown in the following illustration.  Note that here the ``spiceInterface`` module
must be executed prior to :ref:`spacecraft` as the
:ref:`transRefMsgPayload` input message must be properly setup before initializing
the spacecraft module which reads this in during ``Reset()``.

.. image:: /_images/static/test_scenarioSpiceSpacecraft.svg
   :align: center


When the simulation completes 2 plots are shown for the MRP attitude tracking error history,
as well as the Hubble inertial orbital coordinates with respect to the Earth.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

.. image:: /_images/Scenarios/scenarioSpiceSpacecraft1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSpiceSpacecraft2.svg
   :align: center



"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, simpleNav() and
#           mrpFeedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 19, 2016
#
import os

import numpy as np

np.set_printoptions(precision=16)

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import simpleNav

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import attTrackingError

# import message declarations
from Basilisk.architecture import messaging

# attempt to import vizard
from Basilisk.utilities import vizSupport


# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "Hubble"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # setup spice library for Earth ephemeris and Hubble states
    timeInitString = "2015 February 10, 00:00:00.0 TDB"
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)
    gravFactory.spiceObject.zeroBase = 'Earth'
    scNames = ["HUBBLE SPACE TELESCOPE"]
    gravFactory.spiceObject.addSpacecraftNames(messaging.StringVector(scNames))
    gravFactory.spiceObject.loadSpiceKernel("hst_edited.bsp", bskPath + '/supportData/EphemerisData/')

    # need spice to run before spacecraft module as it provides the spacecraft translational states
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)
    scSim.AddModelToTask(simTaskName, scObject)

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.K = 3.5
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    #
    # create simulation messages
    #
    # The MRP Feedback algorithm requires the vehicle configuration structure. This defines various spacecraft
    # related states such as the inertia tensor and the position vector between the primary Body-fixed frame
    # B origin and the center of mass (defaulted to zero).  The message payload is created through
    configData = messaging.VehicleConfigMsgPayload()
    configData.ISCPntB_B = I
    configDataMsg = messaging.VehicleConfigMsg().write(configData)

    #
    # connect the messages to the modules
    #
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(configDataMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    scObject.transRefInMsg.subscribeTo(gravFactory.spiceObject.transRefStateOutMsgs[0])

    #
    # Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    snLog = sNavObject.scStateInMsg.recorder(samplingTime)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    mrpLog = mrpControl.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, snLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, mrpLog)

    #
    #   set initial Spacecraft States
    #
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
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

    # unload custom Spice kernel
    gravFactory.unloadSpiceKernels()
    gravFactory.spiceObject.unloadSpiceKernel("hst_edited.bsp", bskPath + '/supportData/EphemerisData/')

    #
    #   plot the results
    #
    timeAxis = attErrorLog.times()
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attErrorLog.sigma_BR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, mrpLog.torqueRequestBody[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Control Torque $L_r$ [Nm]')
    pltName = fileName + "2"

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attErrorLog.omega_BR_B[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    plt.figure(4)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, snLog.r_BN_N[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial Position [km]')
    figureList[pltName] = plt.figure(4)

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
        True  # show_plots
    )
