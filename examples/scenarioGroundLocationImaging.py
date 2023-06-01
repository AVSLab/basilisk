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

This scenario demonstrates imaging a target on the surface of the Earth based on access and attitude error requirements.
Simulated data generated from this image is stored in an on-board storage device and then downlinked to a ground
station.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioGroundLocationImaging.py

The simulation uses :ref:`groundLocation` to create an output message with the desired ground target's inertial
position.  This is fed to the 2D pointing module :ref:`locationPointing` which directs the 3rd body axis to point
towards the target. Once the target is accessible per the :ref:`groundLocation` class' ``accessOutMsg`` and below the
prescribed attitude error from the :ref:`locationPointing` class' ``attGuidOutMsg``, the
:ref:`simpleInstrumentController` sends a ``deviceStatusOutMsg`` to the :ref:`simpleInstrument` to turn the imager on
for a single time step. The :ref:`simpleInstrument` sends this data to a :ref:`partitionedStorageUnit`. The data is
then downlinked using a :ref:`spaceToGroundTransmitter` once a ground station represented by another
:ref:`groundLocation` class is accessible.

A reset of the :ref:`groundLocation` and :ref:`simpleInstrumentController` for the purposes of switching to a new target
after the first one is imaged is also demonstrated.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

The following plots illustrate the 2D pointing error, access data, image commands, and level of the storage unit.

.. image:: /_images/Scenarios/scenarioGroundLocationImaging1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioGroundLocationImaging2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioGroundLocationImaging3.svg
   :align: center

.. image:: /_images/Scenarios/scenarioGroundLocationImaging4.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque(), simpleNav(), locationPoint(), groundLocation(),
#           simpleInstrumentController(), simpleInstrument(), partitionedStorageUnit(), and spaceToGroundTransmitter()
#           modules.  Will point a spacecraft axis at two Earth fixed locations and downlink the associated data.
# Author:   Adam Herrmann
# Creation Date:  May 11, 2021
#

import os

import matplotlib.pyplot as plt
import numpy as np
# import message declarations
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import locationPointing
# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import simpleInstrumentController
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import groundLocation
from Basilisk.simulation import partitionedStorageUnit
from Basilisk.simulation import simpleInstrument
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spaceToGroundTransmitter
# import simulation related support
from Basilisk.simulation import spacecraft
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import astroFunctions
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport

try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
def plot_attitude_error(timeLineSet, dataSigmaBR):
    """Plot the attitude result."""
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    vectorData = dataSigmaBR
    sNorm = np.array([np.linalg.norm(v) for v in vectorData])
    plt.plot(timeLineSet, sNorm,
             color=unitTestSupport.getLineColor(1, 3),
             )
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error Norm $|\sigma_{B/R}|$')
    ax.set_yscale('log')

    return


def plot_data_levels(timeLineSet, storageLevel, storedData):
    plt.figure(4)
    plt.plot(timeLineSet, storageLevel / 8E3, label='Data Unit Total Storage Level (KB)')
    plt.plot(timeLineSet, storedData[:, 0] / 8E3, label='Boulder Partition Level (KB)')
    plt.plot(timeLineSet, storedData[:, 1] / 8E3, label='Santiago Partition Level (KB)')
    plt.xlabel('Time (min)')
    plt.ylabel('Data Stored (KB)')
    plt.grid(True)
    plt.legend()

    return


def plot_device_status(timeLineSet, deviceStatus):
    plt.figure(3)
    plt.plot(timeLineSet, deviceStatus)
    plt.xlabel('Time [min]')
    plt.ylabel('Device Status')

    return


def plot_access(timeLineSet, hasAccess):
    plt.figure(2)
    plt.plot(timeLineSet, hasAccess)
    plt.xlabel('Time [min]')
    plt.ylabel('Imaging Target Access')

    return


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
    simulationTime = macros.min2nano(20.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.0)
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
    scSim.AddModelToTask(simTaskName, scObject, ModelPriority=100)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    #   initialize Spacecraft States with initialization variables
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = (6378 + 600)*1000.  # meters
    oe.e = 0.01
    oe.i = 63.3 * macros.D2R
    oe.Omega = 88.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 135.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    # use the input flag to determine which external torque should be applied
    # Note that all variables are initialized to zero.  Thus, not setting this
    # vector would leave it's components all zero for the simulation.
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject, ModelPriority=90)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject, ModelPriority=100)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # Create the initial imaging target
    imagingTarget = groundLocation.GroundLocation()
    imagingTarget.ModelTag = "ImagingTarget"
    imagingTarget.planetRadius = astroFunctions.E_radius*1e3
    imagingTarget.specifyLocation(np.radians(40.009971), np.radians(-105.243895), 1624)
    imagingTarget.minimumElevation = np.radians(10.)
    imagingTarget.maximumRange = 1e9
    imagingTarget.addSpacecraftToModel(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, imagingTarget, ModelPriority=100)

    # Create a ground station in Singapore
    singaporeStation = groundLocation.GroundLocation()
    singaporeStation.ModelTag = "SingaporeStation"
    singaporeStation.planetRadius = astroFunctions.E_radius*1e3
    singaporeStation.specifyLocation(np.radians(1.3521), np.radians(103.8198), 15)
    singaporeStation.minimumElevation = np.radians(5.)
    singaporeStation.maximumRange = 1e9
    singaporeStation.addSpacecraftToModel(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, singaporeStation, ModelPriority=100)

    # Create a "transmitter"
    transmitter = spaceToGroundTransmitter.SpaceToGroundTransmitter()
    transmitter.ModelTag = "transmitter"
    transmitter.nodeBaudRate = -1E5   # baud
    transmitter.packetSize = -8E6   # bits
    transmitter.numBuffers = 2
    transmitter.addAccessMsgToTransmitter(singaporeStation.accessOutMsgs[-1])
    scSim.AddModelToTask(simTaskName, transmitter, ModelPriority=99)

    # Create an instrument
    instrument = simpleInstrument.SimpleInstrument()
    instrument.ModelTag = "instrument1"
    instrument.nodeBaudRate = 8E6  # baud, assumes the instantaneous writing of a single image
    instrument.nodeDataName = "boulder"
    scSim.AddModelToTask(simTaskName, instrument, ModelPriority=90)

    # Create a partitionedStorageUnit and attach the instrument to it
    dataMonitor = partitionedStorageUnit.PartitionedStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageCapacity = 0.25*8E7  # bits
    dataMonitor.addDataNodeToModel(instrument.nodeDataOutMsg)
    dataMonitor.addDataNodeToModel(transmitter.nodeDataOutMsg)
    dataMonitor.addPartition("boulder")
    dataMonitor.addPartition("santiago")
    scSim.AddModelToTask(simTaskName, dataMonitor, ModelPriority=89)
    transmitter.addStorageUnitToTransmitter(dataMonitor.storageUnitDataOutMsg)

    #
    #   setup the FSW algorithm tasks
    #

    # setup Boulder pointing guidance module
    locPointConfig = locationPointing.locationPointingConfig()
    locPointWrap = scSim.setModelDataWrap(locPointConfig)
    locPointWrap.ModelTag = "locPoint"
    scSim.AddModelToTask(simTaskName, locPointWrap, locPointConfig, ModelPriority=99)
    locPointConfig.pHat_B = [0, 0, 1]
    locPointConfig.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    locPointConfig.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    locPointConfig.locationInMsg.subscribeTo(imagingTarget.currentGroundStateOutMsg)

    # setup the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig, ModelPriority=98)
    mrpControlConfig.guidInMsg.subscribeTo(locPointConfig.attGuidOutMsg)
    mrpControlConfig.K = 5.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # connect torque command to external torque effector
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    # setup the simpleInstrumentController module
    simpleInsControlConfig = simpleInstrumentController.simpleInstrumentControllerConfig()
    simpleInsControlConfig.attErrTolerance = 0.1
    simpleInsControlWrap = scSim.setModelDataWrap(simpleInsControlConfig)
    simpleInsControlWrap.ModelTag = "instrumentController"
    simpleInsControlConfig.attGuidInMsg.subscribeTo(locPointConfig.attGuidOutMsg)
    simpleInsControlConfig.locationAccessInMsg.subscribeTo(imagingTarget.accessOutMsgs[-1])
    scSim.AddModelToTask(simTaskName, simpleInsControlWrap, simpleInsControlConfig, ModelPriority=97)
    instrument.nodeStatusInMsg.subscribeTo(simpleInsControlConfig.deviceCmdOutMsg)

    #
    #   Setup data logging before the simulation is initialized
    #
    deviceLog = simpleInsControlConfig.deviceCmdOutMsg.recorder()
    mrpLog = mrpControlConfig.cmdTorqueOutMsg.recorder()
    attErrLog = locPointConfig.attGuidOutMsg.recorder()
    snAttLog = sNavObject.attOutMsg.recorder()
    snTransLog = sNavObject.transOutMsg.recorder()
    dataMonLog = dataMonitor.storageUnitDataOutMsg.recorder()
    transmitterLog = transmitter.nodeDataOutMsg.recorder()
    locationLog = imagingTarget.accessOutMsgs[-1].recorder()

    scSim.AddModelToTask(simTaskName, dataMonLog)
    scSim.AddModelToTask(simTaskName, mrpLog)
    scSim.AddModelToTask(simTaskName, attErrLog)
    scSim.AddModelToTask(simTaskName, snAttLog)
    scSim.AddModelToTask(simTaskName, snTransLog)
    scSim.AddModelToTask(simTaskName, deviceLog)
    scSim.AddModelToTask(simTaskName, transmitterLog)
    scSim.AddModelToTask(simTaskName, locationLog)

    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    configDataMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)
    mrpControlConfig.vehConfigInMsg.subscribeTo(configDataMsg)

    #
    # setup Vizard visualization elements
    #
    if vizFound:
        genericSensorHUD = vizInterface.GenericSensor()
        genericSensorHUD.r_SB_B = [0., 1., 1.]
        genericSensorHUD.fieldOfView.push_back(20.0 * macros.D2R)  # single value means a conic sensor
        genericSensorHUD.normalVector = [0., 0., 1.]
        genericSensorHUD.color = vizInterface.IntVector(vizSupport.toRGBA255("red", alpha=0.25))
        genericSensorHUD.label = "genSen1"
        cmdInMsg = messaging.DeviceCmdMsgReader()
        cmdInMsg.subscribeTo(simpleInsControlConfig.deviceCmdOutMsg)
        genericSensorHUD.genericSensorCmdInMsg = cmdInMsg

        transceiverHUD = vizInterface.Transceiver()
        transceiverHUD.r_SB_B = [0.23, 0., 1.38]
        transceiverHUD.fieldOfView = 40.0 * macros.D2R
        transceiverHUD.normalVector = [-1., 0., 1.]
        transceiverHUD.color = vizInterface.IntVector(vizSupport.toRGBA255("cyan"))
        transceiverHUD.label = "antenna"
        trInMsg = messaging.DataNodeUsageMsgReader()
        trInMsg.subscribeTo(transmitter.nodeDataOutMsg)
        transceiverHUD.transceiverStateInMsgs.push_back(trInMsg)

        hdDevicePanel = vizInterface.GenericStorage()
        hdDevicePanel.label = "Main Disk"
        hdDevicePanel.units = "bits"
        hdDevicePanel.color = vizInterface.IntVector(vizSupport.toRGBA255("blue") + vizSupport.toRGBA255("red"))
        hdDevicePanel.thresholds = vizInterface.IntVector([50])
        hdInMsg = messaging.DataStorageStatusMsgReader()
        hdInMsg.subscribeTo(dataMonitor.storageUnitDataOutMsg)
        hdDevicePanel.dataStorageStateInMsg = hdInMsg
        # if this scenario is to interface with the BSK Viz, uncomment the "saveFile" line
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                  # , saveFile=fileName
                                                  , genericSensorList=genericSensorHUD
                                                  , transceiverList=transceiverHUD
                                                  , genericStorageList=hdDevicePanel
                                                  )
        # the following command sets Viz settings for the first spacecraft in the simulation
        vizSupport.setInstrumentGuiSetting(viz,
                                           showGenericSensorLabels=True,
                                           showTransceiverLabels=True,
                                           showGenericStoragePanel=True
                                           )

        # Add the Boulder target
        vizSupport.addLocation(viz, stationName="Boulder Target"
                               , parentBodyName=earth.displayName
                               , r_GP_P=imagingTarget.r_LP_P_Init
                               , fieldOfView=np.radians(160.)
                               , color='pink'
                               , range=2000.0*1000  # meters
                               )

        # Add target line to first Boulder
        vizSupport.createTargetLine(viz, toBodyName="Boulder Target", lineColor="red")

        # Add the Santiago target
        vizSupport.addLocation(viz, stationName="Santiago Target"
                               , parentBodyName=earth.displayName
                               , r_GP_P=[[1761771.6422437236], [-5022201.882030934], [-3515898.6046771165]]
                               , fieldOfView=np.radians(160.)
                               , color='pink'
                               , range=2000.0*1000  # meters
                               )

        # Add the Santiago target
        vizSupport.addLocation(viz, stationName="Singapore Station"
                               , parentBodyName=earth.displayName
                               , r_GP_P=singaporeStation.r_LP_P_Init
                               , fieldOfView=np.radians(160.)
                               , color='green'
                               , range=2000.0*1000  # meters
                               )

        viz.settings.spacecraftSizeMultiplier = 1.5
        viz.settings.showLocationCommLines = 1
        viz.settings.showLocationCones = 1
        viz.settings.showLocationLabels = 1
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
    #   configure new ground location target in Santiago, Chile
    #
    imagingTarget.specifyLocation(np.radians(-33.4489), np.radians(-70.6693), 570)
    instrument.nodeDataName = "santiago"
    simpleInsControlConfig.imaged = 0

    # update targeting line to point to Santiago and be blue
    if vizFound:
        vizSupport.targetLineList[0].lineColor = vizSupport.toRGBA255('blue')
        vizSupport.targetLineList[0].toBodyName = "Santiago Target"
        vizSupport.updateTargetLineList(viz)

    #
    #   configure the new simulation stop time and execute sim
    #
    scSim.ConfigureStopTime(simulationTime + 3*simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataSigmaBR = attErrLog.sigma_BR
    storageLevel = dataMonLog.storageLevel
    storedDataName = dataMonLog.storedDataName
    storedData = dataMonLog.storedData
    deviceCmd = deviceLog.deviceCmd
    hasAccess = locationLog.hasAccess

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeLineSet = attErrLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plot_attitude_error(timeLineSet, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_access(timeLineSet, hasAccess)
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_device_status(timeLineSet, deviceCmd)
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plot_data_levels(timeLineSet, storageLevel, storedData)
    pltName = fileName + "4"
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