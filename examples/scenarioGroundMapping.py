#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario demonstrates mapping several points on the surface of the Earth.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioGroundMapping.py

The simulation uses :ref:`groundMapping` to check if a spacecraft has access to any of the mapping points defined on the
surface of the Earth. For each mapping point, a :ref:`accessMsgPayload` is output to the :ref:`mappingInstrument` module,
which images the mapping points and sends the simulated data to a :ref:`partitionedStorageUnit`. While this is not
representative of the total amount of data collected during mapping, it does store which points have and have not been
mapped, which is useful for many operations problems. It is suggested that the user adds a :ref:`simpleInstrument` and
:ref:`simpleStorageUnit` if they desire to realistically capture how much data was actually collected.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

The following plots illustrate the attitude error of the spacecraft, a 3D view of the spacecraft trajectory and the mapping
points expressed in the planet-centered, planet-fixed frame, and the number of points stored in the data buffer.

.. image:: /_images/Scenarios/scenarioGroundMapping1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioGroundMapping2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioGroundMapping3.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the groundMapping(), mappingInstrument(), partitionedStorageUnit() modules.
# Author:   Adam Herrmann
# Creation Date:  April 18th, 2022
#

import math as m
import os

import matplotlib.pyplot as plt
import numpy as np
# import message declarations
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import locationPointing
# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import groundMapping
from Basilisk.simulation import mappingInstrument
from Basilisk.simulation import partitionedStorageUnit
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import astroFunctions
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import planetStates
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


def plot_map_progress(mapping_points, map_access, sc_pos_PCPF):
    fig = plt.figure(2)
    ax = plt.axes(projection='3d')
    ax.plot(sc_pos_PCPF[:,0]/1000, sc_pos_PCPF[:,1]/1000, sc_pos_PCPF[:,2]/1000, '.')
    ax.plot(mapping_points[map_access,0]/1000, mapping_points[map_access,1]/1000, mapping_points[map_access,2]/1000, '.', c='g')
    ax.set_xlabel('X [km]')
    ax.set_ylabel('Y [km]')
    ax.set_zlabel('Z [km]')

    return


def plot_stored_points(timeLineSet, stored_points):
    fig = plt.figure(3)
    plt.plot(timeLineSet, stored_points)
    plt.xlabel('Time [min]')
    plt.ylabel('Stored Points')

    return


def generate_mapping_points(num_points, radius):
    """Generates a number of mapping points on the surface of the body using a Fibonacci sphere
       Algorithm from:
       https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere"""

    points = []
    phi = m.pi * (3. - m.sqrt(5.))  # golden angle in radians

    for i in range(num_points):
        y = 1 - (i / float(num_points - 1)) * 2  # y goes from 1 to -1
        r = m.sqrt(1 - y * y)  # radius at y

        theta = phi * i  # golden angle increment

        x = m.cos(theta) * r
        z = m.sin(theta) * r

        points.append(radius*np.array([x, y, z]))

    return np.array(points)


def run(show_plots, useCentral):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useCentral (bool): Flag if the Earth is the central body or not
    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(5.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # Add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # Clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # Setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = useCentral  # ensure this is the central gravitational body
    mu = earth.mu

    timeInitString = '2020 MAY 21 18:28:03 (UTC)'
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/', timeInitString)
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject, ModelPriority=100)

    # Attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = (6378 + 2000)*1000.  # meters
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

    if useCentral:
        scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
        scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N
    else:
        planetPosition, planetVelocity = planetStates.planetPositionVelocity('EARTH', timeInitString)
        scObject.hub.r_CN_NInit = rN + np.array(planetPosition)
        scObject.hub.v_CN_NInit = vN + np.array(planetVelocity)

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject, ModelPriority=95)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject, ModelPriority=99)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # Generate the mapping points
    N = 500
    mapping_points = generate_mapping_points(N, astroFunctions.E_radius*1e3)

    # Create the ground mapping module
    groundMap = groundMapping.GroundMapping()
    groundMap.ModelTag = "groundMapping"
    for map_idx in range(N):
        groundMap.addPointToModel(mapping_points[map_idx,:])
    groundMap.minimumElevation = np.radians(45.)
    groundMap.maximumRange = 1e9
    groundMap.cameraPos_B = [0, 0, 0]
    groundMap.nHat_B = [0, 0, 1]
    groundMap.halfFieldOfView = np.radians(22.5)
    groundMap.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    groundMap.planetInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    scSim.AddModelToTask(simTaskName, groundMap, ModelPriority=1)

    # Create the mapping instrument
    mapInstrument = mappingInstrument.MappingInstrument()
    mapInstrument.nodeBaudRate = 1
    for map_idx in range(N):
        mapInstrument.addMappingPoint(groundMap.accessOutMsgs[map_idx], str(map_idx))
    scSim.AddModelToTask(simTaskName, mapInstrument, ModelPriority=2)

    # Create the partitioned storage unit
    storageUnit = partitionedStorageUnit.PartitionedStorageUnit()
    storageUnit.ModelTag = "storageUnit"
    storageUnit.storageCapacity = 8E9  # bits (1 GB)
    for map_idx in range(N):
        storageUnit.addDataNodeToModel(mapInstrument.dataNodeOutMsgs[map_idx])
        storageUnit.addPartition(str(map_idx))
    scSim.AddModelToTask(simTaskName, storageUnit, ModelPriority=3)

    # Create the ephemeris converter module
    ephemConverter = ephemerisConverter.EphemerisConverter()
    ephemConverter.ModelTag = "ephemConverter"
    ephemConverter.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[0])
    scSim.AddModelToTask(simTaskName, ephemConverter, ModelPriority=98)

    # Setup nadir pointing guidance module
    locPointConfig = locationPointing.locationPointingConfig()
    locPointWrap = scSim.setModelDataWrap(locPointConfig)
    locPointWrap.ModelTag = "locPoint"
    scSim.AddModelToTask(simTaskName, locPointWrap, locPointConfig, ModelPriority=97)
    locPointConfig.pHat_B = [0, 0, 1]
    locPointConfig.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    locPointConfig.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    locPointConfig.celBodyInMsg.subscribeTo(ephemConverter.ephemOutMsgs[0])

    # Setup the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "MRP_Feedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig, ModelPriority=96)
    mrpControlConfig.guidInMsg.subscribeTo(locPointConfig.attGuidOutMsg)
    mrpControlConfig.K = 5.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # connect torque command to external torque effector
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    configDataMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)
    mrpControlConfig.vehConfigInMsg.subscribeTo(configDataMsg)

    # Setup data logging before the simulation is initialized
    mrpLog = mrpControlConfig.cmdTorqueOutMsg.recorder()
    attErrLog = locPointConfig.attGuidOutMsg.recorder()
    snAttLog = sNavObject.attOutMsg.recorder()
    snTransLog = sNavObject.transOutMsg.recorder()
    scLog = scObject.scStateOutMsg.recorder()
    planetLog = gravFactory.spiceObject.planetStateOutMsgs[0].recorder()
    storageLog = storageUnit.storageUnitDataOutMsg.recorder()

    # Setup the logging for the mapping locations
    mapLog = []
    for idx in range(0, N):
        mapLog.append(groundMap.accessOutMsgs[idx].recorder())
        scSim.AddModelToTask(simTaskName, mapLog[idx])

    # Add all of the logs to the task
    scSim.AddModelToTask(simTaskName, mrpLog)
    scSim.AddModelToTask(simTaskName, attErrLog)
    scSim.AddModelToTask(simTaskName, snAttLog)
    scSim.AddModelToTask(simTaskName, snTransLog)
    scSim.AddModelToTask(simTaskName, scLog)
    scSim.AddModelToTask(simTaskName, planetLog)
    scSim.AddModelToTask(simTaskName, storageLog)

    # setup Vizard support
    if vizSupport.vizFound:
        genericSensorHUD = vizInterface.GenericSensor()
        genericSensorHUD.r_SB_B = [0., 0., 0.]
        genericSensorHUD.fieldOfView.push_back(45 * macros.D2R)  # single value means a conic sensor
        genericSensorHUD.normalVector = [0., 0., 1.]
        genericSensorHUD.color = vizInterface.IntVector(vizSupport.toRGBA255("red", alpha=0.25))
        genericSensorHUD.label = "genSen1"

        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                  # , saveFile=fileName
                                                  , genericSensorList=genericSensorHUD
                                                  )
        # the following command sets Viz settings for the first spacecraft in the simulation
        vizSupport.setInstrumentGuiSetting(viz,
                                           showGenericSensorLabels=True,
                                           )

    # Need to call the self-init and cross-init methods
    scSim.InitializeSimulation()

    # Set the simulation time.
    scSim.ConfigureStopTime(macros.hour2nano(1))

    # Begin the simulation time run set above
    scSim.ExecuteSimulation()

    # Retrieve the logged data
    dataSigmaBR = attErrLog.sigma_BR
    sc_pos_N = scLog.r_BN_N
    dcms_PN = planetLog.J20002Pfix
    planet_pos_N = planetLog.PositionVector

    timeLineSet = attErrLog.times() * macros.NANO2MIN

    # Compute the PCPF position of the spacecraft
    sc_pos_PCPF = np.zeros((len(sc_pos_N), 3))
    for idx in range(0, len(sc_pos_N)):
        sc_pos_PCPF[idx,:] = np.matmul(dcms_PN[idx,:,:], sc_pos_N[idx,:] - planet_pos_N[idx,:])

    # Retrieve the access messages
    map_access = np.zeros(N, dtype=bool)
    for idx in range(0, N):
        access = mapLog[idx].hasAccess
        if sum(access):
            map_access[idx] = 1

    storedData = storageLog.storedData
    stored_points = np.zeros(len(timeLineSet))
    for idx in range(0, len(timeLineSet)):
        stored_points[idx] = np.count_nonzero(storedData[idx,:])

    # Plot the results
    plt.close("all")  # clears out plots from earlier test runs

    plot_attitude_error(timeLineSet, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_map_progress(mapping_points, map_access, sc_pos_PCPF)
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_stored_points(timeLineSet, stored_points)
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

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
        True
    )
