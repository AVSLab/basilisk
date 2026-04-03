
.. _migratingToBsk2:

Migrating Basilisk Scripts from Version 1.X to 2.X
==================================================

Motivation
----------
This document discusses what user-facing changes occurred with the new messaging system in Basilisk version 2.0
and higher.  The string messaging system is replaced with a smarter message object system that

- prevents the user from connecting to the wrong message type in the C/C++ code
- warns the user if the Python scripts connects messages of the wrong type
- has much faster message recording functionality
- requires explicit message connection setup, i.e. no hidden implicit message naming

The overall goal is to create a new implementation is that is easier to use and understand, as well as faster
to execute.

However, nothing is for free.  Making these changes was not possible without breaking existing code.  This migration
help page outlines all the user-facing changes that have occurred.  This facilities the process of upgrading legacy
Basilisk python scripts to function with the new message system, etc.

For changes related to updating Basilisk C and C++ module code, see :ref:`migratingModuleToBsk2`.

Message Names
-------------
The module input and output messages are no longer specified through a name, and then subscribed to via an ID handle.
Rather, the messages have become smart objects that can be directly connected to the receiving module.  Thus,
while before we had ``stateOutMsgName`` and ``stateOutMsgId`` variables, now a single msg variable named
``stateOutMsg`` is used.   See :ref:`codingGuidelines` for more info on message naming.

Importing Basilisk architecture Packages
----------------------------------------
All message payload definitions are now stored contained in the ``message2`` package, providing a
unified interface to all flight software and simulation messages.  Thus, instead of importing::

    from Basilisk.fswAlgorithms import fswMessages

just import ``message2`` using::

    from Basilisk.architecture import messaging

Further, the ``sim_model``, ``alg_contain`` and ``MessagingAccess`` packages are now stored
within the new ``architecture`` library and imported using::

    from Basilisk.architecture import alg_contain
    from Basilisk.architecture import sim_model
    from Basilisk.architecture import MessagingAccess


Configuring Module Input/Output Message Names
---------------------------------------------
The Basilisk modules no longer use message names.  Rather, the message objects now direct are connected
to each other within Python.  Thus, the BSK module doesn't have to set message names from Python.

Module and Message Naming Changes
---------------------------------
Some early Basilisk modules and message names never complied with the naming guidelines in :ref:`codingGuidelines`.
The following list outlines any module or message naming changes that occurred in this upgrade process.  That is,
message naming is listed if it is outside the standard adjusted (see :ref:`codingGuidelines`) where
``descriptionOutMsgName`` becomes ``descriptionOutMsg``.    
This list makes it simple to see what naming will need to be changed.  Further, some modules have now a new name.
For example, the module ``spacecraftPlus`` is now called simply ``spacecraft``.  These module name changes are
also listed in this table below.

.. table:: Table of Module Message and Name Changes
    :widths: 40 30 30

    +---------------------------+---------------------------------+-----------------------------------+
    | Module Name               | Old Msg Name                    | New Msg Interface                 |
    +===========================+=================================+===================================+
    | attTrackingError          | ``outputDataMessageName``       | ``attGuidOutMsg``                 |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputNavMessageName``         | ``attNavInMsg``                   |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputRefMessageName``         | ``attRefInMsg``                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | bore_ang_calc  →          | ``StateString``                 |  ``scStateInMsg``                 |
    | ``boreAngCalc``           +---------------------------------+-----------------------------------+
    |                           | ``celBodyString``               | ``celBodyInMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``OutputDataString``            | ``angOutMsg``                     |
    +---------------------------+---------------------------------+-----------------------------------+
    | celestialTwoBodyPoint     | ``outputDataName``              | ``attRefOutMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputCelMessName``            | ``celBodyInMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputSecMessName``            | ``secCelBodyInMsg``               |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputNavDataName``            | ``transNavInMsg``                 |
    +---------------------------+---------------------------------+-----------------------------------+
    | cheby_pos_ephem  →        |                                 |                                   |
    | ``chebyPosEphem``         |                                 |                                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | clock_synch  →            |  ``clockOutputName``            |  ``clockOutMsg``                  |
    | ``simSynch``              |                                 |                                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | coarse_sun_sensor →       | ``outputConstellationMessage``  | ``constellationOutMsg``           |
    | ``coarseSunSensor``       +---------------------------------+-----------------------------------+
    |                           | ``cssConfigLogMsgName``         | ``cssConfigLogOutMsg``            |
    +---------------------------+---------------------------------+-----------------------------------+
    | cssComm                   | ``SensorListName``              | ``sensorListInMsg``               |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``OutputDataName``              | ``cssArrayOutMsg``                |
    +---------------------------+---------------------------------+-----------------------------------+
    | DataStorageUnitBase       | ``nodeDataUseMsgNames``         | ``nodeDataUseInMsgs``             |
    +---------------------------+---------------------------------+-----------------------------------+
    | dvAccumulation            | ``outputNavName``               | ``dvAcumOutMsg``                  |
    +---------------------------+---------------------------------+-----------------------------------+
    | dvAttEffect               | ``inputControlName``            | ``cmdTorqueBodyInMsg``            |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``outputDataName``              | ``thrOnTimeOutMsg``               |
    +---------------------------+---------------------------------+-----------------------------------+
    | dvExecuteGuidance         | ``outputDataName``              | ``burnExecOutMsg``                |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``outputThrName``               | ``thrCmdOutMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputBurnDataName``           | ``burnDataInMsg``                 |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputNavDataName``            | ``navDataInMsg``                  |
    +---------------------------+---------------------------------+-----------------------------------+
    | dvGuidance                | ``outputDataName``              | ``attRefOutMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputBurnDataName``           | ``burnDataInMsg``                 |
    +---------------------------+---------------------------------+-----------------------------------+
    | ephemeris_converter →     |                                 | ``ephemOutMsgs``                  |
    | ``ephemerisConverter``    +---------------------------------+-----------------------------------+
    |                           |                                 | ``spiceInMsgs``                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | ephem_Difference →        |                                 |                                   |
    | ``ephemDifference``       |                                 |                                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | ephem_nav_converter →     |                                 |                                   |
    | ``ephemNavConverter``     |                                 |                                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | faultDetection            | ``navMeasPrimaryMsgName``       | ``navMeasPrimaryInMsg``           |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``navMeasSecondaryMsgName``     | ``navMeasSecondaryInMsg``         |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``cameraConfigMsgName``         | ``cameraConfigInMsg``             |
    +---------------------------+---------------------------------+-----------------------------------+
    | GravBodyData              | ``bodyInMsgName``               | ``planetBodyInMsg``               |
    +                           +---------------------------------+-----------------------------------+
    |                           |                                 | ``planetName``                    |
    +---------------------------+---------------------------------+-----------------------------------+
    | headingSuKF               | ``cameraConfigMsgName``         | ``cameraConfigInMsg``             |
    +---------------------------+---------------------------------+-----------------------------------+
    | hillPoint                 | ``outputDataName``              | ``attRefOutMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputCelMessName``            | ``celBodyInMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputNavDataName``            | ``transNavInMsg``                 |
    +---------------------------+---------------------------------+-----------------------------------+
    | horizonOpNav              | ``cameraConfigMsgName``         | ``cameraConfigInMsg``             |
    +---------------------------+---------------------------------+-----------------------------------+
    | imuComm                   | ``InputDataName``               | ``imuComInMsg``                   |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``InputPropsName``              | ``vehConfigInMsg``                |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``OutputDataName``              | ``imuSensorOutMsg``               |
    +---------------------------+---------------------------------+-----------------------------------+
    | imu_sensor →              | ``InputStateMsg``               | ``scStateInMsg``                  |
    | ``imuSensor``             +---------------------------------+-----------------------------------+
    |                           | ``OutputDataMsg``               | ``sensorOutMsg``                  |
    +---------------------------+---------------------------------+-----------------------------------+
    | inertial3D                | ``outputDataName``              | ``attRefOutMsg``                  |
    +---------------------------+---------------------------------+-----------------------------------+
    | inertial3DSpin            | ``outputDataName``              | ``attRefOutMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputRefName``                | ``attRefInMsg``                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | lowPassFilterTorqueCommand| ``outputDataName``              | ``cmdTorqueOutMsg``               |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputDataName``               | ``cmdTorqueInMsg``                |
    +---------------------------+---------------------------------+-----------------------------------+
    | oe_state_ephem  →         |                                 |                                   |
    | ``oeStateEphem``          |                                 |                                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | orb_elem_convert  →       | ``StateString``                 | ``scStateInMsg``                  |
    + ``orbElemConvert``        +---------------------------------+-----------------------------------+
    |                           | ``StateString``                 | ``spiceStateInMsg``               |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``StateString``                 | ``elemInMsg``                     |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``OutputDataString``            | ``scStateOutMsg``                 |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``OutputDataString``            | ``spiceStateOutMsg``              |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``OutputDataString``            | ``elemOutMsg``                    |
    +---------------------------+---------------------------------+-----------------------------------+
    | magnetometer              | ``stateIntMsgName``             | ``stateInMsg``                    |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``magIntMsgName``               | ``magInMsg``                      |
    +---------------------------+---------------------------------+-----------------------------------+
    | MRP_Feedback →            | ``outputDataName``              | ``cmdTorqueOutMsg``               |
    + ``mrpFeedback``           +---------------------------------+-----------------------------------+
    |                           | ``inputGuidName``               | ``guidInMsg``                     |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputRWSpeedsName``           | ``rwSpeedsInMsg``                 |
    +---------------------------+---------------------------------+-----------------------------------+
    | MRP_PD →                  | ``outputDataName``              | ``cmdTorqueOutMsg``               |
    + ``mrpPD``                 +---------------------------------+-----------------------------------+
    |                           | ``inputGuidName``               | ``guidInMsg``                     |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputVehicleConfigDataName``  | ``vehConfigInMsg``                |
    +---------------------------+---------------------------------+-----------------------------------+
    | MRP_Steering →            | ``outputDataName``              | ``rateCmdOutMsg``                 |
    + ``mrpSteering``           +---------------------------------+-----------------------------------+
    |                           | ``inputGuidName``               | ``guidInMsg``                     |
    +---------------------------+---------------------------------+-----------------------------------+
    | navAggregate              | ``outputAttName``               | ``navAttOutMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``outputTransName``             | ``navTransOutMsg``                |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputNavName``                | ``navAttInMsg``                   |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputNavName``                | ``navTransInMsg``                 |
    +---------------------------+---------------------------------+-----------------------------------+
    | opNavPoint                | ``cameraConfigMsgName``         | ``cameraConfigInMsg``             |
    +---------------------------+---------------------------------+-----------------------------------+
    | pixelLineConverter        | ``cameraConfigMsgName``         | ``cameraConfigInMsg``             |
    +---------------------------+---------------------------------+-----------------------------------+
    | pixelLineBiasUKF          | ``cameraConfigMsgName``         | ``cameraConfigInMsg``             |
    +---------------------------+---------------------------------+-----------------------------------+
    | PRV_Steering →            | ``outputDataName``              | ``rateCmdOutMsg``                 |
    + ``prvSteering``           +---------------------------------+-----------------------------------+
    |                           | ``inputGuidName``               | ``guidInMsg``                     |
    +---------------------------+---------------------------------+-----------------------------------+
    | radiation_pressure →      |                                 |                                   |
    | ``radiationPressure``     |                                 |                                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | rasterManager             | ``AttStateOutMsgName``          | ``attStateOutMsg``                |
    +---------------------------+---------------------------------+-----------------------------------+
    | rateServoFullNonlinear    | ``outputDataName``              | ``cmdTorqueOutMsg``               |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputGuidName``               | ``guidInMsg``                     |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputRWSpeedsName``           | ``rwSpeedsInMsg``                 |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputRateSteeringName``       | ``rateSteeringInMsg``             |
    +---------------------------+---------------------------------+-----------------------------------+
    | reactionWheelStateEffector| ``OutputDataString``            | ``rwSpeedOutMsg``                 |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``InputCmds``                   | ``rwMotorCmdInMsg``               |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``rwOutMsgNames``               | ``rwOutMsgs``                     |
    +---------------------------+---------------------------------+-----------------------------------+
    | rwMotorTorque             | ``scMassStateOutMsgName``       | ``rwMotorTorqueOutMsg``           |
    +---------------------------+---------------------------------+-----------------------------------+
    | rwMotorTorque             | ``scMassStateOutMsgName``       | ``rwMotorTorqueOutMsg``           |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputVehControlName``         | ``vehControlInMsg``               |
    +---------------------------+---------------------------------+-----------------------------------+
    | PowerStorageBase          | ``nodePowerUseMsgNames``        | ``nodePowerUseInMsgs``            |
    +---------------------------+---------------------------------+-----------------------------------+
    | rwNullSpace               | ``inputRWCommands``             | ``rwMotorTorqueInMsg``            |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputRWSpeeds``               | ``rwSpeedsInMsg``                 |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputRWConfigData``           | ``rwConfigInMsg``                 |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``outputControlName``           | ``rwMotorTorqueOutMsg``           |
    +---------------------------+---------------------------------+-----------------------------------+
    | simpleDeadband            | ``outputDataName``              | ``attGuidOutMsg``                 |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputGuidName``               | ``guidInMsg``                     |
    +---------------------------+---------------------------------+-----------------------------------+
    | simple_nav →              | ``outputAttMessage``            | ``attOutMsg``                     |
    + ``simpleNav``             +---------------------------------+-----------------------------------+
    |                           | ``outputTransMessage``          | ``transOutMsg``                   |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputStateName``              | ``scStateInMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputSunName``                | ``sunStateInMsg``                 |
    +---------------------------+---------------------------------+-----------------------------------+
    | spacecraftDynamics →      |                                 |                                   |
    |  ``spacecraftSystem``     |                                 |                                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | spacecraftPlus →          | ``scMassStateOutMsgName``       | ``scMassOutMsg``                  |
    |  ``spacecraft``           |                                 |                                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | spaceToGroundTransmitter  | ``storageUnitMsgNames``         | ``storageUnitInMsgs``             |
    +                           +---------------------------------+-----------------------------------+
    |                           |``groundLocationAccessMsgNames`` | ``stAttOutMsg``                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | simpleTransmitter         | ``storageUnitMsgNames``         | ``storageUnitInMsgs``             |
    +---------------------------+---------------------------------+-----------------------------------+
    | spice_interface →         | ``outputTimePort``              | ``spiceTimeOutMsg``               |
    + ``spiceInterface``        +---------------------------------+-----------------------------------+
    |                           | ``planetNames``                 | ``planetStateOutMsgs``            |
    +---------------------------+---------------------------------+-----------------------------------+
    | star_tracker →            | ``inputStateMessage``           | ``scStateInMsg``                  |
    + ``starTracker``           +---------------------------------+-----------------------------------+
    |                           | ``outputStateMessage``          | ``sensorOutMsg``                  |
    +---------------------------+---------------------------------+-----------------------------------+
    | stComm                    | ``InputDataName``               | ``stSensorInMsg``                 |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``OutputDataName``              | ``stAttOutMsg``                   |
    +---------------------------+---------------------------------+-----------------------------------+
    | sunSafeACS                | ``outputDataName``              | ``cmdTorqueBodyInMsg``            |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputGuidName``               | ``guidInMsg``                     |
    +---------------------------+---------------------------------+-----------------------------------+
    | thrForceMapping           | ``outputDataName``              | ``thrForceCmdOutMsg``             |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputVehControlName``         | ``cmdTorqueInMsg``                |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputThrusterConfName``       | ``thrConfigInMsg``                |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputVehicleConfigDataName``  | ``vehConfigInMsg``                |
    +---------------------------+---------------------------------+-----------------------------------+
    | thrusterDynamicEffector   | ``InputCmds``                   | ``cmdsInMsg``                     |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``thrusterOutMsgNames``         | ``thrusterOutMsgs``               |
    +---------------------------+---------------------------------+-----------------------------------+
    | thrustRWDesat             | ``inputSpeedName``              | ``rwSpeedInMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputRWConfigData``           | ``rwConfigInMsg``                 |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputThrConfigName``          | ``thrConfigInMsg``                |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputMassPropsName``          | ``vecConfigInMsg``                |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``outputThrName``               | ``thrCmdOutMsg``                  |
    +---------------------------+---------------------------------+-----------------------------------+
    | vehicleConfigData         | ``outputPropsName``             | ``vecConfigOutMsg``               |
    +---------------------------+---------------------------------+-----------------------------------+
    | velocityPoint             | ``inputControlName``            | ``attRefOutMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputCelMessName``            | ``celBodyInMsg``                  |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``inputNavDataName``            | ``transNavInMsg``                 |
    +---------------------------+---------------------------------+-----------------------------------+
    | VSCMGStateEffector        | ``InputCmds``                   | ``cmdInMsg``                      |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``OutputDataString``            | ``speedOutMsg``                   |
    +                           +---------------------------------+-----------------------------------+
    |                           | ``vscmgOutMsgNames``            | ``vscmgOutMsgs``                  |
    +---------------------------+---------------------------------+-----------------------------------+


Setting a Basilisk Message from Python using the default C++ wrapper
--------------------------------------------------------------------
Import ``messages2`` to have access to all message definitions::

    from Basilisk.architecture import messaging

To create the message content of type ``ParticularMsgPayload``, first get a copy of the message structure using::

    msgData = messaging.ParticularMsgPayload()

Next, fill in ``msgData`` with the needed information.  The structure is always initialized to zero on creation.
When done, use the following command to create the Msg object and get a copy for other modules to subscribe to.::

    msg = messaging.ParticularMsg().write(msgData, time)

The ``time`` is the message write time in nano-seconds.  It is optional and defaults to 0.

If you want to just create a message from Python, but not write to it, you can use::

    msg = messaging.ParticularMsg()

This will create a zero'd message payload with a header that indicates it has never been written.  To write
to it at a later time you simply use::

    msg.write(msgData, time)

Note that stand-alone messages written in Python don't have a module ID.  The message module ID is thus set to 0.
If you want to specify another module ID, this can be done with::

    msg.write(msgData, time, moduleID)

Next consider a C++ module that needs to not write to it's own C++ wrapped output message
``module.someOutMsg``, but rather to a stand-alone C++ message ``standAloneMsg`` created in Python.
Thus the module output is re-directed to another message object.
This can be done very simply with replacing the module internal message with the external
message of the same type::

    module.somOutMsg = standAloneMsg

Any module ``nextModule.SomeInMsg`` that needs to read the output of ``module.someOutMsg`` can be setup to
read the output of the stand-alone message ``standAloneMsg`` instead using::

    nextModule.SomeInMsg.subscribeTo(standAloneMsg)

Setting a C-Wrapped Basilisk Message from Python
------------------------------------------------
In rare cases you might want to create a C-wrapped msg copy in Python.  The default should be to use the
C++ wrapped method shown above.  One scenario where a C-wrapped message might be needed is if you have
multiple C-modules that should write to the same output message.  In this case it is possible to create a
stand-alone C-wrapped message copy in Python and re-direct the module output message to write to this stand-alone
message instead.  As the C-module only calls C-code, in this situation the stand-alone message must have
a C-wrapped interface.  An example of this is shown in :ref:`scenarioAttitudeFeedback`.

Creating a C-Wrapped Message
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The C-wrapped message interface is imported using::

    from Basilisk.architecture import messaging

The message payload is still created as before, using for example::

    msgData = messaging.ParticularMsgPayload()

To create a C-wrapped message copy use::

    msg = messaging.ParticularMsg_C().init()

This step creates an instance of ``ParticularMsg_C`` and adds itself as an author so you can write to it.
Now you can write to the C-wrapped message as you with with C++ wrapped messages::

    msg.write(msgData)

If you want to create a C-wrapped message and write to it in one step, you can use::

    msg = messaging.ParticularMsg_C().init(msgData)

Redirecting Module Output to Stand-Alone C-Wrapped Message
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Next consider the scenario where you don't want to have a C module to write to it's own output message
``module.SomeOutMsg`` of type ``ParticularMsg``, but rather you want the module to write to a stand-alone
module message ``standAloneMsg`` of the same type.  This can be done with::

    messaging.ParticularMsg_C_addAuthor(module.someOutMsg, standAloneMsg)

Any module ``nextModule.SomeInMsg`` that needs to read the output of ``module.someOutMsg`` can be setup to
read the output of the stand-alone message ``standAloneMsg`` instead using::

    nextModule.SomeInMsg.subscribeTo(standAloneMsg)


Reading a Basilisk Message from Python
--------------------------------------
Assume ``bskObject`` is the Basilisk module created in Python.  To read an output message ``someOutMsg``
and print a variable ``someMsgVariable`` within this outpout message, you can use::

    msgCopy = bskObject.someOutMsg.read()
    print(msgCopy.someMsgVariable)

Connecting Output to Input Messages in Python
---------------------------------------------
Assume you have a message ``someMsg`` that you want to connect to another Basilisk module.  This message
can be a stand-alone message in Python, or a output message within a Basilisk module.  It doesn't matter if this
message ``someMsg`` is created in a C or C++ Basilisk module.

If you want to connect to the input message ``someInMsg`` of a C++ Basilisk module ``moduleObject``,
then you can use::

        moduleObject.someInMsg.subscribeTo(someMsg)

If you want to connect the input message ``someInMsg`` of a C wrapped Basilisk module ``moduleConfig``,
then you can use::

        moduleConfig.someInMsg.subscribeTo(someMsg)

It does not matter if these message interfaces are based in C or C++. The ``subscribeTo()`` method handles this
automatically.

Recorded Message Data
---------------------
The recording of messages is much simplified.  There are a few changes to note in the format of the recorded data.

Here is some sample code.  The only line required to create a message recorder module for the message is::

    attErrorRec = attErrorConfig.attGuidOutMsg.recorder()

This creates an object that can be added to a Basilisk task list through::

    scSim.AddModelToTask(simTaskName, attErrorRec)

The update rate of ``simTaskName`` controls the frequency at which this message is recorded.
If you want the message to be logged at a lower rate, but still keep the recorder module in the ``simTaskName`` task
queue, then you can set this with ``.recorder(value)``.  Here ``value`` is the minimum time period (ns) before
a message is recorded. If you want to set a desired number of data points to be recorded, then
you can use the ``samplingRatio()`` helper function to determine a minimum recording time::

    numDataPoints = 50
    samplingTime = unitTestSupport.samplingRatio(simulationTime, simulationTimeStep, numDataPoints)
    dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataRec)

.. caution::

    If you add the recorder module to another task running at a different frequency, be aware that the message
    time information is that of when the message was recorded, not when it is written.  Thus, in a
    case where the recording update rate is not an integer multiple of the simulation task rate
    the latest message data is recorded, but the time stamp is not when
    the message was written, but when it was recorded.  If you are logging orbital positions, your recorded
    position would thus be off relative to the time stamp.
    Recording the message when the module wrote the message ensures the
    recording time tag is in sync with the data.  The easiest way to get down-sampled data is to add the
    message recorder module to the same task that contains the module writing the message.  Ensure the
    recorder is called after the module such that the recorded module message is current for this time step.

If you are starting and stopping the simulation and need to update the minimum time interval before
messages are recorded, you can do this with::

    dataRec.updateTimeInterval(newSamplingTime)

That is it.  The data is now recorded into ``attErrorRec`` automatically during the simulation run.
In the new messaging system  the time information when the message is recorded
is no longer pre-pended in a first column, but rather provided as a
separate array accessed through ``datRec.times()``.  This means recording `N` time steps of a 3D vector no longer no longer
yields a `Nx4` array, but rather a `Nx3` array.

To record a particular variable `variableName` within the message you can use::

    varLog = dataRec.variableName

If ``variableName`` is a `N` dimensional vector of values, the ``varLog`` will be an `MxN` matrix of data
values where `M` is the number of recorded time steps.  You can use the typical python commands to select a subset
of this matrix.

Some plotting or value checking logic might have to be updated.
For example, to plot using recorded data use::

    for idx in range(3):
        plt.plot(attErrorRec.times() * macros.NANO2MIN, attErrorRec.sigma_BR[:, idx])

If you want to retrieve a vector of times when the message was written (i.e. recorded the ``.timeWritten()`` message
information), then you can do this through ``dataRec.timesWritten()``.  Depending on how ofter a message is
written by a module, and how often a message is recorded, these times can be different.

Running Basilisk Simulation
---------------------------
The method ``.InitializeSimulationAndDiscover()`` has been removed in favor of ``.InitializeSimulation()`` as
it did the regular simulation initialization and setup cross-process messaging using the old messaging system.  This
is no longer needed.  Thus, rather then having 2 methods to initialize a Basilisk simulation, only the regular
initializaiton method is now used.


Miscellaneous Changes
---------------------
If from Python you access ``#define`` values of ``macroDefinitions.h``, such as::

    simFswInterfaceMessages.MAX_EFF_CNT
    fswMessages.MAX_EFF_CNT

then you can now access these definitions using  ``messaging.i`` using::

    messaging.MAX_EFF_CNT



.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/7psCHvvrf4Q" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
