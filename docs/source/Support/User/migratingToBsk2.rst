
.. _migratingToBsk2:

Migrating Basilisk Scripts from Version 1.X to 2.X
==================================================

Motivation
----------
This document discusses what user-facing changes occurred with the new messaging system in Basilisk version 2.0
and higher.  The string messaging system is replaced with a smarter message object system that

- prevents the user from connecting to the wrong message type in the C/C++ code
- warns the user if the Python scripts connects messages of the wrong type
- has much faster logging functionality
- requires explicit message connection setup, i.e. no hidden implicit message naming

The overall goal is to create a new implementation is that is easier to use and understand, as well as faster
to execute.

However, nothing is for free.  Making these changes was not possible without breaking existing code.  This migration
help page outlines all the user-facing changes that have occurred.  This facilities the process of upgrading legacy
Basilisk python scripts to function with the new message system, etc.

For changes related to updating Basilisk C and C++ module code, see :ref:`migratingBskModuleToBsk2`.

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

    from Basilisk.architecture import messaging2

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
This list makes it simple to see what naming will need to be changed.

.. table:: Table of Module Message and Name Changes
    :widths: 40 30 30

    +---------------------------+-------------------------------+-----------------------------------+
    | Module Name               | Old Msg Name                  | New Msg Interface                 |
    +===========================+===============================+===================================+
    | attTrackingError          | ``outputDataMessageName``     | ``attGuidOutMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputNavMessageName``       | ``attNavInMsg``                   |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRefMessageName``       | ``attRefInMsg``                   |
    +---------------------------+-------------------------------+-----------------------------------+
    | bore_ang_calc  →          | ``StateString``               |  ``scStateInMsg``                 |
    | ``boreAngCalc``           +-------------------------------+-----------------------------------+
    |                           | ``celBodyString``             | ``celBodyInMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``OutputDataString``          | ``angOutMsg``                     |
    +---------------------------+-------------------------------+-----------------------------------+
    | celestialTwoBodyPoint     | ``outputDataName``            | ``attRefOutMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputCelMessName``          | ``celBodyInMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputSecMessName``          | ``secCelBodyInMsg``               |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputNavDataName``          | ``transNavInMsg``                 |
    +---------------------------+-------------------------------+-----------------------------------+
    | cheby_pos_ephem  →        |                               |                                   |
    | ``chebyPosEphem``         |                               |                                   |
    +---------------------------+-------------------------------+-----------------------------------+
    | cssComm                   | ``SensorListName``            | ``sensorListInMsg``               |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``OutputDataName``            | ``cssArrayOutMsg``                |
    +---------------------------+-------------------------------+-----------------------------------+
    | dvAccumulation            | ``outputNavName``             | ``dvAcumOutMsg``                  |
    +---------------------------+-------------------------------+-----------------------------------+
    | dvAttEffect               | ``inputControlName``          | ``cmdTorqueBodyInMsg``            |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``outputDataName``            | ``thrOnTimeOutMsg``               |
    +---------------------------+-------------------------------+-----------------------------------+
    | dvExecuteGuidance         | ``outputDataName``            | ``burnExecOutMsg``                |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``outputThrName``             | ``thrCmdOutMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputBurnDataName``         | ``burnDataInMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputNavDataName``          | ``navDataInMsg``                  |
    +---------------------------+-------------------------------+-----------------------------------+
    | dvGuidance                | ``outputDataName``            | ``attRefOutMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputBurnDataName``         | ``burnDataInMsg``                 |
    +---------------------------+-------------------------------+-----------------------------------+
    | ephem_Difference →        |                               |                                   |
    | ``ephemDifference``       |                               |                                   |
    +---------------------------+-------------------------------+-----------------------------------+
    | ephem_nav_converter →     |                               |                                   |
    | ``ephemNavConverter``     |                               |                                   |
    +---------------------------+-------------------------------+-----------------------------------+
    | faultDetection            | ``navMeasPrimaryMsgName``     | ``navMeasPrimaryInMsg``           |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``navMeasSecondaryMsgName``   | ``navMeasSecondaryInMsg``         |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``cameraConfigMsgName``       | ``cameraConfigInMsg``             |
    +---------------------------+-------------------------------+-----------------------------------+
    | GravBodyData              | ``bodyInMsgName``             | ``planetBodyInMsg``               |
    +                           +-------------------------------+-----------------------------------+
    |                           |                               | ``planetName``                    |
    +---------------------------+-------------------------------+-----------------------------------+
    | headingSuKF               | ``cameraConfigMsgName``       | ``cameraConfigInMsg``             |
    +---------------------------+-------------------------------+-----------------------------------+
    | hillPoint                 | ``outputDataName``            | ``attRefOutMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputCelMessName``          | ``celBodyInMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputNavDataName``          | ``transNavInMsg``                 |
    +---------------------------+-------------------------------+-----------------------------------+
    | horizonOpNav              | ``cameraConfigMsgName``       | ``cameraConfigInMsg``             |
    +---------------------------+-------------------------------+-----------------------------------+
    | imuComm                   | ``InputDataName``             | ``imuComInMsg``                   |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``InputPropsName``            | ``vehConfigInMsg``                |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``OutputDataName``            | ``imuSensorOutMsg``               |
    +---------------------------+-------------------------------+-----------------------------------+
    | inertial3D                | ``outputDataName``            | ``attRefOutMsg``                  |
    +---------------------------+-------------------------------+-----------------------------------+
    | inertial3DSpin            | ``outputDataName``            | ``attRefOutMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRefName``              | ``attRefInMsg``                   |
    +---------------------------+-------------------------------+-----------------------------------+
    | lowPassFilterTorqueCommand| ``outputDataName``            | ``cmdTorqueOutMsg``               |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputDataName``             | ``cmdTorqueInMsg``                |
    +---------------------------+-------------------------------+-----------------------------------+
    | oe_state_ephem  →         |                               |                                   |
    | ``oeStateEphem``          |                               |                                   |
    +---------------------------+-------------------------------+-----------------------------------+
    | orb_elem_convert  →       | ``StateString``               | ``scStateInMsg``                  |
    + ``orbElemConvert``        +-------------------------------+-----------------------------------+
    |                           | ``StateString``               | ``spiceStateInMsg``               |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``StateString``               | ``elemInMsg``                     |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``OutputDataString``          | ``scStateOutMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``OutputDataString``          | ``spiceStateOutMsg``              |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``OutputDataString``          | ``elemOutMsg``                    |
    +---------------------------+-------------------------------+-----------------------------------+
    | MRP_Feedback →            | ``outputDataName``            | ``cmdTorqueOutMsg``               |
    + ``mrpFeedback``           +-------------------------------+-----------------------------------+
    |                           | ``inputGuidName``             | ``guidInMsg``                     |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRWSpeedsName``         | ``rwSpeedsInMsg``                 |
    +---------------------------+-------------------------------+-----------------------------------+
    | MRP_PD →                  | ``outputDataName``            | ``cmdTorqueOutMsg``               |
    + ``mrpPD``                 +-------------------------------+-----------------------------------+
    |                           | ``inputGuidName``             | ``guidInMsg``                     |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputVehicleConfigDataName``| ``vehConfigInMsg``                |
    +---------------------------+-------------------------------+-----------------------------------+
    | MRP_Steering →            | ``outputDataName``            | ``rateCmdOutMsg``                 |
    + ``mrpSteering``           +-------------------------------+-----------------------------------+
    |                           | ``inputGuidName``             | ``guidInMsg``                     |
    +---------------------------+-------------------------------+-----------------------------------+
    | navAggregate              | ``outputAttName``             | ``navAttOutMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``outputTransName``           | ``navTransOutMsg``                |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputNavName``              | ``navAttInMsg``                   |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputNavName``              | ``navTransInMsg``                 |
    +---------------------------+-------------------------------+-----------------------------------+
    | opNavPoint                | ``cameraConfigMsgName``       | ``cameraConfigInMsg``             |
    +---------------------------+-------------------------------+-----------------------------------+
    | pixelLineConverter        | ``cameraConfigMsgName``       | ``cameraConfigInMsg``             |
    +---------------------------+-------------------------------+-----------------------------------+
    | pixelLineBiasUKF          | ``cameraConfigMsgName``       | ``cameraConfigInMsg``             |
    +---------------------------+-------------------------------+-----------------------------------+
    | PRV_Steering →            | ``outputDataName``            | ``rateCmdOutMsg``                 |
    + ``prvSteering``           +-------------------------------+-----------------------------------+
    |                           | ``inputGuidName``             | ``guidInMsg``                     |
    +---------------------------+-------------------------------+-----------------------------------+
    | radiation_pressure →      |                               |                                   |
    | ``radiationPressure``     |                               |                                   |
    +---------------------------+-------------------------------+-----------------------------------+
    | rasterManager             | ``AttStateOutMsgName``        | ``attStateOutMsg``                |
    +---------------------------+-------------------------------+-----------------------------------+
    | rateServoFullNonlinear    | ``outputDataName``            | ``cmdTorqueOutMsg``               |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputGuidName``             | ``guidInMsg``                     |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRWSpeedsName``         | ``rwSpeedsInMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRateSteeringName``     | ``rateSteeringInMsg``             |
    +---------------------------+-------------------------------+-----------------------------------+
    | reactionWheelStateEffector| ``OutputDataString``          | ``rwSpeedOutMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``InputCmds``                 | ``rwMotorCmdInMsg``               |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``rwOutMsgNames``             | ``rwOutMsgs``                     |
    +---------------------------+-------------------------------+-----------------------------------+
    | rwMotorTorque             | ``scMassStateOutMsgName``     | ``rwMotorTorqueOutMsg``           |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputVehControlName``       | ``vehControlInMsg``               |
    +---------------------------+-------------------------------+-----------------------------------+
    | rwMotorVoltage            | ``inputRWSpeedsInMsgName``    | ``rwSpeedInMsg``                  |
    +---------------------------+-------------------------------+-----------------------------------+
    | rwNullSpace               | ``inputRWCommands``           | ``rwMotorTorqueInMsg``            |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRWSpeeds``             | ``rwSpeedsInMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRWConfigData``         | ``rwConfigInMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``outputControlName``         | ``rwMotorTorqueOutMsg``           |
    +---------------------------+-------------------------------+-----------------------------------+
    | simpleDeadband            | ``outputDataName``            | ``attGuidOutMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputGuidName``             | ``guidInMsg``                     |
    +---------------------------+-------------------------------+-----------------------------------+
    | simple_nav →              | ``outputAttMessage``          | ``attOutMsg``                     |
    + ``simpleNav``             +-------------------------------+-----------------------------------+
    |                           | ``outputTransMessage``        | ``transOutMsg``                   |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputStateName``            | ``scStateInMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputSunName``              | ``sunStateInMsg``                 |
    +---------------------------+-------------------------------+-----------------------------------+
    | spacecraftPlus            | ``scMassStateOutMsgName``     | ``scMassOutMsg``                  |
    +---------------------------+-------------------------------+-----------------------------------+
    | spice_interface →         | ``outputTimePort``            | ``spiceTimeOutMsg``               |
    + ``spiceInterface``        +-------------------------------+-----------------------------------+
    |                           | ``planetNames``               | ``planetStateOutMsgs``            |
    +---------------------------+-------------------------------+-----------------------------------+
    | stComm                    | ``InputDataName``             | ``stSensorInMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``OutputDataName``            | ``stAttOutMsg``                   |
    +---------------------------+-------------------------------+-----------------------------------+
    | sunSafeACS                | ``outputDataName``            | ``cmdTorqueBodyInMsg``            |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputGuidName``             | ``guidInMsg``                     |
    +---------------------------+-------------------------------+-----------------------------------+
    | thrForceMapping           | ``outputDataName``            | ``thrForceCmdOutMsg``             |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputVehControlName``       | ``cmdTorqueInMsg``                |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputThrusterConfName``     | ``thrConfigInMsg``                |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputVehicleConfigDataName``| ``vehConfigInMsg``                |
    +---------------------------+-------------------------------+-----------------------------------+
    | thrusterDynamicEffector   | ``InputCmds``                 | ``cmdsInMsg``                     |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``thrusterOutMsgNames``       | ``thrusterOutMsgs``               |
    +---------------------------+-------------------------------+-----------------------------------+
    | thrustRWDesat             | ``inputSpeedName``            | ``rwSpeedInMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRWConfigData``         | ``rwConfigInMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputThrConfigName``        | ``thrConfigInMsg``                |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputMassPropsName``        | ``vecConfigInMsg``                |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``outputThrName``             | ``thrCmdOutMsg``                  |
    +---------------------------+-------------------------------+-----------------------------------+
    | vehicleConfigData         | ``outputPropsName``           | ``vecConfigOutMsg``               |
    +---------------------------+-------------------------------+-----------------------------------+
    | velocityPoint             | ``inputControlName``          | ``attRefOutMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputCelMessName``          | ``celBodyInMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputNavDataName``          | ``transNavInMsg``                 |
    +---------------------------+-------------------------------+-----------------------------------+
    | VSCMGStateEffector        | ``InputCmds``                 | ``cmdInMsg``                      |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``OutputDataString``          | ``speedOutMsg``                   |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``vscmgOutMsgNames``          | ``vscmgOutMsgs``                  |
    +---------------------------+-------------------------------+-----------------------------------+


Setting a Basilisk Message from Python
--------------------------------------
Import ``messages2`` to have access to all message definitions::

    from Basilisk.simulation import messaging2

To create the message content of type ``ParticularMsgPayload``, first get a copy of the message structure using::

    msgData = messaging2.ParticularMsgPayload()

Next, fill in ``msgData`` with the needed information.  The structure is always initialized to zero on creation.
When done, use the following command to create the Msg object and get a copy for other modules to subscribe to.::

    msg = messaging2.ParticularMsg().write(msgData, time)

The ``time`` is the message write time in nano-seconds.  It is optional and defaults to 0.

If you want to just create a message from Python, but not write to it, you can use::

    msg = messaging2.ParticularMsg()

This will create a zero'd message payload with a header that indicates it has never been written.  To write
to it at a later time you simply use::

    msg.write(msgData, time)

Note that stand-alone messages written in Python don't have a module ID.  The message module ID is thus set to -1.

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

Logged Data
-----------
The logging of messages is much simplified.  There are a few changes to note in the format of the logged data.

Here is some sample code.  The only line required to log the state output message use::

    attErrorLog = attErrorConfig.attGuidOutMsg.log()

This creates an object that can be added to a task list through::

    scSim.AddModelToTask(logTaskName, attErrorLog)

The update rate of ``logTaskName`` controls the frequency at which this message is logged.

That is it.  The data is now logged into ``attErrorLog`` automatically during the simulation run.
In the new messaging system  the time information is no longer pre-pended in a first column, but rather provided as a
separate array accessed through ``.times()``.  This means logging `N` time steps of a 3D vector no longer no longer
yields a `Nx4` array, but rather a `Nx3` array.  Some plotting or value checking logic might have to be updated.
For example, to plot using the log data use::

    for idx in range(3):
        plt.plot(attErrorLog.times() * macros.NANO2MIN, attErrorLog.sigma_BR[:, idx])



Miscellaneous Changes
---------------------
If from Python you access ``#define`` values of ``macroDefinitions.h``, such as::

    simFswInterfaceMessages.MAX_EFF_CNT
    fswMessages.MAX_EFF_CNT

then you can now access these definitions using  ``messaging2.i`` using::

    messaging2.MAX_EFF_CNT