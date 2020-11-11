
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
    | Module Name               | Old Msg Name                  | New Msg Interface Name            |
    +===========================+===============================+===================================+
    | attTrackingError          | ``outputDataMessage``         | ``attGuidOutMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputNavMessage``           | ``attNavInMsg``                   |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRefMessage``           | ``attRefInMsg``                   |
    +---------------------------+-------------------------------+-----------------------------------+
    | gravityEffector           | ``bodyInMsgName``             | ``planetBodyInMsg``               |
    +---------------------------+-------------------------------+-----------------------------------+
    | inertial3D                | ``outMsg``                    | ``attRefOutMsg``                  |
    +---------------------------+-------------------------------+-----------------------------------+
    | inertial3DSpin            | ``outputData``                | ``attRefOutMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRefName``              | ``attRefInMsg``                   |
    +---------------------------+-------------------------------+-----------------------------------+
    | MRP_Feedback →            | ``outputDataName``            | ``cmdTorqueOutMsg``               |
    + ``mrpFeedback``           +-------------------------------+-----------------------------------+
    |                           | ``inputGuidName``             | ``guidInMsg``                     |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``vehConfigInMsgName``        | ``vehConfigInMsg``                |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputRWSpeedsName``         | ``rwSpeedsInMsg``                 |
    +---------------------------+-------------------------------+-----------------------------------+
    | simple_nav →              | ``outputAttMessage``          | ``attOutMsg``                     |
    + ``simpleNav``             +-------------------------------+-----------------------------------+
    |                           | ``outputTransMessage``        | ``transOutMsg``                   |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputStateName``            | ``scStateInMsg``                  |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``inputSunName``              | ``sunStateInMsg``                 |
    +---------------------------+-------------------------------+-----------------------------------+
    | spacecraftPlus            | ``scStateOutMsgName``         | ``scStateOutMsg``                 |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``scMassStateOutMsgName``     | ``scMassOutMsg``                  |
    +---------------------------+-------------------------------+-----------------------------------+
    | extForceTorque            | ``cmdTorqueInMsgName``        | ``cmdTorqueInMsg``                |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``cmdForceInertialInMsgName`` | ``cmdForceInertialInMsg``         |
    +                           +-------------------------------+-----------------------------------+
    |                           | ``cmdForceBodyInMsgName``     | ``cmdForceBodyInMsg``             |
    +---------------------------+-------------------------------+-----------------------------------+


Setting a Basilisk Message from Python
--------------------------------------
Import ``messages2`` to have access to all message definitions::

    from Basilisk.simulation import messaging2

To create a message of type ``ParticularMsg``, first get a copy of the message structure using::

    msgData = messaging2.ParticularMsg()

Next, fill in ``msgData`` with the needed information.  The structure is always initialized to zero on creation.
When done, use the following command to create the Msg object and get a copy for other modules to subscribe to.::

    msg = messaging2.ParticularMsg().write(msgData, time)

The ``time`` is the message write time in nano-seconds.  It is optional and defaults to 0.

Miscellaneous Changes
---------------------
If from Python you access ``#define`` values of ``macroDefinitions.h``, such as::

    simFswInterfaceMessages.MAX_EFF_CNT
    fswMessages.MAX_EFF_CNT

then you can now access these definitions using  ``message.i`` using::

    message.MAX_EFF_CNT