Executive Summary
-----------------

This module compares the magnitude of the accumulated Delta-V from the :ref:`NavTransMsgPayload` message with the
desired Delta-V magnitude of the burn maneuver from the :ref:`DvBurnCmdMsgPayload` message. The module assumes that the
thrusters have been turned on by another module (such as :ref:`thrFiringRemainder`, for example), and turns the
thrusters off once the desired Delta-V has been accumulated. A minimum and maximum time can also be specified. The
thrusters are only turned off once the desired Delta-V has been accumulated and the burn time is greater than the
specified minimum time (defaults to 0.0 seconds), unless the burn time exceeds the maximum time. If the burn time is
greater than the maximum time, the thrusters are turned off regardless of the accumulated Delta-V. If no maximum time
is specified, the burn is only stopped using the accumulated Delta-V criteria.
The user should specify the flight software time step (control period) using the defaultControlPeriod parameter.
Otherwise the burn time is not accurately computed.

If the same set of DV thrusters is also used for attitude control (with the :ref:`thrForceMapping`,
:ref:`thrFiringRemainder` or :ref:`thrFiringSchmitt` modules and the baseThrustState in the corresponding modules
set to 1), the thrusters are turned on by :ref:`thrFiringRemainder` or :ref:`thrFiringSchmitt` at the beginning of the
burn, and the :ref:`dvExecuteGuidance` module turns off the thrusters at the end of the burn. To ensure that the
:ref:`THRArrayOnTimeCmdMsgPayload` output message of the :ref:`dvExecuteGuidance` module turns off the thrusters, this
module should be updated more frequently than :ref:`thrFiringRemainder` or :ref:`thrFiringSchmitt` and with a lower (!)
task priority.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - navDataInMsg
      - :ref:`NavTransMsgPayload`
      - navigation input message that includes dv accumulation info
    * - burnDataInMsg
      - :ref:`DvBurnCmdMsgPayload`
      - commanded burn input message
    * - thrCmdOutMsg
      - :ref:`THRArrayOnTimeCmdMsgPayload`
      - thruster command on time output message
    * - burnExecOutMsg
      - :ref:`DvExecutionDataMsgPayload`
      - burn execution output message

User Guide
----------
The module is first initialized as follows:

.. code-block:: python

    moduleConfig = dvExecuteGuidance.dvExecuteGuidanceConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "dvExecuteGuidance"
    moduleConfig.defaultControlPeriod = 0.5
    moduleConfig.minTime = 2.0
    moduleConfig.maxTime = 10.0
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

The input messages are then connected:

.. code-block:: python

    moduleConfig.navDataInMsg.subscribeTo(navTransMsg)
    moduleConfig.burnDataInMsg.subscribeTo(dvBurnCmdMsg)
