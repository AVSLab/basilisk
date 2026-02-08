Executive Summary
-----------------
The ``arrayMotorTorqueToSingleActuators`` module is an adapter that converts an array motor torque command message into a
set of scalar actuator command messages. The module reads a single input message of type
:ref:`ArrayMotorTorqueMsgPayload`, which contains a motor torque vector :math:`\boldsymbol{u}`. At each update, the
module writes :math:`N` output messages of type :ref:`SingleActuatorMsgPayload` such that

.. math::
    u_i^{(\text{out})} = u_i, \quad i = 0, 1, \ldots, N-1

where :math:`u_i^{(\text{out})}` is the scalar command written to the :math:`i`-th actuator output message. This supports
connecting flight software modules that publish array-valued motor commands to simulation actuators that expect
individual scalar actuator command messages.

Message Connection Description
------------------------------
The following table lists the module input and output messages.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - torqueInMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - Input message containing the motor torque array
    * - actuatorOutMsgs
      - :ref:`SingleActuatorMsgPayload`
      - Vector of output messages containing one scalar actuator command per actuator

Module Functions
----------------
Below is a list of functions that this simulation module performs

    - Reads the incoming ``ArrayMotorTorqueMsg`` input
    - Copies each array entry into the corresponding scalar ``SingleActuatorMsg`` output
    - Writes all scalar actuator output messages each update cycle

Module Assumptions and Limitations
----------------------------------
    - The mapping is order-preserving: array index :math:`i` maps to output actuator index :math:`i`
    - The module assumes :math:`N` does not exceed the size of the input motor torque array provided by
      :ref:`ArrayMotorTorqueMsgPayload`

Test Description and Success Criteria
-------------------------------------
The unit test for this module is defined in :ref:`test_arrayMotorTorqueToSingleActuators`.
The test initializes the module with a configurable number of actuators :math:`N`, publishes a known torque vector to
the input message, and runs one simulation step. The test passes if each scalar output message matches the
corresponding input array element to within numerical tolerance.
