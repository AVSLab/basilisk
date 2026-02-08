Executive Summary
-----------------
The ``scalarJointStatesToRWSpeed`` module is an adapter that converts a set of scalar joint state messages into a single
reaction wheel speed message. The module reads :math:`N` input messages of type :ref:`ScalarJointStateMsgPayload`, each
containing a scalar joint rate :math:`\dot{\theta}_i`. At each update, the module
packs the joint rates into the output :ref:`RWSpeedMsgPayload` such that

.. math::
    \Omega_i = \dot{\theta}_i, \quad i = 0, 1, \ldots, N-1

where :math:`\Omega_i` is the :math:`i`-th reaction wheel speed in the output message. This supports using MuJoCo hinge
joint rates (or any scalar joint rates) as inputs to Basilisk flight software that expects a reaction wheel speed
message.

Message Connection Description
------------------------------
The following table lists the module input and output messages.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - jointStateInMsgs
      - :ref:`ScalarJointStateMsgPayload`
      - Vector of input messages containing the scalar joint states and rates
    * - rwSpeedOutMsg
      - :ref:`RWSpeedMsgPayload`
      - Output message containing the packed reaction wheel speeds

Module Functions
----------------
Below is a list of functions that this simulation module performs

    - Reads all ``ScalarJointStateMsg`` inputs
    - Copies each input joint rate into the corresponding element of the ``RWSpeedMsg`` output
    - Writes the packed ``RWSpeedMsg`` output message each update cycle

Module Assumptions and Limitations
----------------------------------
    - The mapping is order-preserving: input message index :math:`i` maps to output wheel speed index :math:`i`
    - The module uses the joint state from each input message
    - The module assumes :math:`N` does not exceed the maximum wheel speed vector length supported by
      :ref:`RWSpeedMsgPayload`

Test Description and Success Criteria
-------------------------------------
The unit test for this module is defined in :ref:`test_scalarJointStatesToRWSpeed`.
The test initializes the module with a configurable number of joints :math:`N` and drives each joint input message with
a distinct constant rate. The test passes if the output ``wheelSpeeds`` vector matches the expected values for every
index to within numerical tolerance.
