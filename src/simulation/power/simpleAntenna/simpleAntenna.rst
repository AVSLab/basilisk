Executive Summary
-----------------
This module describes how the simple antenna model operates within the simulation environment.The purpose of this module is to provide a basic representation of antenna behavior for spacecraft communication systems.The antenna is based on a simple model (2D Gaussian beam pattern). The user is able to set gain, HPBW (elevation and azimuth), Tx and Rx power demand, radiation efficiency, bandwidth, noise temperature.If the antenna is in ground environment, atmospheric losses are computed based on the ITU-R P.676-12 model.In this case the user is able to set ambient pressure, temperature, humidity, oxygen and water vapor partial pressures (default values according to ISA).

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - scState
      - :ref:`scStateOutMsgPayload`
      - spacecraft state
    * - groundState
      - :ref:`currentGroundStateMsgPayload`
      - ground state
    * - antennaState
      - :ref:`antennaStateMsgPayload`
      - setting antenna to [off / Rx / Tx / RxTx]
    * - isaAtmState
      - :ref:`atmosphereMsgPayload`
      - atmospheric conditions at a certain height
    * - antennaState
      - :ref:`antennaOutMsgPayload`
      - output msg description
    * - powerDemand
      - :ref:`nodePowerOutMsgPayload`
      - output msg description
