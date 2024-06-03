Executive Summary
-----------------
This module implements a feedback control law to damp the angular rates of the spacecraft until they are brought to zero.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - navAttInMsg
      - :ref:`NavAttMsgPayload`
      - Input navigation message
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - Output C++-wrapped message containing the rate-damping torque in Body-frame coordinates
    * - cmdTorqueOutMsgC
      - :ref:`CmdTorqueBodyMsgPayload`
      - Output C-wrapped message containing the rate-damping torque in Body-frame coordinates



Detailed Module Description
---------------------------
The control law implemented is the following:

.. math::
    \boldsymbol{u} = - P \boldsymbol{\omega}_\mathcal{B/N}

where :math:`P` is a positive, user-defined scalar quantity. The control law is globally asymptotically stabilizing.


User Guide
----------
The required module configuration is::

    attControl = rateDamp.RateDamp()
    attControl.setRateGain(P)
    attControl.ModelTag = "rateDamp"
