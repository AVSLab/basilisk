Executive Summary
-----------------

This module combines attitude information from one spacecraft state message
with translational information from another spacecraft state message.  The
merged output is useful when a simulation has separate attitude and
center-of-mass state sources, but a downstream navigation module expects a
single ``SCStatesMsgPayload`` input.


Message Connection Descriptions
-------------------------------
The following table lists the module input and output messages.  The message
type contains a link to the message structure definition, while the description
provides information on what the message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attStateInMsg
      - :ref:`SCStatesMsgPayload`
      - Input spacecraft state message used as the attitude source.
    * - transStateInMsg
      - :ref:`SCStatesMsgPayload`
      - Input spacecraft state message used as the translation source.
    * - stateOutMsg
      - :ref:`SCStatesMsgPayload`
      - Output spacecraft state message containing merged attitude and
        translation fields.


Module Assumptions and Limitations
----------------------------------
The output attitude fields ``sigma_BN`` and ``omega_BN_B`` are copied from
``attStateInMsg``.  The output translation fields ``r_BN_N`` and ``v_BN_N`` are
populated from ``transStateInMsg`` fields ``r_CN_N`` and ``v_CN_N``.


User Guide
----------
The module is imported through the standard simulation package:

.. code-block:: python

    from Basilisk.simulation import stateMerge

    merge = stateMerge.StateMerge()
    merge.ModelTag = "stateMerge"

The two state input messages are connected in the usual Basilisk manner:

.. code-block:: python

    merge.attStateInMsg.subscribeTo(attStateMsg)
    merge.transStateInMsg.subscribeTo(transStateMsg)
    simpleNavObject.scStateInMsg.subscribeTo(merge.stateOutMsg)
