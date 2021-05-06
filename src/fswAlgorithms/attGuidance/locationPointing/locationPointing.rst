Executive Summary
-----------------
This module generates an attitude guidance message to make a specified spacecraft pointing vector target a location on a planet given the spacecraft and ground states.

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
    * - SCInMsg
      - :ref:`SCStatesMsgPayload`
      - describe space craft states
    * - LocationInMsg
      - :ref:`GroundStateMsgPayload`
      - ground state info
    * - AttGuidOutMsg
      - :ref:`AttGuidMsgPayload`
      - attitude guidance data for control modules

Detailed Module Description
-------------------------------
.. math::
    :label: eq-fswModule-firstLaw

    \hat{e} = \frac{\hat{p}x\hat{r}_{L/S}}{|\hat{p}x\hat{r}_{L/S}|}



