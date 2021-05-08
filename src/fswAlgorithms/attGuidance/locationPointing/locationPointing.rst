Executive Summary
-----------------
This module generates an attitude guidance message to make a specified spacecraft pointing vector target an inertial location.
This location could be on a planet if this module is connected with :ref:`groundLocation` for example.  

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
    * - scInMsg
      - :ref:`SCStatesMsgPayload`
      - input msg with inertial spacecraft states 
    * - LocationInMsg
      - :ref:`GroundStateMsgPayload`
      - input msg containing the inertial point location of interest
    * - AttGuidOutMsg
      - :ref:`AttGuidMsgPayload`
      - output message with the attitude guidance



Detailed Module Description
-------------------------------
.. math::
    :label: eq-fswModule-firstLaw

    \hat{e} = \frac{\hat{p}x\hat{r}_{L/S}}{|\hat{p}x\hat{r}_{L/S}|}



