.. _autoCModule:

Module: autoCModule
===================

Executive Summary
-----------------
This is an auto-created sample C module.  The description is included with the module class definition

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
    * - someInMsg
      - :ref:`AttRefMsgPayload`
      - input msg description
    * - some2InMsg
      - :ref:`AttRefMsgPayload`
      - input msg description
    * - anotherInMsg
      - :ref:`CSSConfigMsgPayload`
      - input msg description
    * - some2OutMsg
      - :ref:`AttRefMsgPayload`
      - output msg description
    * - someOutMsg
      - :ref:`SCStatesMsgPayload`
      - output msg description



----

.. autodoxygenfile:: autoCModule.h
   :project: autoCModule

