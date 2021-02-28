Executive Summary
-----------------

This is a very basic dummy C++ Basilisk module that can be used as a template to create other C++ modules.
It mimics the functionality of :ref:`cModuleTemplate`.  See that module for a more complete discussion
of how to write the RST module documentation file.  


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
    * - dataInMsg
      - :ref:`CModuleTemplateMsgPayload`
      - (optional) Input message description.  Note here if this message is optional, and what the default behavior
        is if this message is not provided.
    * - dataOutMsg
      - :ref:`CModuleTemplateMsgPayload`
      - Output message description.
