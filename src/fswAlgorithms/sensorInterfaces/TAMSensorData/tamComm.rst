Executive Summary
-----------------
This module reads in a message of type :ref:`TAMSensorBodyFswMsg`, outputs the magnetometer measurement vector in vehicle's body coordinates ``tam_B`` with the name of ``tamOutMsgName``.

Module Assumptions and Limitations
----------------------------------
No assumptions are made.

Message Connection Descriptions
-------------------------------
The following table lists the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. table:: Module I/O Messages
        :widths: 25 25 100

        +-------------------+----------------------------+-------------------------------------+
        | Msg Variable Name | Msg Type                   | Description                         |
        +===================+============================+=====================================+
        | tamInMsgName      | :ref:`TAMSensorIntMsg`     | TAM sensor interface input message  |
        +-------------------+----------------------------+-------------------------------------+
        | tamOutMsgName     | :ref:`TAMSensorBodyFswMsg` | TAM sensor interface output message |
        +-------------------+----------------------------+-------------------------------------+


User Guide
----------
In order to transform the ``tam_S`` vector of :ref:`TAMSensorIntMsg` from sensor to body frame, ``dcm_BS`` should be defined.
