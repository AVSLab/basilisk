
Executive Summary
-----------------
This module provides an interface to package up Basilisk messages and pass them onto the :ref:`Vizard <vizard>`
application.  This allows for the Basilisk simulation data to be recorded to a file for play-back, or for
live streaming of the simulation data.  It is possible to record the simulation data of a single spacecraft or a
multitude of spacecraft.

Module Assumptions and Limitations
----------------------------------
The module is only able to read Basilisk simulation state messages that are accessable from the task group that
is calling ``vizInterface``.


Message Connection Descriptions
-------------------------------
The following messages are set directly within ``vizInterface``.  Additional messages are set within the
``VizSpacecraftData`` data structures for each spacecraft.


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Parameter
      - Default
      - Description
    * - opnavImageOutMsgs
      - :ref:`CameraImageMsgPayload`
      - (optional) vector of Image output messages, the corresponding camera configuration input message is setup
        through ``vizInterface.addCamMsgToModule(msg)``
    * - epochInMsg
      - :ref:`EpochMsgPayload`
      - (optional) simulation epoch date/time input msg
    * - spiceInMsgs
      - :ref:`SpicePlanetStateMsgPayload`
      - (optional) vector of input messages of planet Spice data

The ``VizSpacecraftData`` structure, defined in :ref:`vizStructures`, contains a range of input messages for each spacecraft added.

.. list-table:: ``VizSpacecraftData`` input messages per spacecraft
    :widths: 25 25 50
    :header-rows: 1

    * - Parameter
      - Default
      - Description
    * - scStateInMsg
      - :ref:`SCStatesMsgPayload`
      - msg of incoming spacecraft state data
    * - rwInMsgs
      - :ref:`RWConfigLogMsgPayload`
      - (optional) Vector of incoming RW state messages
    * - thrInMsgs
      - :ref:`THROutputMsgPayload`
      - (optional) vector of thruster input messages
    * - cssInMsgs
      - :ref:`CSSConfigLogMsgPayload`
      - (optional) Vector of CSS config log messages


Handshaking Protocol
--------------------
``vizInterface`` facilitates the port configuration and handshaking process between BSK and Vizard. The current implementation of this protocol is defined here for developers, for use if an alternate visualization destination is desired.

BSK instantiates the broadcast socket using ``zmq_bind``, and the 2-way socket using ``zmq_connect``. Vizard joins these sockets using reciprocal ``connect`` and ``bind`` structure where appropriate.

The remaining complexity is with the 2-way socket, which couples BSK and Vizard in lockstep. After instantiation, ``vizInterface`` empties the socket and sends "PING" to start communication. When sending a sim update, ``vizInterface`` first sends the string "SIM_UPDATE", followed by 2 empty messages, followed by the serialized message protobuffer. It then listens for the response "OK". Periodic "PING" messages may be sent to keep the socket alive. To receive user input from Vizard, ``vizInterface`` sends the string "REQUEST_INPUT". It then collects the user input message, followed by a status string. The string "VIZARD_INPUT" signifies a successful send, while "ERROR" signifies an issue during transmission. When using ``opNav``, the string "REQUEST_IMAGE\_" is sent to request for an image from configured onboard cameras.

For more specifics in message packaging and handling, see the source code for this module.


User Guide
----------
The ``vizInterface`` module can be directly configured, or setup using the helper methods in :ref:`vizSupport`.
More information can be found in :ref:`vizardSettings` page.  The :ref:`scenarioFormationBasic` illustrates and
discusses how to configure ``vizInterface`` for use with multiple satellites.
