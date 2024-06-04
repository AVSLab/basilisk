
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


User Guide
----------
The ``vizInterface`` module can be directly configured, or setup using the helper methods in :ref:`vizSupport`.
More information can be found in :ref:`vizardSettings` page.  The :ref:`scenarioFormationBasic` illustrates and
discusses how to configure ``vizInterface`` for use with multiple satellites.
