
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
:ref:`VizSpacecraftData` data structures for each spacecraft.



.. table:: Module I/O Messages
    :widths: 25 25 100

    +-----------------------+-----------------------------------+---------------------------------------------------+
    | Msg Variable Name     | Msg Type                          | Description                                       |
    +=======================+===================================+===================================================+
    | opnavImageOutMsgName  | :ref:`CameraImageMsg`             | If Vizard is used as a camera sensor, this is the |
    |                       |                                   | the name of the output message of the camera      |
    |                       |                                   | image.                                            |
    +-----------------------+-----------------------------------+---------------------------------------------------+
    | cameraConfInMsgName   | :ref:`CameraConfigMsg`            | Name of the incoming camera data                  |
    +-----------------------+-----------------------------------+---------------------------------------------------+


User Guide
----------
The ``vizInterface`` module can be directly configured, or setup using the helper methods in :ref:`vizSupport`.
More information can be found in :ref:`vizardSettings` page.  The :ref:`scenarioFormationBasic` illustrates and
discusses how to configure ``vizInterface`` for use with multiple satellites.



