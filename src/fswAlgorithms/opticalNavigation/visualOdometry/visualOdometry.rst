Executive Summary
-----------------
The visualOdometry module implements an algorithm which seeks to find the
direction of motion of a camera tracking features between images.
The output of the module is the DirectionOfMotion message which contains a unit
vector of the velocity direction in the camera frame, the covariance in the camera frame,
the time, and camera ID.

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
    * - keyPointPairInMsg
      - :ref:`PairedKeyPointsMsgPayload`
      - Key points matched between two images input message
    * - cameraConfigInMsg
      - :ref:`CameraConfigMsgPayload`
      - camera configuration input message
    * - dirOfMotionMsgOutput
      - :ref:`DirectionOfMotionMsgPayload`
      - direction of motion output message

Module Assumptions and Limitations
----------------------------------

The pixel error on the features :math:`\sigma_{uv}` is assumed constant.

Algorithm
---------

The algorithm is described and derived in the reference paper:
"Image-Based Lunar Terrain Relative Navigation Without a Map: Measurements" by John A. Christian et al,
published in JOURNAL OF SPACECRAFT AND ROCKETS Vol. 58, No. 1, January–February 2021.
The summary can be found in pseudo-code on page 171 and similar notation is used in the
implementation.

User Guide
----------
The module is first initialized as follows:

.. code-block:: python

    module = visualOdometry.VisualOdometry()
    module.ModelTag = "directionOfMotion"
    module.errorTolerance = 1E-5
    module.sigma_uv = 1
    module.deltaKsi_tilde = 0.1

The input messages also need to be connected:

.. code-block:: python

    module.keyPointPairInMsg.subscribeTo(keyPointMsg)
    module.cameraConfigInMsg.subscribeTo(cameraMsg)
