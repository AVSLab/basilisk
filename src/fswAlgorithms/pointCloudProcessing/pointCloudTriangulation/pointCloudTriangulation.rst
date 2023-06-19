Executive Summary
-----------------
This module creates a point cloud using triangulation. Given the pixel coordinates of features (key points, such as
boulders on an asteroid) obtained from two images that the camera took at two known camera locations, the point cloud is
estimated by triangulation. The triangulation algorithm is based on
`this paper by S. Henry and J. A. Christian <https://doi.org/10.2514/1.G006989>`__ (Equation 20).

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

    * - ephemerisInMsg
      - :ref:`EphemerisMsgPayload`
      - ephemeris input message
    * - navTransInMsg
      - :ref:`NavTransMsgPayload`
      - translational navigation input message
    * - directionOfMotionInMsg
      - :ref:`DirectionOfMotionMsgPayload`
      - direction of motion input message
    * - keyPointsInMsg
      - :ref:`PairedKeyPointsMsgPayload`
      - key points (features) input message
    * - cameraConfigInMsg
      - :ref:`CameraConfigMsgPayload`
      - camera configuration input message
    * - pointCloudOutMsg
      - :ref:`PointCloudMsgPayload`
      - point cloud output message

Module Assumptions and Limitations
----------------------------------
The module assumes the number of key points (features) in both "images" (at camera location 1 and 2) of the
:ref:`PairedKeyPointsMsgPayload` are equal, and that the key points are in the same order in both images.
The module also assumes that all key points come from a single camera, so the camera calibration matrix :math:`[K]` is
the same for all key points.

Algorithm
---------
The unknown feature location (point of point cloud) :math:`{}^N\mathbf{r}`, expressed in the inertial frame N, is
estimated by solving the linear least squares problem

.. math::
    :label: eq:LLS

    \begin{bmatrix}
        [\tilde{\bar{\mathbf{x}}_1}][CN_1] \\
        [\tilde{\bar{\mathbf{x}}_2}][CN_2] \\
        \vdots \\
        [\tilde{\bar{\mathbf{x}}_n}][CN_n]
    \end{bmatrix} {}^N\mathbf{r} =
    \begin{bmatrix}
        [\tilde{\bar{\mathbf{x}}_1}][CN_1] {}^N\mathbf{p}_1 \\
        [\tilde{\bar{\mathbf{x}}_2}][CN_2] {}^N\mathbf{p}_2 \\
        \vdots \\
        [\tilde{\bar{\mathbf{x}}_n}][CN_n] {}^N\mathbf{p}_n
    \end{bmatrix}

where :math:`{}^N\mathbf{p}_i` are the known camera locations, :math:`\bar{\mathbf{x}}_i = [K_i] \bar{\mathbf{u}}_i`
with :math:`\bar{\mathbf{u}}_i = [\mathbf{u}_i, 1]^T` and image point in pixel space :math:`\mathbf{u}_i`. The tilde
:math:`[\tilde{\mathbf{x}}]` indicates the skew-symmetric matrix that is equivalent to the cross product
:math:`\mathbf{x} \times`.

The module solves this least squares problem in a loop for all points of the point cloud.

User Guide
----------
The module is first initialized as follows:

.. code-block:: python

    module = pointCloudTriangulation.PointCloudTriangulation()
    module.ModelTag = "pointCloudTriangulation"
    module.numberTimeStepsInitialPhase = 3  # optional (defaults to 5)
    unitTestSim.AddModelToTask(unitTaskName, module)

The input messages are then connected:

.. code-block:: python

    module.ephemerisInMsg.subscribeTo(ephemerisInMsg)
    module.navTransInMsg.subscribeTo(navTransInMsg)
    module.directionOfMotionInMsg.subscribeTo(directionOfMotionInMsg)
    module.keyPointsInMsg.subscribeTo(keyPointsInMsg)
    module.cameraConfigInMsg.subscribeTo(cameraConfigInMsg)
