Executive Summary
-----------------
This module estimates the position of a camera using triangulation. Given a point cloud of feature locations (such as
boulders on an asteroid), and the pixel coordinates of these features (key points) obtained from an image that the
camera took, the camera location is estimated by triangulation. The triangulation algorithm is based on
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
    * - pointCloudInMsg
      - :ref:`PointCloudMsgPayload`
      - point cloud input message
    * - keyPointsInMsg
      - :ref:`PairedKeyPointsMsgPayload`
      - key points (features) input message
    * - cameraConfigInMsg
      - :ref:`CameraConfigMsgPayload`
      - camera configuration input message
    * - cameraLocationOutMsg
      - :ref:`CameraLocalizationMsgPayload`
      - camera location output message

Module Assumptions and Limitations
----------------------------------
The module assumes the number of features in the provided point cloud and the number of key points from the camera
image are equal, and that the features and corresponding key points are in the same order. The module also assumes that
all key points come from a single camera, so the camera calibration matrix :math:`[K]` and the direction cosine matrix
(DCM) :math:`[CN]` that maps from the inertial frame N to the camera frame C is the same for all key points.

Algorithm
---------
The unknown camera location :math:`{}^N\mathbf{r}`, expressed in the inertial frame N, is estimated by solving the
linear least squares problem

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

where :math:`{}^N\mathbf{p}_i` are the known points (point cloud), :math:`\bar{\mathbf{x}}_i = [K_i] \bar{\mathbf{u}}_i`
with :math:`\bar{\mathbf{u}}_i = [\mathbf{u}_i, 1]^T` and image point in pixel space :math:`\mathbf{u}_i`. The tilde
:math:`[\tilde{\mathbf{x}}]` indicates the skew-symmetric matrix that is equivalent to the cross product
:math:`\mathbf{x} \times`.

User Guide
----------
The module is first initialized as follows:

.. code-block:: python

    module = cameraTriangulation.CameraTriangulation()
    module.ModelTag = "cameraTriangulation"
    unitTestSim.AddModelToTask(unitTaskName, module)

The input messages are then connected:

.. code-block:: python

    module.pointCloudInMsg.subscribeTo(pointCloudInMsg)
    module.keyPointsInMsg.subscribeTo(keyPointsInMsg)
    module.cameraConfigInMsg.subscribeTo(cameraConfigInMsg)
