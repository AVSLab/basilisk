Executive Summary
-----------------

Scaling Iterative Closest Point is an implementation of the Iterative Closest Point
algorith, with a scale factor added to the optimization. The algorithm seeks to
identify the rotation, scale, and translation in order to best overlap two point clouds.
This implementation produces the measured point cloud after it has been fit to the reference
as well as the geometric transformations that were found by SICP in a separate message.


Message Connection Descriptions
-------------------------------

The input messages are both of the same type since they are both point clouds. One is a
measured point cloud (which could be output by a LIDAR for instance) and the reference
is input given the space model of the target.
The output messages are the geometrical transformations identified by the algorithm
(translation, scale, and rotation), and the resulting measured point cloud after it has
been transformed to fit the reference.


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - measuredPointCloud
      - :ref:`PointCloudMsgPayload`
      - input point cloud that is measured and needs to be fit
    * - referencePointCloud
      - :ref:`PointCloudMsgPayload`
      - input reference point cloud that the measured cloud is trying to fit to
    * - outputPointCloud
      - :ref:`PointCloudMsgPayload`
      - output point cloud once transformed to fit the reference
    * - outputSICPData
      - :ref:`SICPMsgPayload`
      - output transformations to apply to the input measured cloud to best fit the reference

Detailed Module Description
---------------------------

The algorithm is described and derived in the reference paper:
:download:`SICP by Du et al </../../src/fswAlgorithms/pointCloudProcessing/SICP/_Documentation/SICP_Du.pdf>`
The summary can be found in pseudo-code pages 444-445 and similar notation is used in the
implementation.

User Guide
----------
This section is to outline the steps needed to setup a SICPin Python.

#. Import the SICP class::

    from Basilisk.fswAlgorithms import scalingIterativeClosestPoint

#. Create an instantiation of the module::

    sicp = scalingIterativeClosestPoint.ScalingIterativeClosestPoint()

#. Define all physical parameters. The max number of iterations describes how times SICP will
try and fit the data if the min errors are not met. The errorTolerance defines that maximum
error, scalingMin and Max are the max allowable scaling range for the scalar term::

    sicp.maxIterations = 20
    sicp.errorTolerance = 1E-5
    sicp.scalingMax = 1.1
    sicp.scalingMin = 0.9

#. Subscribe to the input cloud messages ::

    inputPointCloud = messaging.PointCloudMsgPayload()
    referencePointCloud = messaging.PointCloudMsgPayload()

