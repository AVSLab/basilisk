Executive Summary
-----------------

Module to initialize and prepare data going into an Iterative Closest Point algorithm.
The module reads previous ICP results and ephemeris/camera data and sets the initial conditions
depending on if the module had been recently reset.
This process produces a normalized measured point cloud as well as an ICP message containing
a set of R, S, and t parameters to initialize the next ICP attempt.


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
    * - inputMeasuredPointCloud
      - :ref:`PointCloudMsgPayload`
      - input point cloud that was measured and is unnormalized
    * - inputSICPData
      - :ref:`SICPMsgPayload`
      - ICP data from the algorithms previous iteration
    * - ephemerisInMsg
      - :ref:`EphemerisMsgPayload`
      - Ephemeris of the spacecraft to get initial
    * - inputSICPData
      - :ref:`SICPMsgPayload`
      - ICP data from the algorithms previous iteration
    * - normalizedMeasuredPointCloud
      - :ref:`PointCloudMsgPayload`
      - ouput point cloud after normalization
    * - initializeSICPMsg
      - :ref:`SICPMsgPayload`
      - output transformations to apply as initial conditions to ICP

Detailed Module Description
---------------------------

The module first takes the average of all the points in the measured point cloud.
It then divides each point by this average norm.

In case a reset has recently been called, the spacecraft ephemeris is read as well as the camera frame relative
to the body. These are used to set the base rotation, translation, and scale.

If previous ICP algorithms have produced results, the last results are used directly as initalization.


User Guide
----------
This section is to outline the steps needed to setup a SICPin Python.

#. Import the SICP class::

    from Basilisk.fswAlgorithms import initializeICP

#. Create an instantiation of the module::

    initICP = initializeICP.InitializeICP()

#. Define all physical parameters. The max number of iterations describes how times SICP will
try and fit the data if the min errors are not met. The errorTolerance defines that maximum
error, scalingMin and Max are the max allowable scaling range for the scalar term::

    initICP.maxIterations = 20
    initICP.errorTolerance = 1E-5
    initICP.scalingMax = 1.1
    initICP.scalingMin = 0.9

#. Subscribe to the input cloud messages ::

    inputPointCloud = messaging.PointCloudMsgPayload()
    referencePointCloud = messaging.PointCloudMsgPayload()

