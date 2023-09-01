
Executive Summary
-----------------
This module simulates a camera pinhole model that computes landmark pixels as seen by a spacecraft.
It reads the planet's ephemeris and spacecraft state messages as inputs. It outputs a vector of landmark messages.

The module can also account for Sun's lighting constraint by setting a mask angle. 



Module Assumptions and Limitations
----------------------------------
The module assumes a landmark is characterized by its position and surface normal (to check field of view and lighting conditions). Diffuse reflection of the light is assumed thus the reflected ray is aligned with the surface normal. Under the previous assumptions, the landmark can be thought as an infinitesimal surface element.

The pinhole camera model lacks ray-tracing capabilities. Consequently, when using a landmark distribution based on a non-convex shape, it is possible that false positives occur. This means that landmarks that are not visible can be marked as such.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ephemerisInMsg
      - :ref:`EphemerisMsgPayload`
      - planet ephemeris input message.
    * - scStateInMsg
      - :ref:`SCStatesMsgPayload`
      - vector of sc state input messages.
    * - landmarkOutMsgs
      - :ref:`LandmarkMsgPayload`
      - vector of landmark messages.


Detailed Module Description
---------------------------
The ``pinholeCamera`` module handles the following behavior:

#. Reads in the spacecraft's state using the ``scStateInMsg`` and planet's ephemeris using the ``ephemerisInMsg`` input message
#. Computes spacecraft position ``r_BP_P`` and orientation with respect to planet frame ``dcm_BP``. It also computes planet to Sun's unit-vector in planet frame ``e_SP_P``.
#. It loops through each landmark determining if it is within camera field of view and illuminated by Sun.
#. If the previous conditions are met, the landmark pixel is saved.
#. It writes a vector of ``landmarkOutMsgs`` messages for each landmark.


Determining Landmark Pixels
~~~~~~~~~~~~~~~~~~~~~~~~~~~
By assuming :math:`z` as the focal direction, the pinhole camera model maps a 3D point to 2D image plane coordinates as:

.. math:: \begin{bmatrix}u\\ v\end{bmatrix} = \frac{f}{z}\begin{bmatrix}x\\y\end{bmatrix}
    :label: eq:pixel:1

Since the image is digital, the pixel is defined by

.. math:: p_x = \begin{cases}\text{ceil}(u/w_p)\>\>\>\>\text{if}\>\>u\geq 0\\\text{floor}(u/w_p)\>\>\text{if}\>\>u<0\end{cases}\>\>\>\>\>\>\>\> p_y = \begin{cases}\text{ceil}(v/w_p)\>\>\>\>\text{if}\>\>v\geq 0\\\text{floor}(v/w_p)\>\>\text{if}\>\>v<0\end{cases}
    :label: eq:pixel:2

.. _glPixelSketch:
.. figure:: /../../src/simulation/navigation/pinholeCamera/_Documentation/Images/camera.png
    :align: center

    Figure 1: Diagram of pinhole camera

A landmark is marked as visible if it pass the subsequent checks. The first check is that the landmark is illuminated by the Sun, thus

.. math:: \pi/2 - \arccos(\mathbf{e}^{P}_{SP}\cdot\mathbf{n}^{P}_{L}) \geq \theta_{Sun}
    :label: eq:check_landmark:1

where :math:`\mathbf{e}^{P}_{SP}` is planet to Sun unit-vector, :math:`\mathbf{n}^{P}_{L}` is the landmark normal and :math:`\theta_{Sun}` Sun's mask angle.

The following checks first filter landmarks that are behind the camera focal direction :math:`\mathbf{k}^{P}_{C}`. Then, it eliminates the landmarks that do not reflect light to the camera:

.. math:: \pi/2 - \arccos(-\mathbf{k}^{P}_{C}\cdot\mathbf{e}^{P}_{BL}) \geq 0
    :label: eq:check_landmark:2

.. math:: \pi/2 - \arccos(-\mathbf{k}^{P}_{C}\cdot\mathbf{n}^{P}_{L}) \geq 0
    :label: eq:check_landmark:3

where :math:`\mathbf{k}^{P}_{C}` is the camera focal direction and :math:`\mathbf{e}^{P}_{BL}` is the unit-vector from landmark to spacecraft.

Finally, for the remaining landmarks, the ones that are not within camera field of view are also eliminated.


User Guide
----------

To use this module, instantiate the class and provide it with camera parameters and a landmark distribution. The camera parameters comprise: horizontal number of pixel; vertical number of pixel; pixel width; focal length; direction cosine matrix of camera w.r.t. body frame; Sun's mask angle (optional). The landmark distribution is added using the ``addLandmark(pos, normal)`` method. The module has to be subscribed to a planet ephemeris message :ref:`EphemerisMsgPayload` and a spacecraft state message :ref:`SCStatesMsgPayload`.

An instance of pinholeCamera, alongside necessary user-supplied parameters, can be created by calling:

.. code-block:: python

    camera = pinholeCamera.PinholeCamera()
    camera.ModelTag = "camera"
    camera.nxPixel = 2048 # Sets number of horizontal pixels
    camera.nyPixel = 1536 # Sets number of vertical pixels
    camera.wPixel = (17.3*1e-3)/2048 # Sets pixel width
    camera.f = 25*1e-3 # Sets camera focal length
    camera.maskSun = 0*np.pi/180 # Sets Sun's mask angle (optional)
    camera.dcm_CB = [[0,0,-1],[0,1,0],[1,0,0]] # Sets dcm of camera w.r.t. spacecraft body (focal direction is [0,0,1] in camera frame)
    for i in range(n_lmk):
        camera.addLandmark(pos_lmk[i, 0:3], normal_lmk[i, 0:3]) # Adds a landmark
    camera.ephemerisInMsg.subscribeTo(ephemConverter.ephemOutMsgs[0]) # Sets planet ephemeris message
    camera.scStateInMsg.subscribeTo(scObject.scStateOutMsg) # Set spacecraft state message
    scSim.AddModelToTask(simTaskName, camera)


The ``maskSun`` variable is optional and defaults to -90º.  This means by default the entire surface is lighted. Set it to a positive value to consider the lighting constraint.

The ``processBatch(rBatch_BP_P, mrpBatch_BP, eBatch_SP_P, show_progress)`` method allows to execute the module detached from the ``SimulationBaseClass``. It can be called as:

.. code-block:: python

    # Preallocate output pixels
    pixelBatchLmk = np.zeros((n, n_lmk, 2))

    # Process pinhole camera as a batch
    e_SP_P = -r_PN_P / np.linalg.norm(r_PN_P, axis=1)[:, None] # Unit-vector from planet to Sun
    camera.processBatch(r_BP_P, mrp_BP, e_SP_P, True) # Execute method (last argument is a bool, when True it shows progress status)
    isvisibleBatchLmk = np.array(camera.isvisibleBatchLmk) # Save visibility status
    pixelBatchLmk[:, :, 0] = np.array(camera.pixelBatchLmk)[:, 0:n_lmk] # Save px
    pixelBatchLmk[:, :, 1] = np.array(camera.pixelBatchLmk)[:, n_lmk:2*n_lmk] # Save py
