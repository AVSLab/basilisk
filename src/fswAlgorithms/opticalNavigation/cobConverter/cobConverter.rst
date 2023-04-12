Executive Summary
-----------------

Module reads in a message containing the pixel data extracted from the center of brightness measurement and transforms
into a position measurement. The written message contains a heading (unit vector) and a covariance (measurement noise).

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for:

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cameraConfigInMsg
      - :ref:`CameraConfigMsgPayload`
      - Input the camera config message
    * - opnavCOBInMsg
      - :ref:`OpNavCOBMsgPayload`
      - Input enter of brightness message written out by the image processing module
    * - navAttInMsg
      - :ref:`NavAttMsgPayload`
      - Input avigation message containing the spacecraft attitude in the inertial frame
    * - opnavUnitVecOutMsg
      - :ref:`OpNavUnitVecMsgPayload`
      - Output unit vector containing the heading vector and covariance in multiple frames for filtering

Detailed Module Description
---------------------------

Measurement mapping
^^^^^^^^^^^^^^^^^^^^^

This module models the measurement that will be ingested by the following filter. This is essentially a stand-alone
measurement model for the filter composed via messaging.

After reading the input center of brightness message which contains the pixel location of the center
of brightness and the number of pixels that were found, the module reads the camera parameters and uses
them to compute all necessary optics values.

The main relation used between all of the camera values is:

.. math::

    \tan \left(\frac{\mathrm{fov}}{2}\right) &= \frac{H}{2f}\\
    2 \tan \left(\frac{\mathrm{fov}}{2}\right) &= d_x * \mathrm{Res}

where :math:`\mathrm{fov}` represents the field of view, :math:`f` is the focal length,
:math:`d_{x}` is the ratio of pixel pitch to focal length, :math:`\mathrm{Res}` is the resolution, and
:math:`H` is the full sensor size.

With this, the equations computed find the unit vector in the camera frame from focal point to
center of brightness in the image plane (equivalent to setting z=1):

.. math::

    r_x &= (\mathrm{cob}_x - c_x + \frac{1}{2})*d_x \\
    r_y &= (\mathrm{cob}_y - c_y + \frac{1}{2})*d_y \\
    r_z &= 1

where :math:`\mathrm{cob}` are the pixel coordinates of the center of brightmess, :math:`c` are the pixel coordinates
for the center of the image (where the camera boresight intersects the detector), and the  :math:`r` vector is the
unit vector describing the physical heading to the target in the camera frame.

The covariance is found using the numer of detected pixels and the camera parameters, given by:

.. math::

    P = \frac{2 \ pi}{\sqrt{\mathrm{numPixels}}} \left( \begin{bmatrix} d_x^2 & 0 & 0 \\ 0 & d_y^2 & 0
    \\ 0 & 0 & 1 \end{bmatrix}\right)

By reading the camera orientation and the current body attitude in the inertial frame, the final step is to rotate
the covariance and heading vector in all the relevant frames for modules downstream. This is done simply by
converting MRPs to DCMs and performing the matrix mulitplication.
If the incoming image is not valid, the module writes and empty message.

User Guide
----------
This section is to outline the steps needed to setup a center of brightness converter in Python.

#. Import the module::

    from Basilisk.fswAlgorithms import cobConverter

#. Create an instantiation of converter class::

    module = cobConverter.CobConverter()

#. There are no parameters to set directly in this module

#. Subscribe to the messages::

    module.cameraConfigInMsg.subscribeTo(camInMsg)
    module.opnavCOBInMsg.subscribeTo(cobInMsg)
    module.navAttInMsg.subscribeTo(attInMsg)

#. Add model to task with some dataLog::

    sim.AddModelToTask(taskName, dataLog)

