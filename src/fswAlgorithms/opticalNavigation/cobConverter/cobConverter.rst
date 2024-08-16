Executive Summary
-----------------

Module reads in a message containing the pixel data extracted from the center of brightness (COB) measurement and
transforms into a position measurement. The written message contains a heading (unit vector) and a covariance
(measurement noise).

Additionally, the center of mass (COM) can be estimated using a Sun phase angle correction. In this case, another unit
vector message is written containing the heading and covariance for the COM, as well as a message containing information
about the COM offset.

The center of mass correction can be applied using the "Lambertian" or "Binary" method. If no correction should be
performed, the method needs to be "NoCorrection". The Lambertian method assumes that the body is a sphere with
lambertian reflectance, while the Binary method assumes a brightness of either 1 or 0 in the image of the body.

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
      - Input navigation message containing the spacecraft attitude in the inertial frame
    * - ephemInMsg
      - :ref:`EphemerisMsgPayload`
      - Input ephemeris message containing the spacecraft position in the inertial frame
    * - opnavUnitVecCOBOutMsg
      - :ref:`OpNavUnitVecMsgPayload`
      - Output unit vector containing the heading vector of the COB and covariance in multiple frames for filtering
    * - opnavUnitVecCOMOutMsg
      - :ref:`OpNavUnitVecMsgPayload`
      - Output unit vector containing the heading vector of the COM and covariance in multiple frames for filtering
    * - opnavCOMOutMsg
      - :ref:`OpNavCOMMsgPayload`
      - Output message containing information about the COM offset due to the phase angle correction

Detailed Module Description
---------------------------

Measurement mapping
^^^^^^^^^^^^^^^^^^^^^

This module models the measurement that will be ingested by the following filter. This is essentially a stand-alone
measurement model for the filter composed via messaging.

After reading the input center of brightness message which contains the pixel location of the center
of brightness and the number of pixels that were found, the module reads the camera parameters and uses
them to compute all necessary optics values.

The main relations used between all of the camera values can be found in `this paper by J. A. Christian
<https://doi.org/10.1109/ACCESS.2021.3051914>`__.

With this, the unit vector in the camera frame from focal point to center of brightness in the image plane is found by
(equivalent to setting z=1):

.. math::

    \mathbf{r}_{COB}^C &= [K]^{-1} \mathbf{\bar{u}}_{COB}

where :math:`\mathbf{\bar{u}}_{COB} = [\mathrm{cob}_x, \mathrm{cob}_y, 1]^T` with the pixel coordinates of the center of
brightmess :math:`\mathrm{cob}_x` and :math:`\mathrm{cob}_y`, :math:`[K]` is the camera calibration matrix and
:math:`\mathbf{r}_{COB}^N` is the unit vector describing the physical heading to the target in the camera frame.

The covariance is found using the number of detected pixels and the camera parameters, given by:

.. math::

    P = \frac{2 \ pi}{\sqrt{\mathrm{numPixels}}} \left( \begin{bmatrix} d_x^2 & 0 & 0 \\ 0 & d_y^2 & 0
    \\ 0 & 0 & 1 \end{bmatrix}\right)

where :math:`d_x` and :math:`d_y` are the first and second diagonal elements of the camera calibration matrix
:math:`[K]`.

By reading the camera orientation and the current body attitude in the inertial frame, the final step is to rotate
the covariance and heading vector in all the relevant frames for modules downstream. This is done simply by
converting MRPs to DCMs and performing the matrix multiplication.
If the incoming image is not valid, the module writes empty messages.

If a COM correction is to be performed, the offset factor :math:`\gamma` due to the Sun phase angle correction is
obtained for a phase angle :math:`\alpha` using

.. math::

    \gamma = \frac{4}{3 \pi} (1 - \cos\alpha)

for the Binary method or

.. math::

    \gamma = \frac{3 \pi}{16} \left[ \frac{(\cos\alpha + 1) \sin\alpha}{\sin\alpha + (\pi - \alpha) \cos\alpha} \right]

for the Lambertian method. If no correction is to be performed, then :math:`\gamma = 0`. The correction for the COM
location is performed according to `this paper by S. Bhaskaran <https://doi.org/10.1109/AERO.1998.687921>`__. First, the
object radius :math:`R` in meters is converted to the object radius in pixel units :math:`R_c` by

.. math::

    R_c = \frac{R K_x f}{\rho} = \frac{R d_x}{\rho}

where :math:`K_x = d_x/f`, :math:`f` is the focal length in meters, and :math:`\rho` is the distance from the
body center to the spacecraft in meters. Using the sun direction in the image plane :math:`\phi`, the COM location in
pixel space is then computed using

.. math::

    \mathrm{com}_x = \mathrm{cob}_x - \gamma R_c \cos\phi \\
    \mathrm{com}_y = \mathrm{cob}_y - \gamma R_c \sin\phi

Finally, similar to the COB unit vector, the COM unit vector is obtained by

.. math::

    \mathbf{r}_{COM}^C &= [K]^{-1} \mathbf{\bar{u}}_{COM}

where :math:`\mathbf{\bar{u}}_{COM} = [\mathrm{com}_x, \mathrm{com}_y, 1]^T`.

User Guide
----------
This section is to outline the steps needed to setup a center of brightness converter in Python.

#. Import the module::

    from Basilisk.fswAlgorithms import cobConverter

#. Create an instantiation of converter class. The COM/COB correction method and object radius need to be specified::

    module = cobConverter.CobConverter(cobConverter.PhaseAngleCorrectionMethod_NoCorrection, R_obj)  # no correction
    # module = cobConverter.CobConverter(cobConverter.PhaseAngleCorrectionMethod_Lambertian, R_obj)  # Lambertian method
    # module = cobConverter.CobConverter(cobConverter.PhaseAngleCorrectionMethod_Binary, R_obj)  # Binary method

#. The object radius in units of meters for the phase angle correction can be updated by::

    module.setRadius(R_obj)

#. Subscribe to the messages::

    module.cameraConfigInMsg.subscribeTo(camInMsg)
    module.opnavCOBInMsg.subscribeTo(cobInMsg)
    module.navAttInMsg.subscribeTo(attInMsg)
    module.ephemInMsg.subscribeTo(ephemInMsg)

#. Add model to task::

    sim.AddModelToTask(taskName, module)
