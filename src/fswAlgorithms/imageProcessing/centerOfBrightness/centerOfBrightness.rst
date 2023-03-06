Executive Summary
-----------------

Module reads in a message containing a pointer to an image and writes out the weighted center of brightness of the
image for any pixel above a parametrized threshold.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - opnavCirclesOutMsg
      - :ref:`OpNavCOBMsgPayload`
      - output center of brightness message containing number of detected pixels, and center-of-brightness in pixel space
    * - imageInMsg
      - :ref:`CameraImageMsgPayload`
      - camera image input message containing the pointer to the image

Detailed Module Description
---------------------------

Image Processing
^^^^^^^^^^^^^^^^^^^^^

The module performs a small set of operations to prepare the image for the center of brightness detection.
Firstly, it applies a Gaussian blur on the image, which helps soften the edges of bright objects and improves
the detection performance.
Secondly it thresholds the image in order to remove any dim pixels that may not be of interest to the detection
algorith.
Both of these are implemented using OpenCV methods.

Afterwards the weighted center-of-brightness is implemented by finding all the non-zero pixels (all pixels that were
not thresholded down), and finding their intensity (which will be their weight).
Assuming the image is made up of pixels :math:`p` with and x and y component, that :math:`I` is the intensity, and
:math:`I_{\mathrm{min}` is the threshold value, the center of brightness is then defined as:

.. math::

    \mathcal{P} &= \{p \in \mathrm{image} \hspace{5cm} |  \hspace{5cm} I(p) > I_{\mathrm{min}\} \\
    I_\mathrm{tot} &= \sum_{p \in \mathcal{P}} I(p) \\
    p_{\mathrm{cob}} &= \frac{1}{I_\mathrm{tot}}\sum_{p \in \mathcal{P}} I(p) * p }

where the center of brightess :math:`p_{\mathrm{cob}}` has an x and y component which are then written to the message.

If the incomping image is not valid, or there were no pixels above the threshold, the image is tagged as invalid.
Downstream algorithms can therefore know when to skip a measurement.

User Guide
----------
This section is to outline the steps needed to setup a Center of Brightness in Python.

#. Import the centerOfBrightness class::

    from Basilisk.fswAlgorithms import centerOfBrightness

#. Create an instantiation of a Spinning body::

    cobAlgorithm = centerOfBrightness.CenterOfBrightness()

#. Define all physical parameters for a Spinning Body. For example::

    cobAlgorithm.blurSize = 7
    cobAlgorithm.threshold = 10

#. Subscribe to the image message output by the camera model or visualization interface::

    cobAlgorithm.imageInMsg.subscribeTo(imgInMsg)

#. The angular states of the body are created using an output message ``opnavCOBOutMsg``.

    sim.AddModelToTask(taskName, cobAlgorithm)
