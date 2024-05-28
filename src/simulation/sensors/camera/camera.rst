Executive Summary
-----------------
The goal of the camera module is to simulate a camera in the Basilisk
codebase. Although images are provided by the visualization, they are
renders of the Unity engine and are not necessarily representative of
a camera. The module reads in an image from a file, or in the
simulation as a pointer to image data, then corrupts it according to
input parameters. 

Module Assumptions and Limitations
----------------------------------
The practical limitation of this module is that it decodes and
re-encodes the images that are corrupted. Outside of this design choice, the limitations are limited to the
corruption methods used to replicate real camera physics. A Gaussian Dark Current might not always be a good
model to represent such a phenomenon.


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
    * - imageInMsg
      - :ref:`CameraImageMsgPayload`
      - camera input message
    * - cameraConfigOutMsg
      - :ref:`CameraConfigMsgPayload`
      - camera parameters output message
    * - imageOutMsg
      - :ref:`CameraImageMsgPayload`
      - camera output message


Detailed Module Description
---------------------------
Overview of Corruption Sequence
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This modules pulls heavily from the OpenCV library to perform a series of image corruptions. The following methods
are implemented and executed in the sequence shown:

- Gaussian noise on the image
- Blurring
- Dark Current
- HSV adjustments
- BGR adjustments
- Dead and stuck pixels
- Random Cosmic rays

Doxygen documentation for OpenCV can be found `here <https://docs.opencv.org/4.5.5/>`__.

Overview of Corruptions
~~~~~~~~~~~~~~~~~~~~~~~
#. Gaussian noise is designed to simulate camera sensor noise. This is done by using the addWeighted OpenCV method and
   scaling the noise according to the input parameter. The noise is zero-mean with standard deviation equal to twice the
   gaussian noise parameter. The image is then thresholded at the 6-sigma value of the noise parameter in order to keep
   the background dark.

#. Blurring is implemented using the standard OpenCV blur function with size specified by the blur parameter. This type
   of blur is referred to as a box blur. The only requirement is that the blur parameter be an odd number.

#. Dark current is due to thermal properties of the CCD or CMOS sensor in use: as electrons are created independently of
   incoming light, they are captured in the pixel potential wells and appear to be a signal. Dark current noise is the
   statistical variation of this phenomenon. In Basilisk, dark current noise is added with a Gaussian noise model with
   zero standard deviation and mean of 15 × D, where D is another input to the module.

#. The image color can be changed in the HSV color space.  The hue value is rotated by  a specified amount of radians,
   while the saturation and value component can be adjusted through a positive or negative percentage values.  The
   hue value rotates such that 360 degrees becomes 0 degrees.  The saturation and value components are limited to [0,255].

#. The image color can be changed in the BGR color space.  All color channel changes are specified through
   an integer percentage value that can be either positive or negative.  The resulting integer color value is
   limited to lie within [0,255].

#. Dead and stuck pixels are also implemented as a static perturbation. A dead pixel is a pixel on the sensor that always
   reads black. A stuck pixel is a pixel on the sensor that always reads white. At the initialization of the run, a
   random number generates arbitrary pixel coordinates and either saturates them dark or light.
   The corrupted pixels stay the same throughout the simulation and provide an example of a static artifact.

#. Camera sensor with relatively high energies, they can saturate a line of pixels and corrupt the image. Cosmic rays are
   modeled in Basilisk by randomly choosing a point on the sensor as well as second point within a maximal distance of the
   first one. The abundance of cosmic rays on an image depend on the shutter speed amongst other parameters, and the
   module allows to toggle the frequency and quantity of such events.


To read more about the corruptions and for example pictures see section 5.2 of Dr. Thibaud Teil's
`thesis <https://hanspeterschaub.info/Papers/grads/ThibaudTeil.pdf>`__.

Because each successive filter is applied on top of the previous, the order in which they are applied is very important.
Currently Basilisk does not support a custom order with out directly modifying the source code. The order is as shown
in the following table describing the filter and the parameters to control the filter. This order was determined
in part by trying to match a simulated image to a real image of mars and also based on what makes sense.
In the following parameters a value of 0 turns this corruption off and is the default value.  Any filter
that should be supplied must be provided a non-zero filter parameter value.

.. list-table:: Order of Corruptions
    :widths: auto
    :header-rows: 1

    * - Corruption
      - Parameters
      - Notes
    * - Gaussian Noise
      - ``gaussian``
      - [double] Adds noise with a mean of 0 and standard deviation of 2 * scaling parameter
    * - Blur
      - ``blurParam``
      - [double] Determines the size of the box blur. Blur size parameter must be odd
    * - Dark Current
      - ``darkCurrent``
      - [double] Adds noise with mean of 15 * scaling factor and standard deviation of 0
    * - HSV Adjust
      - ``hsv``
      - [3D vector of doubles] First parameter is given in radians and determines the hue shift. Second two parameters are scaling factors for saturation and value
    * - BGR Adjust
      - ``bgrPercent``
      - [3D vector of ints] Parameters correspond to scaling factors for blue, green, and red
    * - Salt/Pepper
      - ``saltPepper``
      - [double] Probability of both stuck and dead pixels is calculated as 0.00002 * scaling parameter
    * - Cosmic Rays
      - ``cosmicRays``
      - [double] Adds the specified number of cosmic rays

User Guide
----------
The test and these few lines show an example setup for the module.

.. code-block:: python
    :linenos:

    moduleConfig.imageInMsg.subscribeTo(inputCamMsg)

    moduleConfig.filename = ""
    moduleConfig.saveImages = 0
    # If images are to be saved, add the directory to which they
    should be saved
    #moduleConfig.saveDir = '/'.join(imagePath.split('/')[:-1]) + '/'

    #Camera config values
    moduleConfig.cameraIsOn = 1
    moduleConfig.sigma_CB = [0,0,1]

    #Noise Values
    moduleConfig.gaussian = 2
    moduleConfig.darkCurrent = 1
    moduleConfig.saltPepper = 2
    moduleConfig.cosmicRays = 1
    moduleConfig.blurParam = 3
    moduleConfig.hsv = [30*macros.D2R, 0, 0]

These scalar double values are written such that 0 provides no corruption of that
type and 10 provides very high levels of errors (not bounding though)
