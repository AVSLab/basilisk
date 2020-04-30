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
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for


.. table:: Module I/O Messages
        :widths: 25 25 100

        +-----------------------+---------------------------------+---------------------------------------------------+
        | Msg Variable Name     | Msg Type                        | Description                                       |
        +=======================+=================================+===================================================+
        | imageInMsgName        | :ref:`CameraImageMsg`           | Input image message                               |
        |                       |                                 | This image is theoretically uncorrupted           |
        |                       |                                 | though this module can also add additional errors.|
        +-----------------------+---------------------------------+---------------------------------------------------+
        | cameraOutMsgName      | :ref:`CameraConfigMsg`          | Camera parameters message.                        |
        +-----------------------+---------------------------------+---------------------------------------------------+
        | imageOutMsgName       | :ref:`CameraImageMsg`           | Output image with corruptions.                    |
        +-----------------------+---------------------------------+---------------------------------------------------+


Detailed Module Description
---------------------------
This modules pulls heavily from the OpenCV library. The methods
implemented create either:

- Gaussian noise on the image
- Dark Current
- Dead and stuck pixels
- Random Cosmic rays
- Blurring
- HSV adjustments
- BGR adjustments

Doxygen documentation for OpenCV can be found `here <https://docs.opencv.org/4.1.2/>`__.

Overview of Corruptions
-----------------------
Gaussian noise is designed to simulate camera sensor noise. This is done by using the addWeighted OpenCV method and
scaling the noise according to the input parameter. The noise is zero-mean with standard deviation equal to twice the
gaussian noise parameter. The image is then thresholded at the 6-σ value of the noise parameter in order to keep
the background dark.

Dark current is due to thermal properties of the CCD or CMOS sensor in use: as electrons are created independently of
incoming light, they are captured in the pixel potential wells and appear to be a signal. Dark current noise is the
statistical variation of this phenomenon. In Basilisk, dark current noise is added with a Gaussian noise model with
zero standard deviation and mean of 15 × D, where D is another input to the module.

Blurring is implemented using the standard OpenCV blur function with size specified by the blur parameter. This type
of blur is referred to as a box blur. The only requirement is that the blur parameter be an odd number.

Dead and stuck pixels are also implemented as a static perturbation. A dead pixel is a pixel on the sensor that always
reads black. A stuck pixel is a pixel on the sensor that always reads white. At the initialization of the run, a
random number generates arbitrary pixel coordinates and either saturates them dark or light.
The corrupted pixels stay the same throughout the simulation and provide an example of a static artifact.

Cosmic rays are ionized nuclei: 90% being protons, 9% alpha particles, and the rest are heavier nuclei. By hitting the
camera sensor with relatively high energies, they can saturate a line of pixels and corrupt the image. Cosmic rays are
modeled in Basilisk by randomly choosing a point on the sensor as well as second point within a maximal distance of the
first one. The abundance of cosmic rays on an image depend on the shutter speed amongst other parameters, and the
module allows to toggle the frequency and quantity of such events.

Finally, the module supports directly adjusting either the pixels of the image through the HSV or BGR colorspace. Note
that OpenCV defaults to the BGR and not RGB colorspace.
Many camera sensors have different color profiles so being able to adjust the color directly is important. Further,
this allows for the adjustment of overall brightness and saturation.

To read more about the corruptions and for example pictures see section 5.2 of Thibaud Teil's thesis.
`Link to Thesis <https://hanspeterschaub.info/Papers/grads/ThibaudTeil.pdf>`__

Because each successive filter is applied on top of the previous, the order in which they are applied is very important.
Currently Basilisk does not support a custom order with out directly modifying the source code. The order is as follows,
gaussian noise, blur, dark current, HSV adjust, RGB adjust, salt/pepper, and cosmic rays. This order was determined
in part by trying to match a simulated image to a real image of mars and also based on what makes sense.

.. list-table:: Order of Corruptions
    :widths: auto
    :header-rows: 1

    * - Corruption
      - Parameters
      - Notes
    * - Gaussian Noise
      - double scaling factor
      - Adds noise with a mean of 0 and standard deviation of 2 * scaling parameter
    * - Blur
      - double blur size
      - Determines the size of the box blur. Blur size parameter must be odd
    * - Dark Current
      - double scaling factor
      - Adds noise with mean of 15 * scaling factor and standard deviation of 0
    * - HSV Adjust
      - vector of three doubles
      - First parameter is given in radians and determines the hue shift. Second two parameters are scaling factors for saturation and value
    * - BGR Adjust
      - vector of three ints
      - Parameters correspond to scaling factors for blue, green, and red
    * - Salt/Pepper
      - double scaling factor
      - Probability of both stuck and dead pixels is calculated as 0.00002 * scaling parameter
    * - Cosmic Rays
      - double number of cosmic rays
      - Adds the specified number of cosmic rays

User Guide
----------
The test and these few lines show an example setup for the module.

.. code-block:: python
    :linenos:

    moduleConfig.imageInMsgName = "sample_image"
    moduleConfig.cameraOutMsgName = "cameraOut"
    moduleConfig.imageOutMsgName = "out_image"
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

These values are written such that 0 provides no corruption of that
type and 10 provides very high levels of errors (not bounding though)
