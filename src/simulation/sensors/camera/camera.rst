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

- Gaussian noise on the image (``gaussian``)
- Dark Noise (``darkCurrent``)
- Dead and stuck pixels (``saltPepper``)
- Random Cosmic rays (``cosmicRays``)
- Blurring (``blurParam``)
- HSV color adjustment (``HSV``)
- RGB percent color adjustment (``RGB

Doxygen documentation for OpenCV can be found `here <https://docs.opencv.org/4.1.2/>`__.


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
