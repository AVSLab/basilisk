Executive Summary
-----------------

Module reads in a image message, and an attitude message.
When two image messages have provided image pointers at seperate times, their attitudes (BN at the time of image) are
identified and saved. The first image is processed to find "good features", meaning using Shi-Tomasi features,
while one the second image is processed it is used with Optical Flow in order to match the features pairs between images.

A message is then written containing the new and old features, the number of features, and the attitudes and times at
which they were taken.

Descriptions of the core algorithms can be found on the OpenCV documentation here: `GoodFeatures`_, `OpticalFlow`_

.. _GoodFeatures: https://docs.opencv.org/3.4/d4/d8c/tutorial_py_shi_tomasi.html
.. _OpticalFlow: https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html


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
    * - keyPointsMsg
      - :ref:`PairedKeyPointsMsgPayload`
      - Output key point message
    * - imageInMsg
      - :ref:`CameraImageMsgPayload`
      - Input camera message containing images
    * - attitudeMsg
      - :ref:`NavAttMsgPayload`
      - Input Spacecraft body to inertial attitude
    * - ephemerisMsg
      - :ref:`EphemerisMsgPayload`
      - Input Target body to inertial attitude

Module Description
-------------------------------

The logic of the module flows as such: as UpdateState is called and messages are processed, it checks
for when a newImage is processed. If no image was processed prior, it finds features in the image,
and stores this data as the old image.
When a second image comes through the module, it is now the new image of a pair. A bool is raised that
both an old and a new image are present. Optical Flow can now be used to track the features between images.
Once that is done all of the data is sent in a message containing the times of the measurements,
attitudes when the images were taken, features, cameraID, and number of features matched.

A mask is applied to the images when used for Shi-Tomasi feature detection in order to constrain the
feature detection to the disk, and avoid sharp edges at the limb. This done using the following steps:

 * First the contours are found and drawn on an image.
 * Second the main contour is filled with black
 * Third the image is blured and thresholded to remove roughness on the limb
 * Finally the edge is dilated to increase the margin off the limb
 * The bitwise_not version of this final image is the mask to be placed on the image

The module outputs a message containing the key point pairs between two sets of images and the
relevant information for each image.

User Guide
----------
This section is to outline the steps needed to setup a OpticalFlow in Python.

#. Import the opticalFlow class::

    from Basilisk.fswAlgorithms import opticalFlow

#. Create an instantiation of the module::

    module = opticalFlow.OpticalFlow()

#. Define general parameters. The minTimeBetweenPairs forces a minimum number of seconds between two images for
feature tracking. For example::

    module.ModelTag = "opticalFlow"
    module.minTimeBetweenPairs = 5

#. Define parameters for the goodFeaturesToTrack method::

    module.maxNumberFeatures = 100
    module.qualityLevel = 0.1
    module.minumumFeatureDistance = 5
    module.blockSize = 10

#. Define parameters specific to the optical flow::

    module.criteriaMaxCount = 10
    module.criteriaEpsilon = 0.01
    module.flowSearchSize = 10
    module.flowMaxLevel = 2

#. Connect the approriate messages::

    module.imageInMsg.subscribeTo(imageInMsg)
    module.attitudeMsg.subscribeTo(attInMsg)
    module.ephemerisMsg.subscribeTo(ephemInMsg)

