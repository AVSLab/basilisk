Executive Summary
-----------------

This module implements a opNav point attitude guidance routine.

This algorithm is intended to be incredibly simple and robust: it finds the angle error between the camera boresight (or desired control axis in the camera frame) and the planet heading in the camera frame and brings them to zero. This is analogous to sunSafePoint.  The file
:download:`PDF Description </../../src/fswAlgorithms/attGuidance/opNavPoint/_Documentation/Basilisk-opNavPoint-20190820.pdf>`.
contains further information on this module's function, how to run it, as well as testing.

Frame Conventions
-----------------
The target heading from ``opnavDataInMsg`` and the camera axis ``alignAxis_C`` are both expressed in camera
frame :math:`\mathcal{C}` components.  The principal rotation axis is computed from these two vectors in camera
components and then mapped into body frame :math:`\mathcal{B}` components with the camera orientation
:math:`[CB]` from ``cameraConfigInMsg``:

.. math::

    {}^{\mathcal{B}}\hat{\boldsymbol{e}} = [CB]^{T}\, {}^{\mathcal{C}}\hat{\boldsymbol{e}}

The output ``sigma_BR`` is built from this body frame axis, so the implied reference attitude points
``alignAxis_C`` at the target for any camera mounting ``sigma_CB``.  The module variable ``opNavMnvrVec`` holds
the body frame axis, while ``eHat180_C`` holds the fallback axis, in camera frame components, used when the target
is nearly opposite ``alignAxis_C``.

Message Connection Descriptions
-------------------------------
The following diagram and table list all the module input and output messages.  The module message connection is
set by the user from Python.  The message type contains a link to the message structure definition, while the
description provides information on what this message is used for.

.. bsk-module-io:: opNavPoint
    :caption: Module I/O Messages

    output attGuidanceOutMsg AttGuidMsgPayload
        name of the output guidance message
    input opnavDataInMsg OpNavMsgPayload
        name of the optical navigation input message
    input imuInMsg NavAttMsgPayload
        name of the incoming IMU message
    input cameraConfigInMsg CameraConfigMsgPayload
        name of the camera config message
