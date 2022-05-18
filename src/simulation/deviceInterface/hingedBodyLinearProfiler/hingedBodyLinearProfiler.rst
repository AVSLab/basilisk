Executive Summary
-----------------
Linear deployment profiler for single hinged rigid body.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg connection is set by the user from python. The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - hingedRigidBodyReferenceOutMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Output message for reference hinged rigid body state

Detailed Model Description
--------------------------

This module provides a reference angle and angle rate for a linear deployment of a single hinged rigid body. The user sets a start and stop time, as well as a start and stop angle, and the reference angle and angle rate are calculated for a linear deployment.


User Guide
----------

This section contains a conceptual overview of the code and an example for the prospective user.

Module Setup
~~~~~~~~~~~~

The interface module is created in python using:

.. code-block:: python
    :linenos:

    testModule = hingedBodyLinearProfiler.HingedBodyLinearProfiler()
    testModule.ModelTag = "deploymentProfiler"

The deployment is defined by four parameters, the starting and ending times and angles. An error will occur if the delta between start and end time is not positive.

A sample setup is done using:

.. code-block:: python
    :linenos:

    from Basilisk.utilities import macros
    from math import pi

    testModule.startTime = macros.sec2nano(30) # [ns] start the deployment 30 seconds into the simulation
    testModule.endTime = macros.sec2nano(330) # [ns] continue deploying for 300 seconds
    testModule.startTheta = 0 # [rad] starting angle in radians
    testModule.endTheta = 10*pi/180 # [rad] ending angle is 10 degrees in the positive direction as defined by hinged rigid body frame

