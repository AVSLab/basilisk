Executive Summary
-----------------

This module simulates a spacecraft mass sensor. It receives the mass properties from a spacecraft object using a ``simulation`` type message, and converts it into a ``FSW`` type message to be used in flight software modules.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - scMassPropsInMsg
      - :ref:`SCMassPropsMsgPayload`
      - Spacecraft mass properties input message. It contains the mass, inertia and center of mass of the spacecraft in a ``simulation`` type message.
    * - vehicleConfigOutMsg
      - :ref:`VehicleConfigMsgPayload`
      - Vehicle configuration output message. It contains the mass, inertia and center of mass of the spacecraft in a ``FSW`` type message.


Detailed Module Description
---------------------------

The module copies the contents from a :ref:`SCMassPropsMsgPayload` to a :ref:`VehicleConfigMsgPayload` message.


Model Assumptions and Limitations
---------------------------------

This code makes the following assumptions:

- **Sensor is perfect:** The sensor measures the exact spacecraft mass properties without noise or any other interference.


User Guide
----------

This section contains conceptual overviews of the code and clear examples for the prospective user.

Module Setup
~~~~~~~~~~~~

The module is created in python using:

.. code-block:: python
    :linenos:

    scMassPropsModule = simpleMassProps.SimpleMassProps()
    scMassPropsModule.ModelTag = "scMassPropsModule"
