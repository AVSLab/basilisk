Executive Summary
-----------------
This document describes how Three-Axis Magnetometer (TAM) devices are modeled in the Basilisk software. The purpose of this module is to implement magnetic field measurements on the sensor frame :math:`S`.

Module Assumptions and Limitations
----------------------------------
Assumptions made in TAM module and the corresponding limitations are shown below:

-   **Magnetic Field Model Inputs**: The magnetometer sensor is limited with the used magnetic field model which are individual magnetic field models complex and having their own assumptions. The reader is referred to the cited literature to learn more about the model limitations and challenges.

-   **Error Inputs**: Since the error models rely on user inputs, these inputs are the most likely source of error in TAM output. Instrument bias would have to be measured experimentally or an educated guess would have to be made. The Gauss-Markov noise model has well-known assumptions and is generally accepted to be a good model for this application.

-   **External Disturbances**: Currently, the module does not consider the external magnetic field, so it is limited to the cases where this effect is not significant. This can be overcome by using magnetic field models taking into these effects account or adding it as an additional term.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - stateInMsg
      - :ref:`SCStatesMsgPayload`
      - input message for spacecraft states
    * - magInMsg
      - :ref:`MagneticFieldMsgPayload`
      - input message for magnetic field data in inertial N frame
    * - tamDataOutMsg
      - :ref:`TAMSensorMsgPayload`
      - magnetic field sensor output message in the sensor frame S



Detailed Module Description
---------------------------
There are a multitude of magnetic field models. As all the magnetic field models are children of :ref:`MagneticFieldBase` base class, magnetometer can be created based on any of the magnetic field model.

Planet Centric Spacecraft Position Vector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
For the following developments, the spacecraft location relative to the planet frame is required. Let :math:`\boldsymbol r_{B/P}` be the spacecraft position vector relative to the planet center.
In the simulation the spacecraft location is given relative to an inertial frame origin :math:`\cal{O}`.
The planet centric position vector is computed using

.. math::

    \boldsymbol r_{B/P} = \boldsymbol r_{B/O} - \boldsymbol r_{P/O}

If no planet ephemeris message is specified, then the planet position vector :math:`r_{P/O}` is set to zero. Let :math:`[PN]` be the direction cosine matrix that relates the rotating planet-fixed frame relative to an inertial frame :math:`\cal{N}`. The simulation provides the spacecraft position vector in inertial frame components. The planet centric position vector is then written in Earth-fixed frame components using

.. math::

    {}^{\cal{P}}{{\boldsymbol r}_{B/P}} = [PN] \ {}^{\cal{N}}{{\boldsymbol r}_{B/P}}


Magnetic Field Models
^^^^^^^^^^^^^^^^^^^^^
The truth of the magnetometer measurements in sensor frame coordinates with no errors are output as:

.. math::

    {}^{\cal{S}}{\boldsymbol B} = [SN]\ {}^{\cal{N}}{\boldsymbol B}

where :math:`[SN]` is the direction cosine matrix from :math:`\cal{N}` to :math:`\cal{S}`, and :math:`{{}^\cal{N}}{ \boldsymbol B}` is the magnetic field vector of the magnetic field model in inertial frame components.

Sensor Error Modeling
^^^^^^^^^^^^^^^^^^^^^
The magnetic field vector of the magnetic field models is considered to be "truth". So, the errors are applied on the "truth" values to simulate a real instrumentation:

.. math::
    \boldsymbol B_{measured} = (\boldsymbol B_{truth} + \boldsymbol e_{noise} + \boldsymbol e_{bias}) f_{scale}

where :math:`\boldsymbol e_{noise}` is the Gaussian noise, :math:`\boldsymbol e_{bias}` is the bias applied on the magnetic field measurements, and :math:`f_{scale}` is the scale factor applied on the measurements for linear scaling.

Saturation
^^^^^^^^^^
Sensors might have specific saturation bounds for their measurements. It also prevents the sensor for giving a value less or higher than the possible hardware output. The saturated values are:

.. math::

    \boldsymbol B_{{sat}_{max}} = \mbox{min}(\boldsymbol B_{measured},  \mbox{maxOutput})

    \boldsymbol B_{{sat}_{min}} = \mbox{max}(\boldsymbol B_{measured},  \mbox{minOutput})

This is the final output of the sensor module.

User Guide
----------

General Module Setup
^^^^^^^^^^^^^^^^^^^^
This section outlines the steps needed to add a Magnetometer module to a sim. First, one of the magnetic field models must be imported:

.. code-block:: python

      from Basilisk.simulation import magneticFieldCenteredDipole
      magModule = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
      magModule.ModelTag = "CenteredDipole"

and/or

.. code-block:: python

      from Basilisk.simulation import magneticFieldWMM
      magModule = magneticFieldWMM.MagneticFieldWMM()
      magModule.ModelTag = "WMM"

Then, the magnetic field measurements must be imported and initialized:

.. code-block:: python

      from Basilisk.simulation import magnetometer
      testModule = magnetometer.Magnetometer()
      testModule.ModelTag = "TAM_sensor"

The model can  be added to a task like other simModels.

.. code-block:: python

      unitTestSim.AddModelToTask(unitTaskName, testModule)

Each Magnetometer module calculates the magnetic field based on the magnetic field and output state messages
of a spacecraft. The spacecraft states are read in through the spacecraft state input message shown above.

Magnetic field data is transformed from inertial to body, then to the sensor frame. The transformation
from :math:`\cal B` to :math:`\cal S` can be set via ``dcm_SB`` using the helper function::

    setBodyToSensorDCM(psi, theta, phi)

where (``psi``, ``theta``, ``phi``) are classical 3-2-1 Euler angles that map from the body frame to the sensor frame :math:`\cal S`.

Specifying TAM Sensor Corruptions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Three types of TAM sensor corruptions can be simulated.  If not specified, all these corruptions are zeroed. To add a Gaussian noise component to the output, the 3d vector ``senNoiseStd`` is set to non-zero values.  This is the standard deviation of Gaussian noise in Tesla.  If any ``senNoiseStd`` component is negative then the noise is not applied.

Next, to simulate a constant bias, the variable ``senBias`` is set to a non-zero value. To simulate a linear scaling of the outputs, the variable ``scaleFactor`` is used.

Finally, to set saturation values, the variables ``maxOutput`` and ``minOutput`` are used. Minimum and maximum bounds for saturation are set to large values as :math:`(-10^{200} \mbox{nT})` and :math:`(10^{200} \mbox{nT})` respectively in order not to saturate the outputs by default.
