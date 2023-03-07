Executive Summary
-----------------

This module models the temperature of a sensor. It takes into account thermal emission, absorption, and conversion of
electrical energy to thermal energy within the sensor. The sensor is assumed to be a flat plate with an insulated
backing.

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
    * - sunInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - Input planet state message for the sun.
    * - stateInMsg
      - :ref:`SCStatesMsgPayload`
      - Input spacecraft state input message.
    * - sunEclipseInMsg
      - :ref:`EclipseMsgPayload`
      - Optional eclipse input message
    * - sensorStatusInMsg
      - :ref:`DeviceStatusMsgPayload`
      - Optional device status input message
    * - temperatureOutMsg
      - :ref:`TemperatureMsgPayload`
      - Output temperature message.

Detailed Module Description
---------------------------

This module simulates the temperature of a sensor impacted by radiative thermal emission, radiative thermal absorption, and
electrical power converted to thermal power.

The radiative thermal emission is computed using the Stefan-Boltzmann Law:

.. math::
    \dot{Q}_{\text{emit}} = \epsilon \cdot \sigma \cdot A \cdot T^4

where :math:`\epsilon` is the emissivity constant, :math:`\sigma` is the Stefan-Boltzmann constant, :math:`A` is the
radiative surface area, and :math:`T` is the current temperature of the sensor.

The radiative thermal absorption, which comes only from the sun in this module, is given by:

.. math::
    \dot{Q}_{\text{absorb}} = shadowFactor \cdot \alpha \cdot S \cdot A_{\text{proj}}

The :math:`shadowFactor` is an indication of whether or not the sensor is in eclipse. A shadowFactor of 0 indicates the sensor
is in eclipse. Likewise, a shadowFactor of 1 indicates the sensor is not in eclipse. The :math:`\alpha` parameter
is the absorptivity constant of the sensor. :math:`S` is the solar constant, and :math:`A_{\text{proj}}` is the projected
area of the sensor with respect to incoming solar flux.

To compute the temperature at each timestep, the following energy balance equation is used:

.. math::
    \rho \cdot Vol \cdot c_p \dfrac{dT}{dt} = \dot{Q}_{\text{absorb}} + \dot{Q}_{in} - \dot{Q}_{\text{emit}}

where :math:`\rho` is the density of the material, :math:`Vol` is the volume of the material, and :math:`c_p` is the
specific heat. :math:`\dot{Q}_{in}` is the electrical power consumed by the sensor.

The change in temperature is solved for, and the current temperature is updated using Euler integration:

.. math::
    T_i = T_{i-1} + \dfrac{dT}{dt} \cdot dt


Model Assumptions and Limitations
---------------------------------

This code makes the following assumptions:

- The sensor is a flat plate with an insulated backing (i.e. there is no conduction from the sensor to the spacecraft).
- The Earth's albedo does not impact the temperature.
- All of the electrical power is converted to heat. The heat is conducted throughout the sensor instantaneously.


User Guide
----------

Default Parameters
~~~~~~~~~~~~~~~~~~

This module has several parameters that are set to default values, but may be changed as shown in the Module Setup
section below. These default parameters are as follows:

.. list-table:: Default Module Parameters
    :widths: 25 50 25
    :header-rows: 1

    * - Parameter
      - Description
      - Default Value
    * - nHat_B
      - Required parameter. Normal vector of the sensor face in the spacecraft body frame.
      - [0.0, 0.0, 0.0]
    * - sensorArea
      - Required parameter. Area of the sensor.
      - -1
    * - sensorAbsorptivity
      - Required parameter. Absorptivity coefficient of sensor.
      - -1
    * - sensorEmissivity
      - Required parameter. Emissivity coefficient of sensor.
      - -1
    * - sensorMass
      - Required parameter. Mass of the sensor.
      - 1.0 kg.
    * - sensorSpecificHeat
      - Required parameter. Specific heat of the sensor.
      - 890 J/kg/K (aluminum).
    * - T_0
      - Optional parameter. Initial temperature of the sensor.
      - 0.0 degrees Celsius.
    * - sensorPowerDraw
      - Optional parameter. Power draw of the sensor.
      - 0.0 W.
    * - sensorPowerStatus
      - Optional parameter. Whether the sensor is powered on (1) or off (0). Overwritten by sensorStatusInMsg (if connected).
      - 1


Module Setup
~~~~~~~~~~~~

The temperature module is created in python using:

.. code-block:: python
    :linenos:

    sensorThermalModel = sensorThermal.SensorThermal()
    sensorThermalModel.ModelTag = 'sensorThermalModel'

A sample setup is done using:

.. code-block:: python
    :linenos:

    sensorThermalModel.nHat_B = [0, 0, 1]  # Body-frame vector of the normal face of the plate
    sensorThermalModel.sensorArea = 1.0  # m^2
    sensorThermalModel.sensorAbsorptivity = 0.25  # unitless
    sensorThermalModel.sensorEmissivity = 0.34  # unitless
    sensorThermalModel.sensorMass = 2.0  # kg
    sensorThermalModel.sensorSpecificHeat = 890  # J/kg/K
    sensorThermalModel.sensorPowerDraw = 30.0   # Watts
    sensorThermalModel.T_0 = 0   # [Celsius]

The relevant messages are then connected to the module:

.. code-block:: python
    :linenos:

    sensorThermalModel.sunInMsg.subscribeTo(sunMsg)
    sensorThermalModel.stateInMsg.subscribeTo(scStateMsg)
    sensorThermalModel.sensorStatusInMsg.subscribeTo(sensorStatusMsg)
    unitTestSim.AddModelToTask(unitTaskName, sensorThermalModel)