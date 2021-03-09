Executive Summary
-----------------

Module that computes the motor temperature. It takes into account mechanical power efficiency, as well as friction, 
as sources of heat. There is also a dissipative term due to the temperature gradient between the motor and the
air surrounding it.

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
    * - rwStateInMsg
      - :ref:`RWConfigLogMsgPayload`
      - Input reaction wheel state message. It contains the mechanical torque, friction torque and wheel speed information to compute the mechanical power.
    * - temperatureOutMsg
      - :ref:`TemperatureMsgPayload`
      - Output temperature message.

Detailed Module Description
---------------------------

This module is a simulation environment module which simulates the temperature of a motor. Temperature is computed as a function of the net power to the motor
and the motor's heat capacity, which maps the net power to a temperature variation.
The heat generation component is composed of two factors: mechanical power inefficiencies (loss through current, etc) and friction contributions. The mechanical
power :math:`P_{mec}` is given by:

.. math::
    P_{mec}=\Omega \times u_s

where :math:`\Omega` is the motor angular velocity and :math:`u_s` represents the applied torque. Given the mechanical efficiency :math:`\eta`, we can convert 
mechanical power into thermal power as follows:

.. math::
    P_{loss}=P_{mec}\frac{1-\eta}{\eta}

Similarly to the mechanical power, the friction dissipation is given by:

.. math::
    P_{f}=\Omega \times \tau_f

where :math:`\tau_f` represents the friction torque. The absolute value of these two terms added together represents the thermal power generation. As for the power
dissipation, we assume a temperature gradient between the motor and the surrounding environment. For simplicity, a constant ambient temperature is considered. The 
thermal power dissipation is calculated using the temperature difference between the motor and the environment, scaled by the ambient's thermal resistance :math:`R_{ambient}`:

.. math::
    P_{dissipation} = (T_k - T_{ambient})/R_{ambient}

The subscript k in the temperature implies that it is the motor's temperature at time k. For the final calculation, we need to convert power into heat. This is done
through a simple Euler integration:

.. math::
    Q=P\Delta t

where :math:`Q` represents the heat, :math:`P` is the power and :math:`\Delta t` is the integration time interval, which is the same as the simulation time step.
Finally, the new motor temperature can be calculated using the temperature at the previous time step plus the net heat scaled by the motor's heat capacity:

.. math::
    T_{k+1} = T_k + \Delta t\frac{P_{generation}-P_{dissipation}}{C_{motor}} = T_k + \Delta t\frac{|P_{loss}|+|P_{f}|-P_{dissipation}/R_{ambient}}{C_{motor}}

where, again, the k+1 subscript denotes the temperature at the next time step.

Tips for estimating :math:`C_{motor}` and :math:`R_{ambient}`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The motor's heat capacity can be estimated from its mass and specific heat capacity:

.. math::
    C_{motor}=m_{motor}c_{motor}

The constant :math:`c` is specific to each material. For example, steel has a specific heat capacity of 466 :math:`Jkg^{-1}Celsius^{-1}`.
As for the ambient thermal resistance, it can be estimated using the following equation:

.. math::
    R_{ambient}=\frac{1}{hA}

where :math:`A` is the area where the heat is being dissipated (for example, the area of a cilinder) and :math:`h` represents the convection
coefficient, which for air should be around 100 :math:`Wm^{-2}Celsius^{-1}`.

Model Functions
---------------

The functions of the temperature model are:

- **Temperature Modelling:** Simulates the motor's temperature given previous temperature, motor characteristics and ambient conditions.


Model Assumptions and Limitations
---------------------------------

This code makes the following assumptions:

- **Ambient temperature is constant:** Ambient temperature is not affected by the motor's temperature.
- **Ambient thermal resistance is constant:** Ambient thermal resistance does not change with temperature, pressure, etc.
- **Motor heat capacity is constant:** Motor heat capacity does not change with temperature.


User Guide
----------

This section contains conceptual overviews of the code and clear examples for the prospective user.

Module Setup
~~~~~~~~~~~~

The temperature module is created in python using:

.. code-block:: python
    :linenos:

    thermalModel = motorThermal.MotorThermal()
    thermalModel.ModelTag = 'rwThermals'

A sample setup is done using:

.. code-block:: python
    :linenos:

    thermalModel.currentTemperature = 40  # [Celsius]
    thermalModel.ambientTemperature = 20  # [Celsius]
    thermalModel.efficiency = 0.7
    thermalModel.ambientThermalResistance = 5  # Air Thermal Resistance
    thermalModel.motorHeatCapacity = 50  # Motor Heat Capacity