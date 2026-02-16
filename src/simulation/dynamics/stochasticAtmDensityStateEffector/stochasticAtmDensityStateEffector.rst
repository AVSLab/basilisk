Executive Summary
-----------------
The ``StochasticAtmDensityStateEffector`` module is a spacecraft
:ref:`StateEffector <StateEffector>` that applies scalar mean-reverting stochastic
noise to atmospheric density. It reads an input :ref:`AtmoPropsMsgPayload`, scales
``neutralDensity`` with a stochastic correction factor, and writes the perturbed
payload.

Module Description
------------------
The internal correction state :math:`x` follows an Ornstein-Uhlenbeck process:

.. math::
   dx = -\frac{1}{\tau}x\,dt + \sqrt{\frac{2}{\tau}}\sigma_{st}\,dW

The output density is:

.. math::
   \rho_{out} = \rho_{in}(1+x)

The ``localTemp`` field is passed through unchanged.

Message Interfaces
------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - atmoDensInMsg
      - :ref:`AtmoPropsMsgPayload`
      - Input atmosphere properties message containing baseline ``neutralDensity`` and ``localTemp``.
    * - atmoDensOutMsg
      - :ref:`AtmoPropsMsgPayload`
      - Output atmosphere properties message with stochastic correction applied to ``neutralDensity``.

Detailed Behavior
-----------------
At each integrator sub-step (through ``computeDerivatives``), the module:

1. Reads the internal state :math:`x`.
2. Sets deterministic drift :math:`\dot{x} = -x/\tau`.
3. Sets one diffusion term with amplitude :math:`\sqrt{2/\tau}\sigma_{st}`.

After integration (through ``writeOutputStateMessages``), it:

1. Reads ``atmoDensInMsg``.
2. Scales ``neutralDensity`` by :math:`(1+x)`.
3. Writes the modified payload to ``atmoDensOutMsg``.

Module Assumptions and Limitations
----------------------------------
- This module perturbs only ``neutralDensity``.
- Because the registered state has a stochastic diffusion source, a stochastic
  integrator must be used for the parent spacecraft dynamics.
