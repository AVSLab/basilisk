Executive Summary
-----------------
The ``StochasticAtmDensity`` module applies scalar mean-reverting stochastic noise to atmospheric density by
specializing :ref:`MeanRevertingNoise <meanRevertingNoise>`. It scales incoming atmospheric
density by a stochastic multiplicative factor and republishes the perturbed atmosphere message.

Module Description
------------------
The inherited Ornstein-Uhlenbeck state :math:`x` evolves as

.. math::
   dx = -\frac{1}{\tau}x\,dt + \sqrt{\frac{2}{\tau}}\sigma_{st}\,dW

The output neutral density is

.. math::
   \rho_{out} = \rho_{in}(1 + x)

All other atmospheric payload fields are passed through unchanged.

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
      - Input atmosphere properties message containing baseline ``neutralDensity`` and other atmospheric fields.
    * - atmoDensOutMsg
      - :ref:`AtmoPropsMsgPayload`
      - Output atmosphere properties message with stochastic density correction applied to ``neutralDensity``.

Detailed Behavior
-----------------
At each update step, the module performs the following operations:

1. Reads the current OU state :math:`x` from the base class.
2. Reads ``atmoDensInMsg``.
3. Multiplies ``neutralDensity`` by :math:`(1+x)`.
4. Writes the modified payload to ``atmoDensOutMsg``.

The OU process parameters are configured through base-class setters:

- ``setStationaryStd(sigma_st)``
- ``setTimeConstant(tau)``

Module Assumptions and Limitations
----------------------------------
- The model applies multiplicative density noise only; it does not model spatial correlation or altitude-dependent
  stochastic parameters internally.
- If :math:`x < -1`, the corrected density can become negative unless constrained externally.

Verification and Testing
------------------------
The module behavior is validated in
``src/simulation/mujocoDynamics/meanRevertingNoise/_UnitTest/test_meanRevertingNoise.py`` by checking that the output
density time series has the expected OU statistics (mean, variance, and correlation time) for a constant nominal
density input.
