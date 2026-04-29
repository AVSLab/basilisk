Executive Summary
-----------------
The ``StochasticAtmDensity`` module applies scalar mean-reverting stochastic noise to atmospheric density by
specializing :ref:`MeanRevertingNoise <meanRevertingNoise>`. It scales incoming atmospheric
density by a stochastic multiplicative factor and republishes the perturbed atmosphere message.

Message Interfaces
------------------
.. bsk-module-io:: stochasticAtmDensity

   input atmoDensInMsg AtmoPropsMsgPayload
      Input atmosphere properties message containing baseline ``neutralDensity`` and other atmospheric fields.

   output atmoDensOutMsg AtmoPropsMsgPayload
      Output atmosphere properties message with stochastic density correction applied to ``neutralDensity``.

Module Description
------------------
The inherited Ornstein-Uhlenbeck state :math:`x` evolves as

.. math::
   \text{d}x = -\frac{1}{\tau}x\,\text{d}t + \sqrt{\frac{2}{\tau}}\sigma_{st}\,\text{d}W

The output neutral density is

.. math::
   \rho_\text{out} = \rho_\text{in}(1 + x)

All other atmospheric payload fields are passed through unchanged.

Verification and Testing
------------------------
The module behavior is validated in
``src/simulation/mujocoDynamics/stochasticAtmDensity/_UnitTest/test_stochasticAtmDensity.py`` by checking that the output
density time series has the expected OU statistics (mean, variance, and correlation time) for a constant nominal
density input.
