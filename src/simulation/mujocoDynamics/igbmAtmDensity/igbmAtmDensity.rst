Executive Summary
-----------------
The ``IgbmAtmDensity`` module applies scalar inhomogeneous geometric Brownian motion (IGBM) noise to atmospheric
density by specializing :ref:`InhomogeneousGeometricBrownianMotion <inhomogeneousGeometricBrownianMotion>`. It scales
incoming atmospheric density by the stochastic multiplicative IGBM factor and republishes the perturbed atmosphere
message.

It is the multiplicative-noise counterpart to :ref:`StochasticAtmDensity <stochasticAtmDensity>` (which uses the
additive-noise Ornstein-Uhlenbeck :ref:`MeanRevertingNoise <meanRevertingNoise>`), and the perturbation size scales with
the factor itself. The exact IGBM factor is positive, but an explicit integrator can occasionally drive it non-positive
for a large step/increment, so the corrected density is clamped to be non-negative (a negative density is unphysical).

Message Interfaces
------------------
.. bsk-module-io:: igbmAtmDensity

   input atmoDensInMsg AtmoPropsMsgPayload
      Input atmosphere properties message containing baseline ``neutralDensity`` and other atmospheric fields.

   output atmoDensOutMsg AtmoPropsMsgPayload
      Output atmosphere properties message with stochastic density correction applied to ``neutralDensity``.

Module Description
------------------
The inherited IGBM state :math:`x` evolves as

.. math::
   \text{d}x = \frac{1}{\tau}\,(\mu - x)\,\text{d}t + \sigma\,x\,\text{d}W

configured in stationary form (mean :math:`\mu`, time constant :math:`\tau`, stationary standard deviation
:math:`\sigma_{st}`, with :math:`\sigma` derived internally — see
:ref:`InhomogeneousGeometricBrownianMotion <inhomogeneousGeometricBrownianMotion>`).

The output neutral density is

.. math::
   \rho_\text{out} = \max\!\left(0,\; \rho_\text{in}\,x\right)

so setting the mean level :math:`\mu = 1` gives a mean-preserving multiplicative correction. The :math:`\max(0,\cdot)`
guards against the rare non-positive :math:`x` an explicit integrator can produce (a negative density is unphysical).
All other atmospheric payload fields are passed through unchanged.

Because the diffusion :math:`\sigma x` is multiplicative (state-dependent), a strong stochastic integrator (e.g.
Euler-Maruyama or an SRI method) must be used.
