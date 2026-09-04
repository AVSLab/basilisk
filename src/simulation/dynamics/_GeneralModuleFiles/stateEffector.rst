Executive Summary
-----------------

``StateEffector`` is the abstract base class for effectors with integrated
states, including reaction wheels, flexible bodies, prescribed motion, and
fuel slosh. A dynamic object calls each effector during every integrator stage
to assemble instantaneous mass properties, Backsubstitution terms, state
derivatives, and energy and momentum contributions.

Retained Mass-Property Derivatives and Equation-of-Motion Overrides
-------------------------------------------------------------------

The original ``EffectorMassProps`` fields describe derivatives of the
retained effector mass properties:

.. math::

    \mathtt{mEffDot},\quad
    \mathtt{rEffPrime\_CB\_B},\quad
    \mathtt{IEffPrimePntB\_B}.

They are used for the reported total-mass, center-of-mass, and inertia
derivatives. The reported center-of-mass derivative follows the complete
retained first-moment quotient rule.

A variable-mass approximation can intentionally omit generic rate-dependent
terms from the equations of motion without corrupting the reported
derivatives. Such an effector sets ``hasMassPropertyRateDynamics`` and fills
``mEffDotDynamics``, ``rEffPrime_CB_BDynamics``, and
``IEffPrimePntB_BDynamics``. :ref:`spacecraft`
uses these overrides only in its equations of motion. The existing
``rEffPrime_CB_B`` remains the body-frame derivative of the retained
effector center of mass. Effectors that do not set the flag retain the
established behavior in which the corresponding retained-property rates are
also used as the dynamics rates.

Dynamics-rate overrides are currently supported only for effectors attached
directly to :ref:`spacecraft`. ``PrescribedMotionStateEffector`` rejects
variable-mass nested effectors rather than silently applying incomplete
variable-mass bookkeeping. The legacy ``SpacecraftSystem`` does not consume
this override contract and provides no runtime guard for it; use
:ref:`spacecraft` for effectors that provide dynamics-rate overrides.

The dynamics-only center-of-mass-rate quantity follows the quotient rule on
the retained mass properties,

.. math::

    \mathbf c'_{\mathrm{dyn}} =
    \frac{\sum_i m_i\mathbf r'_{i,\mathrm{dyn}}}{M}
    - \frac{\dot M_{\mathrm{dyn}}}{M}\mathbf c.

This differs from the derivative reported through ``centerOfMassPrimeSC``,
which additionally carries the source first moments described below.

A complete open-system mass-flow model also requires the position and
velocity at which mass crosses the selected control-volume boundary. Those
quantities cannot be reconstructed from retained mass, center-of-mass, and
inertia rates. The coupled-depletion ``FuelTank.setUpdateOnly(False)`` model
therefore continues to omit the source first moments
:math:`\dot m_i\mathbf r_i`, and a model that needs them must supply its own
momentum-flux loads.

Implementation Requirements
---------------------------

An effector using explicit equation-of-motion rate overrides must:

* assign ``mEffDotDynamics``, ``rEffPrime_CB_BDynamics``, and
  ``IEffPrimePntB_BDynamics`` on every call;
* reset quantities that are zero rather than relying on values from a prior
  integrator stage;
* evaluate all values from the current stage state and ``integTime``;
* document which legacy rate terms are retained or omitted.
