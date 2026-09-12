Executive Summary
-----------------
The ``MeanRevertingNoiseStateEffector`` module is a spacecraft
:ref:`StateEffector<StateEffector>` that propagates a single scalar
mean-reverting stochastic state.

Module Description
------------------
The internal state :math:`x` follows an Ornstein-Uhlenbeck process:

.. math::
   dx = -\frac{1}{\tau}x\,dt + \sqrt{\frac{2}{\tau}}\sigma_{st}\,dW

No module-specific input or output messages are used. Other modules can consume
the state by reading it from the dynamics state manager by name.

The registered state name can be set or queried using ``setStateName("...")`` and ``getStateName()``.

Detailed Behavior
-----------------
At each integrator sub-step, the module:

1. Reads the internal state :math:`x`.
2. Sets deterministic drift :math:`\dot{x} = -x/\tau`.
3. Sets one diffusion term with amplitude :math:`\sqrt{2/\tau}\sigma_{st}`.

Module Assumptions and Limitations
----------------------------------
- Because the registered state has a stochastic diffusion source, a stochastic
  integrator must be used for the parent spacecraft dynamics.


Configuration Validation
------------------------
Numeric setters reject NaN and positive or negative infinity before modifying either the
configured value or an already registered state. ``setTimeConstant()`` requires a finite,
strictly positive time constant, and ``setStationaryStd()`` requires a finite, non-negative
standard deviation. ``setStateValue()`` requires a finite value.

Valid defaults and private configuration members make attachment safe without task scheduling.
The inherited ``Reset()`` leaves the configured parameters and integrated state unchanged.

The drift and diffusion calculations avoid intermediate overflow when taking square roots or
forming stationary-parameter ratios. If the resulting drift or diffusion is non-finite, the
module raises ``BasiliskError`` before storing either result.
