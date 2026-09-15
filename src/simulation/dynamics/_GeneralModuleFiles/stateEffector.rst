Executive Summary
-----------------

``StateEffector`` is the abstract base class for effectors with integrated
states, including reaction wheels, flexible bodies, prescribed motion, and
fuel slosh. A dynamic object calls each effector during every integrator stage
to assemble instantaneous mass properties, Backsubstitution terms, state
derivatives, and energy and momentum contributions.

Manager-Local Naming Protocol
-----------------------------

The opt-in policy described in :ref:`effectorNaming` requires every attached state
effector to declare its states and properties before registration. External C++
effectors can retain legacy support while implementing these preparation hooks:

#. Override ``describeEffectorNames()`` to return a non-empty family and an
   ``EffectorNameSpec`` for each name. Each specification contains a unique local
   key, state or property kind, automatic prefix and suffix, and an optional exact
   custom name. Track explicit assignments in setters; do not infer custom names
   by comparing a string with its constructor default.
#. Override ``getNestedStateEffectors()`` when the effector owns an attachment
   tree. Return all children in attachment order. The shared collection pass
   rejects cycles and children attached more than once.
#. In manager-local registration, collect or validate the declaration with
   ``collectEffectorNames(manager)``, then retrieve final names through
   ``getResolvedEffectorName(manager, key)``. The spacecraft resolves the complete
   set of declarations before it calls effector registration.
#. Use ``registerEffectorState()`` and ``createEffectorProperty()`` with
   ``getEffectorNameRequest()`` and the local key. String-only registration cannot
   claim a reserved name. Preserve the existing registration path under
   ``EffectorNamingPolicy::Legacy`` using ``registerLegacyEffectorState()`` and
   ``createLegacyEffectorProperty()``. Set the ``automatic`` argument to ``true``
   only when no custom name was explicitly assigned. These helpers preserve
   legacy registration behavior and record its use for the dated Python
   deprecation warning, reported once per manager. Call them only for
   states and properties actually registered by the configured model; an unused
   optional state must not trigger a warning.
#. Implement ``bindAttachedDynamicEffectors()`` if there are dependent attachments.
   Refresh their property names and bindings here, after the entire forest is
   registered. Recurse through owned children. Bindings for a dynamic effector
   shared by multiple parents must distinguish each parent and body segment.
#. Reject changes to resolved names and attachment topology. Repeated preparation
   must reuse names and compatible storage. Follow the module's established
   initialization behavior for state values.

For direct C++ use with a shared manager, collect all roots with
``StateEffector::collectEffectorNames(manager, roots)``, register fixed names from
all owners, call ``manager.resolveEffectorNames()``, register every root, and then
bind every root. Resolving each independent tree separately would freeze the
manager before later declarations can participate in collision checks. Name
preparation does not add physical support for otherwise unsupported attachments.

The default declaration is empty. In manager-local mode it raises an actionable
error before state or property registration, including the effector's
``ModelTag`` when the object is also a ``SysModel``. Legacy mode does not require
these hooks.

Generic ``registerState()`` and ``createProperty()`` calls do not identify whether
a string was generated automatically. External effectors that continue to use
these calls in legacy mode therefore do not participate in the automatic-naming
warning. Track assignments explicitly and use the legacy helpers to participate;
fixed hub names and manually registered data can retain the generic calls.

The shared ``dynParamManager.i`` interface reports pending warnings after direct
Python calls with a ``DynParamManager&`` argument return from C++. The dynamic
object wrappers and ``SimulationBaseClass.InitializeSimulation()`` cover
initialization of attached trees; task resets also report pending warnings.
Scheduled managers defer reports throughout these lifecycle calls, including
reports triggered by Python callbacks running on C++ workers. The report uses
``deprecated.deprecationWarn()`` and its standard ordinary/urgent warning
categories. Deferred reports run on the Python calling thread after successful
initialization or reset, including when Python filters treat warnings as
exceptions. Simulations initialized inside these callbacks join the outer
reporting scope. Their model owners are retained until reporting finishes, so
temporary inner simulations can be safely discarded by the callback.
Independent concurrent simulations retain separate reporting scopes, and
unrelated managers retain their own reporting behavior.

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
