
Manager of states for Basilisk dynamical systems.  Allows the state-
effector models of a dynamic object to create, get, and update states
present in the model.

The ``useManagerLocalEffectorNames`` flag selects manager-local automatic effector
names; it defaults to ``False`` for legacy compatibility. Select the policy before
name collection or data registration. See :ref:`effectorNaming` for initialization,
custom names, collision checks, and logging guidance.

Legacy automatic effector naming is deprecated for removal on September 14, 2027.
When a participating effector registers an automatic legacy name, the manager
records a pending diagnostic. Its Python interface reports this once through
``deprecated.deprecationWarn()``: ``BSKDeprecationWarning`` before the removal date,
then ``BSKUrgentDeprecationWarning`` on or after it. Repeated initialization does
not repeat it. Explicit names alone and the manager-local policy do not trigger
this warning. The date is defined in ``dynParamManager.i``; it does not disable
legacy naming automatically.
