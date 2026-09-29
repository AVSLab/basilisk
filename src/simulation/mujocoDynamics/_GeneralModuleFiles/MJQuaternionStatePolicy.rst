These immutable policies connect MuJoCo quaternion states to the generic integrator.
Both store scalar-first quaternions with shape ``4 x 1`` and apply ``3 x 1`` angular
diffusion tangents through MuJoCo's exponential-map update.

``MJNativeQuaternionStatePolicy`` also uses a ``3 x 1`` body-rate drift.
``MJHighOrderQuaternionStatePolicy`` uses a ``4 x 1`` quaternion derivative, adds the
weighted derivative to the base, and normalizes the candidate. ``MJJoint`` selects
the policy during registration according to the scene's attitude-integration setting.
Changing that setting after topology is established is not permitted.

The registry owns each policy. Repeated registration compares policy topology by
value, and integrator descriptors borrow the committed policy. See
:ref:`integratorArchitecture` for joint layout and stage evaluation.
