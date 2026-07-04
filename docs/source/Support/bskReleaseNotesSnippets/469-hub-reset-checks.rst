- :ref:`spacecraft` and :ref:`spacecraftSystem` now validate the user-supplied hub configuration on
  reset: the hub mass ``mHub`` must be strictly positive and the hub inertia tensor ``IHubPntBc_B``
  must be symmetric and positive definite, raising a descriptive error when the configuration is
  inconsistent (issue #469). Previously a zero hub mass or singular hub inertia silently produced
  ``NaN`` states, and a negative hub mass silently reversed the translational response to applied
  forces.
