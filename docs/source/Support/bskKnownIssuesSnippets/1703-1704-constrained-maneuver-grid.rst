- In Basilisk ``2.12.2``, :ref:`constrainedAttitudeManeuver` writes beyond its
  grid-coordinate array when ``N > 20``; ``N = 1`` also causes division by zero
  (#1703). Grid generation now rejects refinement levels outside ``2 <= N <= 20``,
  including changes after construction. Node and neighbor lists now raise errors
  before exceeding their fixed capacities. Use a supported refinement level;
  the checks do not increase the planner's storage limits.
- In Basilisk ``2.12.2``, :ref:`constrainedAttitudeManeuver` can use uninitialized
  endpoint keys when no grid node satisfies the constraints (#1704). Initialization
  now raises an error before graph search in this case. Unit-vector dot products
  are clamped to ``[-1, 1]`` so roundoff cannot misclassify antipodal nodes for
  180-degree keep-in or keep-out cones. Repeated grid generation
  also clears the previous path and grid to avoid retaining old constraints or
  accumulating duplicate neighbors. Goal links now reference the stored node
  instead of a temporary copy, and effort evaluation no longer overwrites the
  configured refinement level. If no free grid node is found, review the
  constraints or try another supported refinement level; this sampling failure
  does not establish that every continuous maneuver is infeasible.
