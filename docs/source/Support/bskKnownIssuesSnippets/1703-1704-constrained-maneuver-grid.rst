- In Basilisk ``2.12.2``, :ref:`constrainedAttitudeManeuver` can overrun fixed arrays or use
  uninitialized endpoints when the grid has no free node (#1703, #1704). On that release, set
  ``2 <= N <= 20`` before each grid build, limit each boresight list to ten entries, and check
  that the grid has distinct, usable endpoints. The planner now validates these limits and
  reports endpoint selection failures before search.

- In ``2.12.2``, an incomplete search, stale inputs, or an MRP shadow change can produce a partial,
  nonfinite, or unnecessarily long reference. Resetting after time zero can skip part of the maneuver.
  On that release, start at simulation time zero, supply written inputs, and verify the final attitude,
  endpoint rates, and angular travel before using the reference. The planner now checks search completion
  and fitted motion and times the maneuver from reset. Cone clearance between waypoints still requires
  a separate check, as described in the :ref:`constrainedAttitudeManeuver` User Guide.
  The default distance heuristic can still select a longer grid path. Set ``costFcnType = 0`` and
  ``useShortestPath = True`` to obtain minimum accumulated MRP edge distance on the generated graph,
  at the cost of potentially more node expansions (#1726). This option does not minimize fitted
  control effort or guarantee continuous cone clearance.

- In ``2.12.2``, invalid :ref:`BSpline` inputs or a very short final knot span can yield an incorrect
  fit, and an empty Python sequence passed to an Eigen vector setter can terminate Python. On that
  release, use finite waypoint coordinates, strictly increasing times starting at zero, and enough
  independent weighted waypoints to determine the fit. Avoid very short final knot spans and empty
  Eigen inputs. The fitters now check their inputs, the evaluator respects the final knot span, and
  the Python bindings handle empty vectors.
