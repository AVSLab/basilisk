- GitHub issue 1681: In tagged releases 2.1.4 through 2.12.2, polyhedral gravity
  summed the absolute values of face tetrahedral volumes. For a closed,
  outward-oriented body that is not star-shaped about its center of mass, this
  overestimates the volume and scales both acceleration and potential by the
  ratio of the true volume to the overestimated volume. For example, the
  fourteen-cube C-shaped body in the issue produces 14/18 of the correct field.
  The corrected :ref:`polyhedralGravityModel` sums signed contributions and
  rejects non-finite or non-positive totals. Initialization also clears cached
  facet and edge dyads on each attempt, allowing corrected meshes to be retried
  without retaining invalid or outdated gravity data. Consistently
  inward-oriented meshes
  must have their facet winding reversed before initialization. On affected
  versions, use a mesh that is star-shaped about its center of mass or upgrade
  to a version containing this fix. The shipped ``eros007790.tab`` mesh is
  unaffected by the volume correction.
