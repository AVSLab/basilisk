- GitHub issue 1671: In Basilisk 2.10.0 through 2.12.2, :ref:`linkBudget` could classify unwritten
  antenna messages as a space-to-space link during reset, disable atmospheric
  attenuation, and apply an erroneous ground-antenna pointing loss. Separately,
  :ref:`simpleAntenna` did not forward the ground surface normal needed for
  atmospheric slant paths, so attenuation did not vary with elevation.
  Publishing antenna outputs during reset alone could also cache sea-level
  conditions before an elevated ground station published its position. The
  modules now forward the surface normal and initialize the link on the first
  update with written antenna inputs and, when atmosphere is enabled, a nonzero
  ground position. Schedule ground and spacecraft producers before antennas,
  and antennas before the link budget. For older versions, explicitly populate
  the normal and initialize the link only after all upstream producers have
  published their states. See :ref:`linkbudget-initialization` for compatibility
  changes to output publication during reset and the timing of initialization
  errors.
