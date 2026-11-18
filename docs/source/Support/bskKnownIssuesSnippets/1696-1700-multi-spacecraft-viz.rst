- In Basilisk ``2.12.2``, :ref:`dataFileToViz` can crash when a spacecraft with an
  empty thruster cluster map precedes one with thrusters (#1696). Empty count
  vectors now retain their spacecraft index. Cluster and count vectors must
  have equal lengths, and counts must be nonnegative. For spacecraft without
  thrusters, pass both an empty cluster vector and an empty count vector.
- In Basilisk ``2.12.2``, :ref:`dataFileToViz` assigns later spacecraft thruster
  and reaction-wheel configuration entries from the beginning of the shared
  lists (#1700). Positions, directions, and limits can therefore be incorrect
  even though the recorded thrust, wheel speed, and motor torque are read
  correctly. Each spacecraft now uses its own configuration entries. Regenerate
  affected Vizard files with the corrected module.
