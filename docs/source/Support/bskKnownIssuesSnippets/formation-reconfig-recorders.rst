- In Basilisk 2.7.0 through 2.12.2, :ref:`scenarioFormationReconfig` could repeatedly
  warn about an unwritten thruster command before the first burn. Its attitude-error plot
  also began with an unwritten sample and then showed the previous flight software update.
  Remove the unused thruster-command and attitude-reference recorders and schedule the
  attitude-error recorder on the flight software task after :ref:`attTrackingError`.
