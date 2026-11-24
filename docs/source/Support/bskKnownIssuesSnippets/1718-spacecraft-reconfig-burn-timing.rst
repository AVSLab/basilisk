- In Basilisk 1.7.2 through 2.12.2, :ref:`spacecraftReconfig` rounded the per-thruster
  half-burn duration down to whole seconds when checking for ignition (issue #1718).
  This could delay the command and release the burn attitude before the commanded pulse ended.
  A plan created at simulation time zero also ran one task interval late because its clock
  discarded the first interval.
  On affected releases, remove the integer cast on ``thrustOnTime`` in all three burn-start checks
  and change their comparisons from ``<`` to ``<=`` so a tick exactly at ignition fires on time.
  Backport the clock fix to measure from the integer planning epoch. This also prevents
  accumulated roundoff from delaying ignition after repeated fractional-second updates.
  In Basilisk 2.8.0 through 2.12.2, a gap of more than about 104 days between updates
  could also stall an active plan. On those releases, reset the module when reactivating it
  after such a gap to build a fresh plan from the current navigation inputs.
  On affected releases, also reset before resuming if a scheduled burn window has passed.
  Otherwise the module can command the missed impulse at the wrong orbital position.
