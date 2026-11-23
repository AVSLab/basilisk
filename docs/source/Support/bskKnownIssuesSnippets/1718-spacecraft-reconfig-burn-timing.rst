- In Basilisk 1.7.2 through 2.12.2, :ref:`spacecraftReconfig` rounded the per-thruster
  half-burn duration down to whole seconds when checking for ignition (issue #1718).
  This could delay the command and release the burn attitude before the commanded pulse ended.
  On affected releases, remove the integer cast on ``thrustOnTime`` in all three burn-start checks
  to use the same fractional duration as the attitude-release checks.
