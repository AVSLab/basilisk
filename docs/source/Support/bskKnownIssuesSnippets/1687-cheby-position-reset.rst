- In Basilisk 2.12.2, reconfiguring :ref:`chebyPosEphem` with a different
  coefficient count and resetting it can retain derivative coefficients from
  the previous fit, producing incorrect velocity while position remains correct
  (issue #1687). The fix clears each record's velocity coefficient cache before
  rebuilding it. On affected versions, create a new module instance or explicitly
  zero every record's ``velChebyCoeff`` array before resetting the module.
