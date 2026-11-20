- In Basilisk ``2.12.2``, ``astroFunctions.JulianDate`` rounds calendar dates up
  to noon, and ``GregorianDate`` can return day zero or month 13 near month ends
  (#1702). Both ``GregorianDate`` and ``exactGregorianDate`` also count 1900
  and 2100 as leap years. All three helpers now use
  Gregorian leap-year rules and count fractional calendar days from midnight.
  Callers that intended noon should pass ``day + 0.5`` to ``JulianDate``.
