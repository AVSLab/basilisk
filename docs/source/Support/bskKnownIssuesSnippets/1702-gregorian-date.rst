- In Basilisk ``2.12.2``, ``astroFunctions.GregorianDate`` can return an incorrect
  month or day near month ends, including day zero and month 13 (#1702).
  The month loop now retains the last day of each month and stops at the
  containing month. The integer ``[year, month, day]`` array interface is
  unchanged. The separate simplified leap-year calculation is also unchanged;
  the routine is limited to dates from 1901 through February 2100.
