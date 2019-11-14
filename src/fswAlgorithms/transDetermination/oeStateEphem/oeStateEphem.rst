
This module takes the TDB time, current object time and computes the state of the object using the time corrected by TDB and the stored Chebyshev coefficients.

If the time provided is outside the specified range for which the stored Chebyshev coefficients are valid then the position vectors rail high/low appropriately.  More information can be found in the
:download:`PDF Description </../../src/fswAlgorithms/transDetermination/oeStateEphem/_Documentation/Basilisk-oeStateEphem-20190426.pdf>`.

