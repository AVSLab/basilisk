To include the WMM code, note that the following lines had to be changed to replace

	*(*magneticmodels)[]

With

	*(*magneticmodels)[1]

- MAG_robustReadMagModels()
- MAG_readMagneticModel_SHDF()
- MAG_PrintSHDFFormat()

========
Append 'f' float suffix to each value in 'float GeoidHeightBuffer' in EGM9615.h

========
Local parser patch (issue #1691)

GeomagnetismLibrary.c is a bundled copy of NOAA/NCEI's WMM source code; its
upstream header identifies revision 1521 dated 2017-01-24. Basilisk maintains
local changes to this copy, including the integration changes above.
Upstream software: https://www.ncei.noaa.gov/products/world-magnetic-model
Issue: https://github.com/AVSLab/basilisk/issues/1691

The standard WMM .COF reader has the following Basilisk-local fixes:

- Both parsing passes use MAG_readInteger() to check conversion and range
  before narrowing degree/order values to int, preventing oversized values
  from wrapping into valid indices.
- MAG_readMagneticModel() checks header and coefficient field counts and
  requires 0 <= m <= n <= nMax before writing coefficient arrays.
- MAG_robustReadMagModels() checks the terminator before parsing the degree,
  guards coefficient-count arithmetic and allocation, propagates reader
  failures, and releases the partial model and open file on failure.

These changes leave the magnetic-field equations and coefficients unchanged.
They do not cover the separate Large and SHDF readers.

When importing a newer upstream version, compare these local changes and
retain or reapply them unless upstream provides equivalent validation and
failure handling. Run the module's _UnitTest suite, including
test_magneticFieldWMMParser.py, and src/tests/test_scenarioMagneticFieldWMM.py
to check malformed-input rejection and valid-file numerical results. Record
the imported upstream revision and whether it supersedes this patch here.
