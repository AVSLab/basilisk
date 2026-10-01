.. _accuracyComparison:

Accuracy Comparison with GMAT and Orekit
========================================

This page documents how Basilisk's orbit propagation compares with two widely used
astrodynamics tools, `GMAT <https://gmat.atlassian.net/wiki/spaces/GW/overview>`__ and
`Orekit <https://www.orekit.org>`__. It is intended to help users who need to show
that Basilisk results agree with an established reference before relying on them.

The comparison is reproduced by the scripts in ``benchmarks/accuracyComparison``:
:download:`compare_with_basilisk.py <../../../../benchmarks/accuracyComparison/compare_with_basilisk.py>`,
the reference generators
:download:`generate_gmat_reference.py <../../../../benchmarks/accuracyComparison/generate_gmat_reference.py>` and
:download:`generate_orekit_reference.py <../../../../benchmarks/accuracyComparison/generate_orekit_reference.py>`,
and the case definitions :download:`cases.json <../../../../benchmarks/accuracyComparison/cases.json>`.
The results on this page were produced with Orekit 13.1 and GMAT R2026a. The GMAT and Orekit ephemerides are not part
of the repository: :ref:`accuracyComparisonReproduce` explains how to generate them and run the comparison. The
comparison is not part of the automated tests because it needs both tools installed and the 30-day run of all cases takes
about two minutes.

Method
------

All three tools propagate the same ten cases, which are defined once in
``benchmarks/accuracyComparison/cases.json``. The perturbations are added one at a time, so a
disagreement can be traced to a single effect.

.. list-table::
    :widths: 22 78
    :header-rows: 1

    * - Case
      - Force model
    * - ``leo_twobody``
      - Point-mass gravity, inclined LEO (:math:`a = 6778` km, :math:`i = 51.6^\circ`)
    * - ``leo_zonal6``
      - Zonal harmonics :math:`J_2` to :math:`J_6`, same LEO, Earth pole fixed along inertial :math:`+Z`
    * - ``gto_zonal6``
      - Same as ``leo_zonal6``, in a GTO (:math:`r_p = 6572` km, :math:`e = 0.73`, :math:`i = 28.5^\circ`)
    * - ``leo_egm8x8``
      - EGM96 degree and order 8, rotating Earth
    * - ``leo_sunmoon``
      - Point-mass gravity plus Sun and Moon third-body perturbations
    * - ``leo_srp``
      - Point-mass gravity plus cannonball solar radiation pressure with Earth shadow
    * - ``leo_drag``
      - Point-mass gravity plus NRLMSISE-00 drag with a co-rotating atmosphere
    * - ``leo_all``
      - EGM96 8x8, Sun and Moon, solar radiation pressure, and drag
    * - ``gto_all``
      - EGM96 8x8, Sun and Moon, and solar radiation pressure (no drag)
    * - ``geo_all``
      - Same as ``gto_all``, in a near-equatorial GEO orbit

.. list-table::
    :widths: 22 78
    :header-rows: 1

    * - Item
      - Setting
    * - Duration, sampling
      - 30 days from 2026-01-01 00:00:00 UTC, one sample every hour
    * - Frame
      - Earth-centered inertial EME2000 (``EarthMJ2000Eq`` in GMAT)
    * - Gravity
      - :math:`\mu = 3.986004415\times10^{14}` m\ :sup:`3`/s\ :sup:`2`, equatorial radius 6378136.3 m, fully normalized EGM96 coefficients, identical files for all tools
    * - Earth orientation
      - Zonal-only cases: pole fixed along inertial :math:`+Z`. Rotating cases: ITRF93 in Basilisk (high-precision NAIF kernel), ITRF with IERS 2010 conventions in Orekit, GMAT's default Earth orientation
    * - Third bodies
      - Sun and Moon, from DE430 (Basilisk), DE421 (GMAT) and DE440 (Orekit)
    * - Spacecraft
      - 1000 kg, :math:`C_r = 1.5` with 10 m\ :sup:`2`, :math:`C_d = 2.2` with 10 m\ :sup:`2`
    * - Solar radiation pressure
      - Cannonball model, flux 1361 W/m\ :sup:`2` at 1 AU (Basilisk's constant, used in all tools), shadow from the conical Earth model
    * - Atmosphere
      - NRLMSISE-00 with constant :math:`F_{10.7} = 150` and :math:`A_p = 15` (:math:`K_p = 3`)
    * - Integrators
      - Basilisk: RKF78, 5 s step (1 s for the cases with drag). Orekit: Dormand-Prince 8(5,3), tolerances :math:`10^{-10}` m and :math:`10^{-13}`. GMAT: Prince-Dormand 7(8), accuracy :math:`10^{-12}`

Results
-------

The table lists the maximum position difference in meters over the 30 days between each pair
of tools. The ``Basilisk - GMAT`` and ``Basilisk - Orekit`` columns are the quantity of
interest. ``GMAT - Orekit`` shows how well the two reference tools agree with each other and
is the natural yardstick for the other columns.

.. list-table::
    :widths: 28 24 24 24
    :header-rows: 1

    * - Case
      - Basilisk - Orekit [m]
      - Basilisk - GMAT [m]
      - GMAT - Orekit [m]
    * - ``leo_twobody``
      - 0.017
      - 0.29
      - 0.27
    * - ``leo_zonal6``
      - 0.018
      - 0.24
      - 0.23
    * - ``gto_zonal6``
      - 0.23
      - 0.36
      - 0.58
    * - ``leo_egm8x8``
      - 2.2
      - 1.5
      - 2.3
    * - ``leo_sunmoon``
      - 0.018
      - 0.28
      - 0.26
    * - ``leo_srp``
      - 24
      - 1.1
      - 25
    * - ``leo_drag``
      - 41 000 (1 700 with apparent solar time)
      - 550
      - 40 700
    * - ``leo_all``
      - 35 000 (1 700 with apparent solar time)
      - 630
      - 36 000
    * - ``gto_all``
      - 62
      - 0.96
      - 61
    * - ``geo_all``
      - 0.039
      - 0.29
      - 0.27

Without drag, Basilisk agrees with GMAT at the meter level or better over a month, and with Orekit at the
meter level or better except for solar radiation pressure in orbits that enter the Earth's shadow, where Orekit
differs from both Basilisk and GMAT by 24 to 62 m; the reason has not been isolated. GMAT and Orekit agree with each
other at the same level as they agree with Basilisk in the other cases.

.. figure:: /_images/accuracyComparison/accuracyComparison_geo_all.svg
   :align: center

   Position differences for the GEO case with all perturbations.

.. figure:: /_images/accuracyComparison/accuracyComparison_leo_srp.svg
   :align: center

   Position differences for the LEO case with solar radiation pressure only. Basilisk and GMAT agree, and Orekit
   differs from both.

Drag
~~~~

Drag differences between tools are far larger in absolute terms because drag shortens the orbit: the drag signal alone is
13 000 km of position difference after 30 days, so a density bias of 0.1% changes the position by 26 km (measured by
scaling the drag coefficient; the response is linear). All three tools use the same drag law, cross-section, drag
coefficient, mass, and co-rotating atmosphere. The differences in the table were traced to three causes, each measured:

- **Altitude definition.** NRLMSISE-00 needs the geodetic latitude and the altitude above the reference
  ellipsoid. Orekit and GMAT use them. Basilisk's :ref:`msisAtmosphere` uses a sphere unless the planet polar radius
  is set with ``setPlanetPolarRadius()``. On a sphere the altitude is wrong by up to 21 km, which changes the density
  by +19% at 45 degrees latitude and +44% at the pole (400 km altitude), and gave 11.6 km of position difference to GMAT
  over two days against 47 m with the ellipsoid. The comparison script sets the polar radius.
- **Local solar time convention.** Orekit's NRLMSISE-00 computes the local apparent solar time from the Sun position.
  GMAT and Basilisk use the mean solar time (second of day plus longitude divided by 15 degrees per hour). The two differ
  by the equation of time, which varies between about -14 and +16 minutes over the year, and over this 30-day
  period changes the density by up to 3% (1.1% rms). At 721 identical states along the 30-day GMAT trajectory the ratio of the
  GMAT to Orekit density has a scatter of 1.1%, that of Basilisk with the mean solar time to GMAT only 0.034%
  (mean ratio 0.99998), and that of Basilisk with the apparent solar time to Orekit 0.003%. This is the whole 41 km difference between GMAT and
  Orekit. Basilisk uses the mean solar time by default and the apparent one with ``setUseApparentSolarTime()``; the script
  option ``--apparent-solar-time`` enables it to match Orekit.
- **Basilisk time step.** The atmosphere and drag evaluate once per integration step, which is 38 km of motion at a 5 s step. The
  resulting error is second order in the step: the 10-day difference between Basilisk (apparent solar time) and Orekit
  falls from 19.3 km at 20 s to 5.0 km at 10 s, 1.35 km at 5 s, 0.45 km at 2.5 s and 0.20 km at 1 s. The cases with drag
  therefore use a 1 s step, with which Basilisk agrees with GMAT to 0.55 km (``leo_drag``) and 0.63 km (``leo_all``)
  over 30 days, 0.004% of the drag signal, and with Orekit to 1.7 km when the apparent solar time is used.

The drag force itself agrees: at identical states the ratio of the Basilisk to the Orekit drag acceleration equals the
density ratio to within :math:`10^{-5}`, so the relative velocity and the force law are the same. With these three points accounted for, the remaining
30-day differences are 0.55 to 1.7 km, which is 0.004% to 0.013% of the drag signal.

.. figure:: /_images/accuracyComparison/accuracyComparison_leo_drag.svg
   :align: center

   Position differences for the LEO case with drag only. The Basilisk and GMAT curves use the mean solar time.

.. figure:: /_images/accuracyComparison/accuracyComparison_leo_all.svg
   :align: center

   Position differences for the LEO case with all perturbations.

Basilisk Behaviours Found by the Comparison
-------------------------------------------

The comparison exposed several Basilisk behaviours that a user comparing with another tool would otherwise have to work
around. They are fixed in the release that contains this page; see the release notes.

- **Planet orientation extrapolation in the gravity effector.** The orientation was advanced with a first-order update of
  the matrix elements, which is not orthonormal and scales the field by a relative error of order
  :math:`(\omega\,\Delta t)^2`. A two-body orbit about a SPICE-driven Earth drifted by 30 m per day at a 5 s step, and the error fell by a
  factor of four each time the step was halved. The orientation is now advanced as a rotation.
- **One-step lag of the environment modules.** The atmosphere, wind, magnetic field, eclipse, and solar flux modules read
  the spacecraft state written at the end of the previous step. At shadow entries and exits this made the eclipse factor a step late; the two
  lags are equal and opposite in force but at different positions, so they accumulate. The solar radiation pressure error in LEO was 533 m
  at a 5 s step and scaled with the step. The spacecraft position is now advanced to the middle of the interval the next spacecraft update
  integrates, which reduced it to 1 m.
- **Atmosphere altitude and local solar time.** See the drag section above, and :ref:`atmosphereBase` and :ref:`msisAtmosphere`.
- **High-precision Earth orientation.** The default ``IAU_EARTH`` frame of the SPICE interface has no nutation or polar motion.
  For rotating-Earth cases the script loads the high-precision Earth kernel and the ITRF93 frame association kernel, and
  requests ``ITRF93`` through ``spicePlanetFrames``. This halved the error of the EGM96 8x8 case. The kernels are not part of
  Basilisk's support-data registry. The script searches for ``earth_000101_260711_260415.bpc`` and ``earth_assoc_itrf93.tf`` in the folder given
  with ``--kernel-dir``, in ``benchmarks/accuracyComparison/data/spice``, and in the Basilisk support-data cache.

.. _accuracyComparisonReproduce:

Reproducing the Results
-----------------------

All commands are run from ``benchmarks/accuracyComparison`` of the Basilisk repository, with a Basilisk build that
includes the changes listed above.

**1. Prerequisites**

- **Orekit** (reference generator): a Java runtime, the Python package ``orekit_jpype`` (``pip install orekit-jpype``), and the
  ``orekit-data.zip`` file, which can be downloaded with ``orekit_jpype.pyhelpers.download_orekit_data_curdir()`` or from the
  `Orekit data repository <https://gitlab.orekit.org/orekit/orekit-data>`__. The results use Orekit 13.1.
- **GMAT** (reference generator): an installation of GMAT R2026a. The generator runs ``bin/GmatConsole`` from the installation folder.
- **Earth orientation kernels** (Basilisk, rotating-Earth cases): the NAIF high-precision Earth PCK
  (``earth_000101_260711_260415.bpc`` was used; any ``earth_*.bpc`` that covers 2026-01-01 to 2026-01-31 works when the
  name is updated in ``compare_with_basilisk.py``) from the
  `NAIF PCK kernels <https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/>`__, and ``earth_assoc_itrf93.tf`` from the
  `NAIF FK kernels <https://naif.jpl.nasa.gov/pub/naif/generic_kernels/fk/planets/>`__. Put both in one folder and pass it with
  ``--kernel-dir``, or in ``data/spice`` next to the scripts.
- ``numpy`` and, for figures, ``matplotlib``.

**2. Generate the reference ephemerides** (each takes less than a minute; the ephemerides are written to ``data/``, which is
ignored by git, or to the folder given with ``--output-dir``):

.. code-block:: bash

    python generate_orekit_reference.py /path/to/orekit-data.zip
    python generate_gmat_reference.py /path/to/GMAT/R2026a

Both scripts accept case names after the path to generate a subset. GMAT stops a propagation within about
:math:`10^{-6}` s of the requested time, so the generator shifts each sample back to the nominal time using the sample's
own velocity.

**3. Run the comparison**

.. code-block:: bash

    python compare_with_basilisk.py --kernel-dir /path/to/kernels                     # all cases, 30 days
    python compare_with_basilisk.py --cases leo_all geo_all --duration-days 5 --kernel-dir /path/to/kernels
    python compare_with_basilisk.py --kernel-dir /path/to/kernels --figures-dir figures      # also save SVG figures
    python compare_with_basilisk.py --cases leo_drag --apparent-solar-time --kernel-dir /path/to/kernels

Use ``--data-dir`` if the reference ephemerides are not in ``data/``. The script prints the maximum position and velocity
differences of each case between Basilisk, GMAT, and Orekit, and optionally saves a figure of the position difference versus
time. The all-case run takes about two minutes.

**4. What to expect**

The differences should match the tables on this page. Exact values depend on the GMAT, Orekit, and kernel versions, so compare
the level of agreement and not the last digits: sub-meter to meter agreement of Basilisk with both tools without drag, the
24 to 62 m Orekit difference in the cases with solar radiation pressure and eclipses, and 0.6 km (GMAT) and 1.7 km (Orekit with
``--apparent-solar-time``) in the drag cases.

Limitations
-----------

Only the force models listed above are compared. Tides, relativistic corrections, albedo, and
spacecraft attitude effects are not covered. The atmosphere comparison uses constant space weather
and a cannonball drag model.
