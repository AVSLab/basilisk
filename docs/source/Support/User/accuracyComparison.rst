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
The results on this page were produced with Orekit 13.1 and GMAT R2026a, and with a Basilisk version that includes the
changes of issue #1456: the rotation-based planet orientation in the gravity effector, the
spacecraft state timing of the environment modules, and the optional ellipsoidal altitude and apparent solar time of
:ref:`msisAtmosphere`. The GMAT and Orekit ephemerides are not part
of the repository: :ref:`accuracyComparisonReproduce` explains how to generate them and run the comparison. The
comparison is not part of the automated tests because it needs both tools installed and the 30-day run of all cases takes
a few minutes.

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
      - Zonal-only cases: pole fixed along inertial :math:`+Z`. Rotating cases: ITRF93 in Basilisk (high-precision NAIF Earth kernel and the ITRF93 frame association kernel, requested through ``spicePlanetFrames``; see :ref:`accuracyComparisonReproduce`), ITRF with IERS 2010 conventions in Orekit, GMAT's default Earth orientation
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

The cases with drag are listed separately because GMAT and Basilisk, by default, and Orekit use different conventions for
the local solar time of NRLMSISE-00 (see below). The Basilisk column compared with GMAT uses the default mean solar time
and the one compared with Orekit uses the apparent solar time. Both use the ellipsoidal altitude and a 1 s time step.

In the two drag figures below Basilisk uses the default mean solar time, as GMAT does, and only Orekit uses the apparent solar
time. The ``Basilisk - Orekit`` and ``GMAT - Orekit`` curves therefore almost coincide (after 30 days 41.2 km and 40.7 km, with a
ratio between 1.01 and 1.015 over the whole run): both show the difference between the mean and the apparent solar time and
not an error of either tool. The ``Basilisk - GMAT`` curve is the actual agreement of the two tools that use the same convention.
With the apparent solar time, Basilisk and Orekit differ by only 1.7 km after 30 days (the table above and
``--apparent-solar-time``).

.. list-table::
    :widths: 28 24 24 24
    :header-rows: 1

    * - Case
      - Basilisk (mean solar time) - GMAT [m]
      - Basilisk (apparent solar time) - Orekit [m]
      - GMAT - Orekit [m]
    * - ``leo_drag``
      - 550
      - 1 700
      - 40 700
    * - ``leo_all``
      - 630
      - 1 700
      - 36 000

Drag
~~~~

Drag differences between tools are far larger in absolute terms because drag shortens the orbit.
All three tools use the same drag law, cross-section, drag
coefficient, mass, and co-rotating atmosphere. The remaining differences come from three settings of the atmosphere:

- **Altitude definition.** NRLMSISE-00 needs the geodetic latitude and the altitude above the reference
  ellipsoid. Orekit and GMAT use them. :ref:`msisAtmosphere` uses a sphere unless the planet polar radius
  is set with ``setPlanetPolarRadius()``. The comparison sets the polar radius.
- **Local solar time convention.** Orekit's NRLMSISE-00 computes the local apparent solar time from the Sun position.
  GMAT and Basilisk by default use the mean solar time (second of day plus longitude divided by 15 degrees per hour). The two
  differ by the equation of time, which varies between about -14 and +16 minutes over the year, and over this 30-day
  period changes the density by up to 3% (1.1% rms). At 721 identical states along the 30-day GMAT trajectory the ratio of the
  GMAT to Orekit density has a scatter of 1.1%, that of Basilisk with the mean solar time to GMAT only 0.034%, and that of
  Basilisk with the apparent solar time to Orekit 0.003%. This is the whole 41 km
  difference between GMAT and Orekit. Use ``setUseApparentSolarTime()`` to match Orekit; the script option
  ``--apparent-solar-time`` enables it.
- **Time step.** The atmosphere and drag evaluate once per integration step, which is 38 km of motion at a 5 s step. The
  resulting error is second order in the step: the 10-day difference between Basilisk (apparent solar time) and Orekit
  is 19.3 km at 20 s, 5.0 km at 10 s, 1.35 km at 5 s, 0.45 km at 2.5 s and 0.20 km at 1 s. The cases with drag
  therefore use a 1 s step, with which Basilisk agrees with GMAT to 0.55 km (``leo_drag``) and 0.63 km (``leo_all``)
  over 30 days, 0.004% of the drag signal, and with Orekit to 1.7 km when the apparent solar time is used.

The drag force itself agrees: at identical states the ratio of the Basilisk to the Orekit drag acceleration equals the
density ratio to within :math:`10^{-5}`, so the relative velocity and the force law are the same. With these settings
the remaining 30-day differences are 0.55 to 1.7 km, which is 0.004% to 0.013% of the drag signal.

.. figure:: /_images/accuracyComparison/accuracyComparison_leo_drag.svg
   :align: center

   Position differences for the LEO case with drag only. Basilisk and GMAT use the mean solar time and Orekit the apparent
   solar time, so the ``Basilisk - Orekit`` and ``GMAT - Orekit`` curves nearly coincide and show this convention
   difference. ``Basilisk - GMAT`` shows the agreement of the two tools with the same convention.

.. figure:: /_images/accuracyComparison/accuracyComparison_leo_all.svg
   :align: center

   Position differences for the LEO case with all perturbations. As in the previous figure, the two curves that involve
   Orekit show the difference between the apparent solar time of Orekit and the mean solar time of Basilisk and GMAT.

.. _accuracyComparisonReproduce:

Reproducing the Results
-----------------------

All commands are run from ``benchmarks/accuracyComparison`` of the Basilisk repository.

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
time. The all-case run takes about six minutes.

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
