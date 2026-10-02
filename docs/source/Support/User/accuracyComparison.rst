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
The results on this page were produced with Orekit 13.1 and GMAT R2026a, and with Basilisk configured as described in
the tables below: the environment modules use the spacecraft state extrapolated to the middle of the step
(``setExtrapolateScStateToStepMidpoint()``), and :ref:`msisAtmosphere` uses the ellipsoidal altitude
(``setPlanetPolarRadius()``) and, where stated, the apparent solar time (``setUseApparentSolarTime()``). The GMAT and Orekit ephemerides are not part
of the repository: :ref:`accuracyComparisonReproduce` explains how to generate them and run the comparison. The
comparison is not part of the automated tests because it needs both tools installed and the run of all cases takes several minutes.

Method
------

All three tools propagate the same nineteen cases, which are defined once in
``benchmarks/accuracyComparison/cases.json``. In the first ten cases the perturbations are added one at a time, so a
disagreement can be traced to a single effect. The other nine extend the comparison to higher-degree gravity, other orbit
types and epochs, real space weather, and attitude-dependent drag and radiation pressure.

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
    * - ``leo_egm70``
      - EGM96 degree and order 70, rotating Earth, same LEO, 7 days
    * - ``sso_drag``
      - Point-mass gravity plus drag in a 700 km sun-synchronous orbit (:math:`i = 98.19^\circ`), epoch 2026-03-20
    * - ``sso_all``
      - Same orbit as ``sso_drag`` with EGM96 20x20, Sun and Moon, solar radiation pressure, and drag
    * - ``molniya_all``
      - EGM96 8x8, Sun and Moon, and solar radiation pressure in a Molniya orbit (:math:`e = 0.74`, :math:`i = 63.4^\circ`), epoch 2026-05-20
    * - ``leo_drag_real_quiet``
      - Point-mass gravity plus drag with the observed space weather of a quiet period (epoch 2024-02-01), 7 days
    * - ``leo_drag_real_storm``
      - Same with the observed space weather of the May 2024 geomagnetic storm (epoch 2024-05-09), 5 days
    * - ``leo_box_fixed``
      - Box spacecraft with a fixed inertial attitude, faceted drag and radiation pressure, 7 days (Orekit reference only)
    * - ``leo_box_spin``
      - Same box spinning at 1.5 deg/s about a fixed body axis, 7 days (Orekit reference only)
    * - ``geo_box_spin``
      - The spinning box in GEO with radiation pressure only, 7 days (Orekit reference only)

.. list-table::
    :widths: 22 78
    :header-rows: 1

    * - Item
      - Setting
    * - Duration, sampling
      - 30 days from 2026-01-01 00:00:00 UTC, one sample every hour. The cases that list another epoch or duration above override these values
    * - Frame
      - Earth-centered inertial EME2000 (``EarthMJ2000Eq`` in GMAT)
    * - Gravity
      - :math:`\mu = 3.986004415\times10^{14}` m\ :sup:`3`/s\ :sup:`2`, equatorial radius 6378136.3 m, fully normalized EGM96 coefficients, identical files for all tools
    * - Earth orientation
      - Zonal-only cases: pole fixed along inertial :math:`+Z`. Rotating cases: ITRF93 in Basilisk (high-precision NAIF Earth kernel and the ITRF93 frame association kernel, requested through ``spicePlanetFrames``; see :ref:`accuracyComparisonReproduce`), ITRF with IERS 2010 conventions in Orekit, GMAT's default Earth orientation
    * - Third bodies
      - Sun and Moon, from DE430 (Basilisk), DE421 (GMAT) and DE440 (Orekit)
    * - Spacecraft
      - 1000 kg, :math:`C_r = 1.5` with 10 m\ :sup:`2`, :math:`C_d = 2.2` with 10 m\ :sup:`2`. The box cases use a 1 m x 2 m x 3 m box without solar arrays, :math:`C_d = 2.2`, absorbed, specular and diffuse fractions of 0.6, 0.2 and 0.2, and the center of pressure at the center of mass, so that no torque changes the spin
    * - Solar radiation pressure
      - Cannonball model, flux 1361 W/m\ :sup:`2` at 1 AU (Basilisk's constant, used in all tools), shadow from the conical Earth model. The occulting body is a sphere of the equatorial radius in all tools (Orekit's default is the oblate Earth, see below)
    * - Atmosphere
      - NRLMSISE-00 with constant :math:`F_{10.7} = 150` and :math:`A_p = 15` (:math:`K_p = 3`), except the two ``real`` cases, which read the same CSSI space-weather file in all tools
    * - Integrators
      - Basilisk: RKF78, 5 s step (1 s for the cases with drag). Orekit: Dormand-Prince 8(5,3), tolerances :math:`10^{-10}` m and :math:`10^{-13}`, maximum step 300 s, 10 s with radiation pressure, and 2 s for the box cases (see below). GMAT: Prince-Dormand 7(8), accuracy :math:`10^{-12}`

Results
-------

The table lists the maximum position difference in meters over the propagation time (30 days, except ``leo_egm70`` with
7 days) between each pair of tools. The ``Basilisk - GMAT`` and ``Basilisk - Orekit`` columns are the quantity of
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
      - 0.41
      - 1.1
      - 1.5
    * - ``gto_all``
      - 0.71
      - 0.96
      - 0.55
    * - ``geo_all``
      - 0.040
      - 0.29
      - 0.27
    * - ``leo_egm70``
      - 0.32
      - 0.89
      - 0.82
    * - ``molniya_all``
      - 4.3
      - 48
      - 52

Without drag, all three tools agree at the meter level over a month, and GMAT and Orekit agree with each other at the same
level as they agree with Basilisk. This includes solar radiation pressure with eclipses (0.4 to 1.5 m). For this the Orekit
reference is set up like the other two tools:

- **Occulting body.** The conical shadow uses a sphere of the equatorial radius, as in Basilisk and GMAT. Orekit's default is the
  oblate Earth, which changes the result of ``leo_srp`` by 178 m and that of ``gto_all`` by 49 m. ``--oblate-shadow`` selects it.
- **Integrator step.** The shadow entry and exit make the force non-smooth, and the adaptive step control of Orekit needs a
  small maximum step to resolve it. The SRP cases use 10 s. With a maximum step of 300 s the difference to Basilisk in
  ``leo_srp`` would be 156 m, with 60 s it is 12 m, and with 10 s 0.4 m. GMAT agrees with Basilisk with its 300 s maximum step.

The agreement does not degrade with the gravity degree: with EGM96 degree and order 70 the three tools agree to better than
1 m after 7 days. ``molniya_all`` agrees with Orekit to 4.3 m, while GMAT is 48 m from Basilisk and 52 m from Orekit. This is not an effect of the
GMAT step (the same 48 m for maximum steps of 300 s, 10 s, and 2 s), and its cause has not been isolated.

.. figure:: /_images/accuracyComparison/accuracyComparison_geo_all.svg
   :align: center

   Position differences for the GEO case with all perturbations.

.. figure:: /_images/accuracyComparison/accuracyComparison_leo_srp.svg
   :align: center

   Position differences for the LEO case with solar radiation pressure only. The three tools agree to 1.5 m.

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
      - 1 500
      - 36 200
    * - ``sso_drag``
      - 2 500
      - 8
      - 3 400
    * - ``sso_all``
      - 2 400
      - 7
      - 3 600

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
  over 30 days, and with Orekit to 1.7 km when the apparent solar time is used.

The drag force itself agrees: at identical states the ratio of the Basilisk to the Orekit drag acceleration equals the
density ratio to within :math:`10^{-5}`, so the relative velocity and the force law are the same. With these settings
the remaining 30-day differences are 0.55 to 1.7 km.

.. figure:: /_images/accuracyComparison/accuracyComparison_leo_drag.svg
   :align: center

   Position differences for the LEO case with drag only. Basilisk and GMAT use the mean solar time and Orekit the apparent
   solar time, so the ``Basilisk - Orekit`` and ``GMAT - Orekit`` curves nearly coincide and show this convention
   difference. ``Basilisk - GMAT`` shows the agreement of the two tools with the same convention.

.. figure:: /_images/accuracyComparison/accuracyComparison_leo_all.svg
   :align: center

   Position differences for the LEO case with all perturbations. As in the previous figure, the two curves that involve
   Orekit show the difference between the apparent solar time of Orekit and the mean solar time of Basilisk and GMAT.

Orbit type and altitude
~~~~~~~~~~~~~~~~~~~~~~~

The sun-synchronous cases at 700 km (``sso_drag`` and ``sso_all``) agree with Orekit to 8 m and 7 m over 30 days when Basilisk uses the
apparent solar time. The difference to GMAT (Basilisk with the mean solar time, as GMAT) is 2.5 km.
The cause is the anomalous oxygen. Above 500 km NRLMSISE-00 provides an effective
total mass density that includes it (``gtd7d``). Basilisk switches to it at 500 km, and so does Orekit. At 121 states of the GMAT trajectory
the GMAT density divided by the density without anomalous oxygen (``gtd7``) is 0.99975 (standard deviation 0.0016), and divided by the
density with it 0.9918 (0.0051), so GMAT's density does not include the anomalous oxygen. At 400 km the two densities are identical.

Real space weather
~~~~~~~~~~~~~~~~~~

In the two ``real`` cases all three tools read the observed F10.7 and A\ :sub:`p` values of the same CSSI file
(``SpaceWeather-All-v1.2.txt``, which GMAT ships). GMAT and Orekit read it directly. Basilisk reads it through
:ref:`spaceWeatherData`, and ``compare_with_basilisk.py`` converts the observed section of the CSSI file to the CelesTrak format
of that module, so that the three tools use the same numbers. The quiet case (epoch 2024-02-01) and the storm case
(epoch 2024-05-09, with the A\ :sub:`p` of 300 of 2024-05-10) are propagated for 7 and 5 days.

.. list-table::
    :widths: 24 19 19 19 19
    :header-rows: 1

    * - Case
      - Basilisk (mean solar time) - GMAT [m]
      - Basilisk (apparent solar time) - Orekit [m]
      - GMAT - Orekit [m]
      - Basilisk (mean solar time, 3-hour Ap) - GMAT [m]
    * - ``leo_drag_real_quiet``
      - 13 000
      - 57
      - 20 100
      - 10 800
    * - ``leo_drag_real_storm``
      - 36 200
      - 250
      - 36 700
      - 9 800

Basilisk with the default daily Ap and Orekit agree to 57 m and 250 m, which includes the storm. At identical states the density of Basilisk with the
apparent solar time divided by Orekit's is 0.9985 (standard deviation 0.0023) in the storm case. GMAT is the outlier, 13 to 36 km from
Basilisk and 20 to 37 km from Orekit, and this is not the solar time convention, because here Basilisk uses GMAT's convention. It is
the Ap input. NRLMSISE-00 can take either the daily Ap or the history of 3-hour Ap values (the second is the storm mode, selected by
switch 9). Orekit and Basilisk (by default) use the daily Ap, GMAT uses the 3-hour history. Basilisk's :ref:`spaceWeatherData` publishes the
3-hour values and :ref:`msisAtmosphere` builds the array from them. By default the module uses the daily Ap, and
``setUseApHistory(True)`` (``--ap-history`` in the script) selects the 3-hour history. In
the storm the daily Ap of 2024-05-10 is 105 and applies from 00:00, although the 3-hour Ap is 12 until the afternoon and the storm
starts at about 15:00. Orekit's and Basilisk's densities jump at that midnight, GMAT's follow the 3-hour values, and the
density ratio of Orekit to GMAT changes between 0.63 and 1.56. With the 3-hour history Basilisk moves toward GMAT: in the storm
the difference falls from 36 km to 9.8 km, and in the quiet case from 13 km to 10.8 km. In the same configuration Basilisk is no longer
close to Orekit, which uses the daily Ap (29 km in the storm, with either solar time convention). The remaining difference to GMAT is not
explained.

.. figure:: /_images/accuracyComparison/accuracyComparison_leo_drag_real_storm.svg
   :align: center

   Position differences for the drag case with the May 2024 geomagnetic storm. Basilisk is shown with both solar time
   conventions; with the apparent solar time it follows Orekit, and GMAT differs from both.

Attitude-dependent drag and radiation pressure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The box cases use :ref:`facetDragDynamicEffector` and :ref:`facetSRPDynamicEffector` in Basilisk and Orekit's
``BoxAndSolarArraySpacecraft`` (without solar array) with a ``FixedRate`` attitude. They are compared with Orekit only:
GMAT's drag model is a cannonball, and its attitude-dependent radiation pressure needs a SPAD file.

.. list-table::
    :widths: 28 36 36
    :header-rows: 1

    * - Case
      - Basilisk (mean solar time) - Orekit [m]
      - Basilisk (apparent solar time) - Orekit [m]
    * - ``leo_box_fixed``
      - 1 460
      - 56
    * - ``leo_box_spin``
      - 970
      - 49
    * - ``geo_box_spin``
      - 0.001
      - (no atmosphere)

With Orekit's step limit of 2 s (see below) the radiation pressure on the spinning box agrees to 1 mm after 7 days in GEO and to
0.7 m in LEO, so the facet model and the attitude handling of the two tools are the same. For the two box cases with drag the
differences are the solar time convention again: with the apparent solar time they drop from 1.5 km and 970 m to 56 m and 48 m, and
the spinning box is as close as the fixed one.

The facets that turn in and out of view make the drag non-smooth, so the box cases use a 2 s maximum step for Orekit. With a
constant density in both tools (no NRLMSISE-00) the difference of the spinning box to Basilisk is 1397 m for a maximum step of 300 s,
1212 m for 60 s, 72 m for 10 s, and 2 m for 2 s, while the fixed attitude does not depend on it (4.7 m). The Basilisk time step
does not matter (1 s and 0.25 s), and the attitude of both tools equals the exact solution of a constant rate to
:math:`10^{-4}` degrees.

.. figure:: /_images/accuracyComparison/accuracyComparison_leo_box_spin.svg
   :align: center

   Position differences between Basilisk and Orekit for the spinning box in LEO with drag and radiation pressure.

Run Time
~~~~~~~~

The reference generators store the wall-clock time of each run next to the ephemerides, and the comparison script prints
it with the Basilisk time. The table lists the times of one run on an Intel Core i7-9700K (3.6 GHz) with Linux, for the
simulated time in the second column.

.. list-table::
    :widths: 30 16 18 18 18
    :header-rows: 1

    * - Case
      - Simulated [days]
      - Basilisk [s]
      - GMAT [s]
      - Orekit [s]
    * - ``leo_twobody``
      - 30
      - 1.6
      - 4.1
      - 0.5
    * - ``leo_zonal6``
      - 30
      - 3.7
      - 3.5
      - 0.7
    * - ``gto_zonal6``
      - 30
      - 3.7
      - 1.7
      - 0.3
    * - ``leo_egm8x8``
      - 30
      - 7.9
      - 2.0
      - 1.7
    * - ``leo_sunmoon``
      - 30
      - 12.1
      - 1.5
      - 1.9
    * - ``leo_srp``
      - 30
      - 7.8
      - 2.1
      - 31.0
    * - ``leo_drag``
      - 30
      - 48.4
      - 6.2
      - 19.0
    * - ``leo_all``
      - 30
      - 104.7
      - 13.2
      - 192.6
    * - ``gto_all``
      - 30
      - 15.9
      - 2.1
      - 46.1
    * - ``geo_all``
      - 30
      - 16.0
      - 1.4
      - 46.0
    * - ``leo_egm70``
      - 7
      - 22.5
      - 9.2
      - 5.0
    * - ``sso_drag``
      - 30
      - 39.1
      - 4.6
      - 17.7
    * - ``sso_all``
      - 30
      - 131.3
      - 11.1
      - 199.2
    * - ``molniya_all``
      - 30
      - 15.6
      - 2.0
      - 48.5
    * - ``leo_drag_real_quiet``
      - 7
      - 9.5
      - 1.9
      - 3.1
    * - ``leo_drag_real_storm``
      - 5
      - 6.9
      - 2.4
      - 2.4
    * - ``leo_box_fixed``
      - 7
      - 13.7
      - n/a
      - 197.6
    * - ``leo_box_spin``
      - 7
      - 13.7
      - n/a
      - 200.1
    * - ``geo_box_spin``
      - 7
      - 1.9
      - n/a
      - 33.5

Basilisk is slower than GMAT in most cases (0.4 to 12 times its time, geometric mean 4.3) and slower than Orekit in the ten cases where Orekit
runs with its default step (2 to 12 times). In the nine cases with radiation pressure, where Orekit needs a 10 s or 2 s maximum step to
resolve the eclipses and the facets (see above), the Orekit time is 6 to 28 times larger than with the default step (31 s instead of 4.9 s for
``leo_srp``) and Basilisk needs 0.06 to 0.7 times as long. This is not a like-for-like efficiency benchmark, for these reasons:

- Basilisk uses a fixed step chosen for accuracy (see the time step study above), while GMAT and Orekit use an adaptive step at
  a given tolerance, so Basilisk takes many more steps in the smooth parts of the orbit. The three tools were not tuned to
  the same accuracy.
- Basilisk runs a full simulation architecture: every module and message is processed at every step, and the environment
  modules, SPICE interface, and recorder are included in its time.
- The Basilisk time is that of ``ExecuteSimulation``. It does not include the module set-up, the initialization, or the SPICE kernel
  loading. The GMAT time is that of the whole ``GmatConsole`` run, which includes its start-up of about 0.24 s, and the Orekit time
  includes the sampling and writing of the ephemeris.
- Each time is a single run, so differences of a few tens of percent are not significant (for example ``sso_drag`` took 48 s and 39 s in two runs).

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
  (``earth_000101_260711_260415.bpc`` was used; any ``earth_*.bpc`` that covers all case epochs and their durations, which
  are 2024-02-01 to 2026-06-19, works when the name is updated in ``compare_with_basilisk.py``) from the
  `NAIF PCK kernels <https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/>`__, and ``earth_assoc_itrf93.tf`` from the
  `NAIF FK kernels <https://naif.jpl.nasa.gov/pub/naif/generic_kernels/fk/planets/>`__. Put both in one folder and pass it with
  ``--kernel-dir``, or in ``data/spice`` next to the scripts.
- **Space weather** (the two ``real`` cases): the CSSI file ``SpaceWeather-All-v1.2.txt``, which GMAT ships in
  ``data/atmosphere/earth``, with observed values for 2024-02-01 to 2024-05-14. Give the same file to both generators
  and to the comparison with ``--weather-file`` (or put it in the data folder).
- ``numpy`` and, for figures, ``matplotlib``.

**2. Generate the reference ephemerides** (the ephemerides are written to ``data/``, which is ignored by git, or to the folder given with ``--output-dir``):

.. code-block:: bash

    python generate_orekit_reference.py /path/to/orekit-data.zip --weather-file /path/to/SpaceWeather-All-v1.2.txt
    python generate_gmat_reference.py /path/to/GMAT/R2026a --weather-file /path/to/SpaceWeather-All-v1.2.txt

Both scripts accept case names after the path to generate a subset. They also write ``orekit_runtime.json`` and
``gmat_runtime.json`` with the wall-clock time of each case, which the comparison prints, so run them on an otherwise idle
machine if the times matter. The cases without GMAT reference (``references`` in ``cases.json``) are skipped by
the GMAT generator. ``generate_orekit_reference.py`` also accepts ``--max-step`` (the maximum integrator step, which ``orekit_max_step_s`` of
each case sets by default) and ``--oblate-shadow``. GMAT stops a propagation within about
:math:`10^{-6}` s of the requested time, so the generator shifts each sample back to the nominal time using the sample's
own velocity.

**3. Run the comparison**

.. code-block:: bash

    python compare_with_basilisk.py --kernel-dir /path/to/kernels                     # all cases, 30 days
    python compare_with_basilisk.py --cases leo_all geo_all --duration-days 5 --kernel-dir /path/to/kernels
    python compare_with_basilisk.py --kernel-dir /path/to/kernels --figures-dir figures      # also save SVG figures
    python compare_with_basilisk.py --cases leo_drag --apparent-solar-time --kernel-dir /path/to/kernels

Use ``--data-dir`` if the reference ephemerides are not in ``data/``. ``--duration-days`` limits the duration of every case
and the run times are then not printed against the stored ones. The script prints the maximum position and velocity
differences of each case between Basilisk, GMAT, and Orekit, and optionally saves a figure of the position difference versus
time. The all-case run takes about fifteen minutes.

**4. What to expect**

The differences should match the tables on this page. Exact values depend on the GMAT, Orekit, and kernel versions, so compare
the level of agreement and not the last digits: sub-meter to meter agreement of Basilisk with both tools without drag, including
gravity of degree 70, and with solar radiation pressure and eclipses, 0.6 km (GMAT) and 1.7 km (Orekit with ``--apparent-solar-time``)
in the 400 km drag cases, 2.5 km to GMAT and 8 m to Orekit at 700 km, 13 to 36 km between GMAT and the other tools with real space
weather (10 km to GMAT in the storm with ``--ap-history``), and 50 m (apparent solar time) for the box cases with drag.
The 48 m of GMAT in ``molniya_all`` is not explained. The run times depend on the machine.

Limitations
-----------

The attitude-dependent cases use a constant-rate spin without torque and are compared with Orekit only. The real space weather
cases use observed values; forecast data are not compared. Because GMAT, Orekit, and Basilisk implement NRLMSISE-00 independently,
the drag comparison also reflects differences in how each tool prepares the model inputs. :ref:`msisAtmosphere` uses the daily Ap index
by default, and the 3-hour Ap history that :ref:`spaceWeatherData` provides only with ``setUseApHistory()`` (see the section on real space weather).
