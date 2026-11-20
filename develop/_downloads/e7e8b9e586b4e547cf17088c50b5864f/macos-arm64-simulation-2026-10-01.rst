Simulation and optional binding experiment
===========================================

Measurements taken on 2026-10-01 with the prototype applied to Basilisk
commit ``f539f81d5d``. See the accompanying
`raw measurements <macos-arm64-simulation-2026-10-01.json>`_.
Timings, module counts, and validation results describe that prototype snapshot.

Packaging
---------

The experiment adds three independently loaded native containers:

- 110 core simulation bindings in ``Basilisk.simulation._simulationCoreNative``.
- All 16 MuJoCo bindings in ``Basilisk.simulation._mujocoNative``.
- All four OpenCV bindings (``camera``, ``centerRadiusCNN``, ``houghCircles``,
  and ``limbFinding``) in ``Basilisk._opNavNative``. This container spans the
  simulation and FSW Python packages and supports the optional ``bsk-opnav`` wheel.

Public Python imports, constructors, messages, and recorders retain their existing
APIs. Each private extension keeps its original initialization entry point and is
initialized only when imported. MuJoCo public aliases retain their existing names.
The core simulation container does not link MuJoCo or OpenCV libraries. Vizard,
external modules, and Rust bindings retain their existing packaging.

Cold imports and builds
-----------------------

.. list-table:: Median of three trials, seconds
   :header-rows: 1

   * - Group
     - Separate cold import
     - Combined cold import
     - Separate rebuild
     - Combined rebuild
   * - Core simulation
     - 12.432
     - 1.212
     - 53.734
     - 55.977
   * - MuJoCo
     - 2.004
     - 0.603
     - 10.793
     - 10.420
   * - OpenCV
     - 0.557
     - 0.241
     - 4.924
     - 4.917

Each full-group rebuild regenerates every selected SWIG wrapper and recompiles
all of that group's objects: 230 for simulation, 30 for MuJoCo, and eight for
OpenCV. Shared implementation libraries and third-party dependencies are already
built. These are binding-group rebuild times, not whole-project clean-build times.
The aggregate's empty Xcode anchor is already compiled before each timed rebuild.

The simulation median increased by about 2.2 seconds (4.2%); its trial ranges
overlap. MuJoCo and OpenCV show no build-time penalty in this small sample.

.. list-table:: Trial ranges, seconds
   :header-rows: 1

   * - Group and layout
     - Cold import range
     - Rebuild range
   * - Core simulation, separate
     - 12.143 to 12.471
     - 53.218 to 54.545
   * - Core simulation, combined
     - 1.166 to 1.339
     - 53.397 to 56.441
   * - MuJoCo, separate
     - 1.960 to 2.097
     - 10.619 to 11.289
   * - MuJoCo, combined
     - 0.495 to 0.604
     - 10.333 to 10.451
   * - OpenCV, separate
     - 0.501 to 0.606
     - 4.889 to 4.952
   * - OpenCV, combined
     - 0.211 to 0.352
     - 4.887 to 4.918

.. list-table:: Warm import medians and native container sizes
   :header-rows: 1

   * - Group
     - Separate warm (s)
     - Combined warm (s)
     - Separate size (MiB)
     - Combined size (MiB)
   * - Core simulation
     - 0.294
     - 0.153
     - 44.63
     - 30.10
   * - MuJoCo
     - 0.285
     - 0.242
     - 3.15
     - 2.03
   * - OpenCV
     - 0.018
     - 0.015
     - 24.98
     - 10.32

Incremental builds
------------------

.. list-table:: Median of three trials, seconds including build-system overhead
   :header-rows: 1

   * - Representative module
     - Separate C++ edit
     - Combined C++ edit
     - Separate interface edit
     - Combined interface edit
   * - spacecraft
     - 3.764
     - 3.884
     - 5.496
     - 5.338
   * - thrOnTimeToForce
     - 3.084
     - 3.040
     - 4.489
     - 4.524
   * - camera
     - 3.467
     - 3.486
     - 4.791
     - 4.732

Every measured edit compiled exactly one object and relinked one library. An
interface edit also regenerated exactly one SWIG wrapper. All unchanged builds
performed zero compilations, zero SWIG generations, and zero links. Changing one
binding therefore does not recompile its neighbors, but does replace the combined
library and can incur its first-load validation again.

Sparse imports
--------------

.. list-table:: One cold-import sample per layout, seconds
   :header-rows: 1

   * - Public module
     - Separate
     - Combined
   * - spacecraft
     - 0.479
     - 0.297
   * - thrOnTimeToForce
     - 0.048
     - 0.077
   * - camera
     - 0.114
     - 0.219

A sparse script can pay more to open a larger container even though unrelated
bindings remain uninitialized. The optional single-module samples demonstrate
that tradeoff; they are not repeated estimates of a typical application's latency.

Pytest startup
--------------

.. list-table:: One comparison with 16 workers, seconds until collection completes
   :header-rows: 1

   * - State
     - Separate simulation
     - Combined simulation
   * - All project native outputs freshly relinked
     - 27.776
     - 21.108
   * - Subsequent warm run
     - 15.731
     - 16.729

FSW, MuJoCo, and OpenCV grouping remained enabled in both cases. Only the core
simulation layout changed. Both layouts collected 8,421 CI-selected tests and
initialized the same 397 Basilisk native modules per worker. Their distinct loaded
native files decreased from 137 to 28. Cold collection improved by about 24%;
there was no warm-collection improvement in this single pair.

The profiler clears collected items before execution. These runs measure startup,
not test execution, and exit code 5 is expected. The archived JSON retains per-worker
readiness and native-load totals without machine-specific absolute file paths.

Validation
----------

- Simulation, image-processing, import compatibility, and optional-wheel regression
  tests: 4,221 passed, including all 152 MuJoCo tests.
- Example scenarios: 270 passed.
- Final binding, packaging, cleanup, and benchmark regressions: 64 passed,
  covering Ninja, Unix Makefiles, and Xcode and unchanged reconfiguration of
  every group.
- Disabling MuJoCo and OpenCV removes their containers, loaders, and private
  shims without changing the core simulation library. Restoring both features
  rebuilds the optional containers successfully.
- Built and repaired a wheel containing all 130 new grouped bindings, installed
  it outside the checkout, and passed 19 installed-package tests, including a
  MuJoCo simulation with Python callbacks across native bindings.

Method and limits
-----------------

macOS 27.2 arm64; Python 3.14.6; CMake 4.3.2 with Ninja; AppleClang 21;
Release; strict warnings, OpenCV, MuJoCo, visualization, and Rust enabled.
Build parallelism: 12. Each experiment changes only its selected group's layout;
all other groups remain combined. Build and timing work ran sequentially.

Import timing starts after importing Basilisk and imports every public proxy in
the selected group. MuJoCo's import also includes its Python-side dependencies.
Cold means the first process after rebuilding or relinking the relevant native
files. Warm means the next fresh process. No reboot or disk-cache flush was used.
OS cache state, scheduling, and instrumentation affect timings; three trials and
one collection comparison do not establish a cross-platform performance guarantee.
Linux and Windows were not exercised in these timing measurements.

This comparison used an earlier benchmark that could switch layouts. Current
measurement instructions are in the repository's
``docs/source/Support/Developer/performanceBenchmarks.rst``; the supported tool
measures the grouped layout only.
