.. _performanceBenchmarks:

Optional Performance Benchmarks
===============================

Basilisk performance benchmarks are developer utilities for measuring selected
implementation choices.  They are not numerical validation tests.  Run them when
investigating execution speed, comparing algorithms, or checking whether a local
change affects a performance-sensitive code path.

Benchmark timing depends on the processor, compiler, build type, system load, and
power management state.  Use a release build, run the benchmark several times from a
quiet terminal, and compare results from the same machine and build configuration.

Dynamics Effector Benchmark
---------------------------

The dynamics effector benchmark measures selected spacecraft, state effector, and
dynamic effector execution speed with a shared benchmark harness, including the
NDOF spinning-body BSM multi-body dynamics case.

Source script: :download:`benchmark_state_effectors.py <../../../../benchmarks/dynamics/benchmark_state_effectors.py>`

.. _benchmark_state_effectors:

.. automodule:: benchmark_state_effectors

Eigen and ``linearAlgebra`` Benchmark
-------------------------------------

The ``benchmark_eigen_linear_algebra`` executable compares common Eigen vector and
matrix operations against the Basilisk ``linearAlgebra.c/h`` utility functions.  It
includes dot products, cross products, fixed-size matrix-vector products, fixed-size
matrix-matrix products, dynamic matrix products, and inverse operations.

Source file: :download:`benchmark_eigen_linear_algebra.cpp <../../../../benchmarks/utilities/benchmark_eigen_linear_algebra.cpp>`

This benchmark is excluded from the normal build with ``EXCLUDE_FROM_ALL``.  Build it
explicitly from the Basilisk repository root directory:

.. code-block:: bash

   cd /path/to/basilisk
   .venv/bin/cmake -S src -B dist3
   .venv/bin/cmake --build dist3 --target benchmark_eigen_linear_algebra --config Release

Then run the benchmark executable:

.. code-block:: bash

   dist3/benchmarks/utilities/benchmark_eigen_linear_algebra

To run a longer benchmark, pass a positive integer scale factor.  For example, the
following command multiplies the default loop counts by ``5``:

.. code-block:: bash

   dist3/benchmarks/utilities/benchmark_eigen_linear_algebra 5

To run only a brief executable smoke check, use ``--smoke``:

.. code-block:: bash

   dist3/benchmarks/utilities/benchmark_eigen_linear_algebra --smoke

Expected Terminal Output
~~~~~~~~~~~~~~~~~~~~~~~~

The exact timings will vary by machine.  A representative run looks like:

.. code-block:: text

   Eigen versus linearAlgebra benchmark
   Build and run this benchmark target in Release mode for meaningful timings.
   The ratio column is linearAlgebra time divided by Eigen time.

   Operation                           Iterations    Eigen ns/op     linearAlg ns/op      linear/Eigen     checksum diff
   v3 dot                                20000000          2.512               2.623             1.044             0.000
   v3 cross                              10000000          1.701               3.458             2.032             0.000
   3x3 multiply                           5000000          2.386               3.538             1.483             0.000
   3x3 transpose multiply                 5000000          2.761               4.241             1.536             0.000
   3x3 vector multiply                    8000000          2.213               4.164             1.881             0.000
   3x3 inverse                            1000000          2.475               4.849             1.959             0.000
   4x4 inverse                             200000          4.401              22.896             5.202             0.000
   6x6 multiply                            800000          8.622              13.384             1.552             0.000
   6x6 vector multiply                    2000000          2.387               5.347             2.240             0.000
   dynamic 6x6 multiply                    500000         68.503              50.585             0.738             0.000
   dynamic 12x12 multiply                  100000        154.105             650.041             4.218             0.000
   generic 6x6 inverse                        200        573.330           54896.460            95.750             0.000

   checksum sink: -1791071704.319

Interpreting the Columns
~~~~~~~~~~~~~~~~~~~~~~~~

``Eigen ns/op``
   Average Eigen execution time for the listed operation, in nanoseconds per
   operation.

``linearAlg ns/op``
   Average ``linearAlgebra`` execution time for the same operation, in nanoseconds
   per operation.

``linear/Eigen``
   Ratio of ``linearAlg ns/op`` divided by ``Eigen ns/op``.  Values greater than
   ``1.0`` mean ``linearAlgebra`` was slower for that operation.  Values less than
   ``1.0`` mean ``linearAlgebra`` was faster.

``checksum diff``
   Relative difference between the Eigen and ``linearAlgebra`` checksums.  This is a
   basic guard that both loops performed comparable numerical work; it is not a
   replacement for a unit test.

Benchmark Smoke Tests
---------------------

The benchmark smoke tests are pass/fail freshness checks.  They do not evaluate
performance; they only verify that the benchmark entry points still build or run with
a minimal workload.

Source test: :download:`test_benchmark_smoke.py <../../../../benchmarks/tests/test_benchmark_smoke.py>`

The benchmark smoke tests are included in the default ``pytest`` collection paths,
so they run when a developer invokes ``pytest`` from the Basilisk repository root
directory or from the ``src`` directory.  To run only the benchmark smoke tests from
the repository root directory:

.. code-block:: bash

   .venv/bin/pytest -q benchmarks/tests

Or from the ``src`` directory:

.. code-block:: bash

   ../.venv/bin/pytest -q ../benchmarks/tests

The Python smoke test runs the dynamics benchmark with one integration step, one
trial, no warmup, one component, and one segment.  The C++ smoke test builds and runs
the ``benchmark_smoke_tests`` CMake target, which executes the Eigen versus
``linearAlgebra`` benchmark in ``--smoke`` mode with capped iteration counts.  The
C++ smoke test requires an already configured ``dist3`` build directory that includes
the benchmark targets; otherwise it is skipped with a reconfigure hint.

Developers can also run the C++ smoke target directly:

.. code-block:: bash

   .venv/bin/cmake --build dist3 --target benchmark_smoke_tests --config Release

Guidelines for Future Benchmarks
--------------------------------

Keep performance benchmarks opt-in unless there is a specific reason to run them in
CI.  New benchmarks should document:

- The target name and source file.
- The build configuration used for meaningful results.
- The command used to run the benchmark.
- Representative terminal output.
- How to interpret each reported metric.
- Any known limitations that affect comparison quality.
