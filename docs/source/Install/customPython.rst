.. toctree::
   :hidden:
.. role:: raw-latex(raw)
   :format: latex
..

.. _customPython:

Using a Custom Python Installation
==================================

The following instructions are guidelines on how to run Basilisk with a
computer that is not using a system installed version of Python. First
some general notes:

-  Basilisk must be built and run with the same Python binary.
-  For Unix platforms Cmake should select the default installed version
   of Python.

To run cmake with the custom python installation that is not found by
the system by default, the path to the python installation can be
provided as a command line argument. Using a BASH shell and from within
``dist3`` folder, to compile with Python 3 you can use::

   cmake ../src -G "Xcode" -DPYTHON_LIBRARY=$(python3-config --prefix)/lib/libpython3.7.dylib -DPYTHON_INCLUDE_DIR=$(python3-config --prefix)/include/python3.7m -DPYTHON_EXECUTABLE=$(python3-config --exec-prefix)/bin/python3.7

Note that the above command didn’t work from a ``tcsh`` shell
environment. The ``python3-config`` command and package allows for easy
configuring of what python to use.
