.. toctree::
   :hidden:
.. role:: raw-latex(raw)
   :format: latex
..

.. _customPython:

Using a Custom Python Installation
==================================

The following instructions are guidelines on how to run Basilisk with a
computer that is not using a system installed version of Python.

-  Basilisk must be built and run with the same Python binary.
   For example, you cannot build for Python 3.7 and run against Python 3.9.
-  The best way to work with different versions of python installed on your computer is to setup
   a virtual python environment, such as with ``venv``.  In this case only that python used to create the
   virtual environment is available and conflicts with other versions are avoided.
-  If you use a virtual environment, but sure to build Basilisk in that virtual environment for the simulation
   script to function.
