Making Python Modules
=====================

To learn how to program a Python module, see :ref:`scenarioAttitudePointingPy`.  The module is defined purely in a python script.  As such there is no header, definition or swig interface file.

.. note::

    Python modules run much slower than C or C++ modules.  They are very convenient when prototyping a module behavior, or having a simple task to perform that is not called very often.

.. warning::

    The python process(es) will always be executed after the regular C/C++ processes.