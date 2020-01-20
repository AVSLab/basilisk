
.. _makingNewBskModule:

Making a New C/C++ Basilisk Module
==================================

.. note::

   This page provides basic information on how to get started writing a new Basilisk module in C or C++.  The intend is to provide general guidelines and tips on how to get started.  It is recommended that the coder study existing modules as well that are similar to the new module being created.

It this is your very first module that you are writing, a good place to start is the :ref:`fswModuleTemplate` found at ``fswAlgorithms/_fswTemplateFolder``.  While this folder contains a generic ANSI-C Basilisk module, the general instructions are also suitable for a C++ module.  This folder contains several instructions on

- how to make a copy of an existing Basilisk folder and then rename the relevant module methods in the ``*.c/cpp``, ``*.h``, ``*.i`` and ``*.rst`` files.
- how to properly document the module functionality and usage
- how to properly document the module unit test(s)

If you are making a C-module, then this template folder is a great place to start.  If you are making a C++ module, it is recommend to find a similar modules such as a dynamics module, sensor module, environment module, and copy its basic functionality. Note that the :ref:`dynamicEffector` and :ref:`stateEffector` classes are treated differently from the rest of the Basilisk modules as they employ a state engine to facilitate the numerical integration of the associated differential equations.

Other modules, such as :ref:`magneticFieldCenteredDipole`, :ref:`exponentialAtmosphere` or :ref:`simpleBattery` modules are based on base classes.  Be sure to study the baseline on the expected overall behavior of such modules, and use the ``customXXXX()`` methods to provide the new module custom functionality.

If you think you are done and would like to contribute this module to the Basilisk repository, be sure to study the :ref:`bskModuleCheckoutList` to complete all required check-out tasks.
