
.. _makingNewBskModule:

Making a New C/C++ Basilisk Module
==================================

.. note::

   This page provides basic information on how to get started writing a new Basilisk module in C or C++.  The intend is to provide general guidelines and tips on how to get started.  It is recommended that the coder study existing modules as well that are similar to the new module being created.

It this is your very first module that you are writing, a good place to start is the :ref:`cModuleTemplate` found at ``fswAlgorithms/_cModuleTemplateFolder``.  While this folder contains a generic ANSI-C Basilisk module, the general instructions are also suitable for a C++ module.  This folder contains several instructions on

- how to make a copy of an existing Basilisk folder and then rename the relevant module methods in the ``*.c/cpp``, ``*.h``, ``*.i`` and ``*.rst`` files.
- how to properly document the module functionality and usage
- how to properly document the module unit test(s)


If you are making a C-module, then this sample/tutorial template :ref:`cModuleTemplate` which is a great place to start.  It is found in ``src/fswAlgorithm/_cModuleTemplateFolder/cModuleTemplate``.  It contains a basic C-based BSK module folder that you can copy and modify as needed. If you are making a C++ module, there is a sample C++ Basilisk found at :ref:`cppModuleTemplate` found in ``src/simulation/cppModuleTemplate``.
Besides starting with the sample BSK modules in C and C++, you can also find a similar module such as a dynamics module, sensor module, environment module, attitude control, navigation, etc., and copy its basic functionality. Note that the :ref:`dynamicEffector` and :ref:`stateEffector` classes are treated differently from the rest of the Basilisk modules as they employ a state engine to facilitate the numerical integration of the associated differential equations.

Other modules, such as :ref:`magneticFieldCenteredDipole`, :ref:`exponentialAtmosphere` or :ref:`simpleBattery` modules are based on base classes.  Be sure to study the baseline on the expected overall behavior of such modules, and use the ``customXXXX()`` methods to provide the new module custom functionality.

If you are using linear algebra to do vector and tensor math, be sure to read the :ref:`codingGuidelines` on how to
name matrix representations of vector and tensors.  Further, for C++ modules Basilisk includes support for the
`Intel Eigen library <http://eigen.tuxfamily.org>`_.  However, note that the ``.toRotationMatrix()`` `method <http://eigen.tuxfamily.org/dox/classEigen_1_1QuaternionBase.html#a8cf07ab9875baba2eecdd62ff93bfc3f>`_ in
Eigen will return the DCM :math:`[NB]`, not :math:`[BN]`.  Thus, the Basilisk specific Eigen MRP ``.toRotationMatrix()`` method
follows this convention.
If you are coding a C-module, then BSK includes the ``linearAlgebra.c/h`` support functions to do many common
linear algebra calculations.  Rigid body kinematics calculations are supported through the ``rigidBodyKinematics.c/h``
library.

If you think you are done and would like to contribute this module to the Basilisk repository, be sure to study the :ref:`bskModuleCheckoutList` to complete all required check-out tasks.
