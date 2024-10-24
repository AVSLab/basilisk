
.. _bskModuleCheckoutList:

Basilisk Module Checkout List 
=============================

This documents contains a series of action items that should be checked
before a Basilisk (BSK) module is approved.

Branch Must Be Up to Date
-------------------------
Make sure the branch is up to date and rebased on the latest develop code before a pull request is made.

Building Basilisk and Testing
-----------------------------

-   Do a clean build of Basilisk and make sure all code compiles as expected (see :ref:`FAQ <FAQ>` on how to do a
    clean build)
-   From the project root directory, run ``python run_all_test.py`` and ensure all python and C/C++ tests are passing
    as expected (see :ref:`installOptionalPackages` for info on installing and running ``pytest``)

Style and Formatting
--------------------

-  Do the code variables satisfy the :ref:`Basilisk code style
   guidelines <codingGuidelines>`?
-  Are 4 spaces used instead of tabs?

Module Programming
------------------

-  Are all module input and output messages Swig’d in the module ``*.i``
   file
-  Does the code contain appropriate general comments
-  Does the code contain Doxygen compatible function descriptions,
   variable definitions, etc.
-  Module startup and initialization

   -  The ``SelfInit()`` routine should declare the module output
      messages
   -  The ``CrossInit()`` routine should subscribe to the module input
      messages
   -  The ``Reset()`` in the FSW modules should reset all the default
      module configuration parameters.
- Is the module using the :ref:`bskLogging` Basilisk logging function?
  A general support library, i.e. non-Basilisk module, should use ``BSK_PRINT()`` instead.

Module Documentation
--------------------

Does the module contain a restructured text documentation file ``xxxx.rst``, where ``xxxx`` should be the same name as the module C or C++ file name.  The :ref:`cModuleTemplate` module contains a sample documentation set for a Basilisk module.   The required sections include:

-   Executive Summary
-   Module Assumptions and Limitations
-   Message Connection Descriptions
-   User Guide

The section `Detailed Module Description` is optional and used if there is extensive functionality and modeling to discuss.

As part of the code checkout build and test the associated documentation (see :ref:`createHtmlDocumentation`).

Module Functionality Testing
----------------------------

Is a ``_UnitTest`` folder included that:

-  includes a python file name starting with\ ``test_``
-  provides a test method that starts with ``test_xxxx()``
-  contains sufficient comments within the test file to explain what is done
-  only uses the test module (if possible), and creates the various
   required input messages
-  checks the module output for all input and module configuration
   conditions
-  performs the validation checking against either custom-computed
   results in other programs (Matlab, Mathematica, hand calculation),
   live Python computed results, or against expected simulation data
   (consistency checking)
-  can also be run with the python command instead of pytest (by
   updating the ``__main()__`` function at the bottom of the python
   file)

Module Integrated Test
----------------------
If an integrated test is provided as a ``test_XXX.py`` file.  Does this test method have a complete description of what is being tested?  The :ref:`test_cModuleTemplateParametrized.py <test_cModuleTemplateParametrized>` file contains a template illustrating the expected information.  Required sections include

    -   Validation Test Description
    -   Test Parameter Discussion
    -   Description of variables being tested

See the :ref:`FAQ <FAQ>` on how to run generate an html validation report using ``pytest --report``.  Note that it is ok to just run this report for the module being tested.

Update Release Notes
--------------------
Update the :ref:`bskReleaseNotes` at ``/docs/source/Support/User/bskReleaseNotes.rst`` to include information about the new features being added.