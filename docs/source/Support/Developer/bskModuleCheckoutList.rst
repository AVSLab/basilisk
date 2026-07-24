
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
    as expected

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

For a Rust module:

- Is the top-level configuration struct marked with
  ``#[bsk_build::module]``?
- Does the module implementation use the ``<moduleName>.rs`` root source
  layout, with a matching ``[lib] path``?
- Does ``build.rs`` call ``bsk_build::generate_bindings()`` with the exact
  marked configuration struct name?
- Does the configuration struct contain only public parameters and annotated
  message ports, with no runtime, logger, or raw-pointer fields?
- Does the ``BskModule`` implementation declare ``type State`` and keep
  internal Rust-only state there, using ``()`` when stateless?
- Does lifecycle code obtain runtime metadata and logging through
  ``BskContext`` rather than retaining borrowed framework pointers?
- Do ``init``, ``reset``, and ``update`` return ``BskResult``, with expected
  validation and runtime failures reported as ``BskError`` rather than
  panics?
- Is the module package listed in the ``src/Cargo.toml`` workspace members?
- Does its ``Cargo.toml`` inherit the workspace minimum Rust version with
  ``rust-version.workspace = true``?
- Does its ``Cargo.toml`` contain the
  ``[package.metadata.basilisk]`` ``module = true`` marker?
- Was the shared ``src/Cargo.lock`` updated and reviewed after dependency
  changes, with no module-local ``Cargo.lock`` added?
- Does the full workspace command from :ref:`rustModules` pass with
  ``--all-features --locked``, along with a Basilisk build using
  ``--rustModules True``?
- Do the feature-minimal ``bsk-build`` tests and strict workspace Clippy check
  documented in :ref:`rustModules` pass?
- Does the workspace retain ``panic="unwind"`` for development and release
  profiles so generated boundaries can contain Rust panics?

Module Documentation
--------------------

Does the module contain a restructured text documentation file ``xxxx.rst``, where ``xxxx`` matches the module name? The :ref:`cModuleTemplate` and :ref:`rustModuleTemplate` modules contain sample documentation sets for a Basilisk module. The required sections include:

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
Follow ``docs/source/Support/bskReleaseNotesSnippets/README.md`` for when a
release-note snippet is required and how snippet files should be formatted.
