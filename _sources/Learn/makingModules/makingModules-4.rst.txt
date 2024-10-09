.. _makingModules-4:

Module Unit Test File
=====================

Inside the module folder there should be a sub-folder called ``_UnitTest``.  It needs to contain one or more python unit test scripts that check the functionality of the Basilisk module.  Each unit test file name must start with ``test_xxxx.py`` such that ``pytest`` recognizes this python file as a test file.

See C module template :ref:`UnitTestcModuleTemplate` to find two sample unit test scripts for :ref:`cModuleTemplate`.  Ideally only the module being tested should be loaded.  Any input messages can be created from Python and connected to the module.