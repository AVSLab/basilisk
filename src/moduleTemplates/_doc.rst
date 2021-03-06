

General Purpose
---------------
The purpose of this folder is to provide examples of Basilisk module templates.  It contains two fully functioning
sample C and C++ modules, as well as two modules which are auto-generated using the :ref:`makeDraftModule`
script found in ``src/utilities/makeDraftModule.py``.

A module folder can contain a series of related module folders.  By adding a ``*.rst`` file to this folder the purpose of a folder can be documented.  The ``*.rst`` file name should be the same as the parent folder.
Note that the :ref:`cModuleTemplate` has an expanded module documentation file that discusses in detail
what can be included, and how RST can be used to include math, figures, citations, links, tables, etc.

The sub-folder called ``_GeneralModuleFiles`` contains support ``*.c/h`` files that are used by all modules.  As a minimum, a file is required that defines the Module specific output message type


Usage of Template Folders
-------------------------
To use the C or C++ template module:

- make a copy of this sub-folder
- rename the sub-folder name
- rename the ``*.c/cpp``, ``*.h``, ``*.i`` and ``*.rst`` files inside this sub-folder with the same module name
- rerun ``python3 conanfile.py`` to create an updated Xcode or MS Studio Project file
- edit the ``*.i`` file first

    - replace the module name, such as ``cModuleTemplate`` or ``cppModuleTemplate``,
      text inside this ``*.i`` file with the new sub-module name
    - include the proper module output message definition ``*.h`` file

- edit the ``*.h`` file next

    - provide a unique sub-module MACRO name at the top of the file
    - replace old module name text inside this ``*.h`` file with new module name
    - add any needed Module configuration data states and remove the dummy variable
    - add any needed output or input message interface variables
    - add any needed sub-module specific support subroutine interface definitions

- edit the ``*.c`` file last

    - update comment with the module name and description
    - replace old module name text inside this ``*.c`` file with new module name
    - add any needed ``*.h`` inclusions at the top.
    - edit the  Update() routine introductory comment to describe what this module does

- edit the ``*.rst`` file to document this module

- in the ``_UnitTest`` folder,

    - replace olde module name text inside ``test_xxx.py`` files with new module name
    - update the unit test to provide an I/O unit test of this module
    - update the test method doc-string to report on what is being tested


Usage of ``makeDraftModules.py``
--------------------------------
The script :ref:`makeDraftModule` in ``basilisk/src/utilities/makeDraftModule.py``
provides a class that creates a default Basilisk
module folder and associated ``.c/cpp``, ``.h``, ``.i``, ``.rst`` and ``_UnitTest/test_xxx.py`` files.
When running ``python3 conanfile.py`` this script is used to create the ``autoCModule`` and ``autoCppModule`` folders.
This provides convenient examples and ensures this auto-module creation capability is tested and up to date
with the latest Basilisk code base.

See the bottom of the support script ``makeDraftModule.py`` for examples how to create a folder with draft
Basilisk module code.  You can

- specify where the folder should be located,
- what the module name will be,
- add a brief description of the module,
- specify the copyright holder of the module
- provide lists of input and output messages including their message type, variable name, description, etc.

The script will create the basic draft module support code that will compile and run.  The ``Reset()`` method
already checks if the input messages are connected (assuming they are all required), and ``Update()`` method
provides sample code the creates buffer variables for the input and output messages, reads in the input messages,
and write to the output messages.  The unit test already loads up the module, creates blank input message copies
that are subscribed to the module, creates recorder modules for each output message, etc.


