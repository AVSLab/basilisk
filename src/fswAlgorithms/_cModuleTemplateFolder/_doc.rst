

General Purpose
---------------
The purpose of this folder is to provide a Basilisk module template.  While this example is for a C-based module, the general guidelines apply to C++ modules as well.

A module folder can contain a series of related module folders.  By adding a ``*.rst`` file to this folder the purpose of a folder can be documented.  The ``*.rst`` file name should be the same as the parent folder.

The sub-folder called ``_GeneralModuleFiles`` contains support ``*.c/h`` files that are used by all modules.  As a minimum, a file is required that defines the Module specific output message type


Usage
-----
To use this template:

- make a copy of this sub-folder
- rename the sub-folder name
- rename the ``*.c``, ``*.h``, ``*.i`` and ``*.rst`` files inside this sub-folder with the same module name
- rerun ``python3 conanfile.py`` to create an updated Xcode or MS Studio Project file
- edit the ``*.i`` file first

    - replace ``cModuleTemplate`` text inside this ``*.i`` file with the new sub-module name
    - include the proper module output message definition ``*.h`` file

- edit the ``*.h`` file next

    - provide a unique sub-module MACRO name at the top of the file
    - replace ``cModuleTemplate`` text inside this ``*.h`` file
    - add any needed Module configuration data states and remove the dummy variable
    - add any needed output or input message interface variables
    - add any needed sub-module specific support subroutine interface definitions

- edit the ``*.c`` file last

    - update comment with the module name and description
    - replace ``cModuleTemplate`` text inside this ``*.c`` file
    - add any needed ``*.h`` inclusions at the top.
    - edit the  Update() routine introductory comment to describe what this module does

- edit the ``*.rst`` file to document this module

- in the ``_UnitTest`` folder,

    - replace ``cModuleTemplate`` text inside ``test_xxx.py`` files
    - update the unit test to provide an I/O unit test of this module
    - update the test method doc-string to report on what is being tested

