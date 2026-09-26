.. _makingDraftModule:

Automation Script to Create a New Basilisk Module
=================================================

The script ``src/utilities/makeDraftModule.py`` provides a convenient tool to create a new Basilisk C or C++ module.
Note that this is a standard Basilisk module and not a dynamic or state effector.  The user provides the script
the basic module info such a module name, location, input and output message names, descriptions, type, etc., and
the script then generates a skeleton C or C++ Basilisk module, the associated swig interface ``*.i`` file, a start
at the RST documentation for this module, as well as a functioning unit test file for this module.

.. note::
    The script cannot setup vectors or lists of input or output messages.  In such cases, it is recommended to
    add a placeholder input/output message of the correct type, and then hand-edit the code to make this a
    vector of message objects.


Header File
-----------
The header file will contain the required includes to import the input/output message type definitions.
The message description strings are used to provide RST/Doxygen compatible comment strings for the
message definitions.

C/C++ File
----------
In the C-module the script generates the required ``init`` calls for each output message.  In C++
there is typically no need for writing a ``SelfInit()`` function as the C++ wrapped output messages
are connected to themselves on construction.

Next, the ``Reset()`` method is setup to check all input messages are connected.  This assumes that all
input messages are required. If this is not the case, simply delete the associate message connection
checking.

The ``Update()`` method sets up buffer variables to hold all the input messages, zero's the output message
buffer variables, reads in all input messages, and then write the output message buffer to the
output message objects.

Swig File
---------
The module ``*.i`` file is setup to include the required input and output message payload types.

RST Module Documentation
------------------------
The module RST documentation file uses the provided module description string to create the module
executive summary.  Further, the module input and output message entries are used to create a generated message
diagram and table listing each message name, type and description.
The draft also includes prompts for assumptions and limitations, algorithm details, and a
user guide. Complete these sections with the implemented behavior and a runnable Python
example. See :ref:`makingModules-3` for authoring instructions and :ref:`cModuleTemplate`
and :ref:`cppModuleTemplate` for completed examples.

Unit Test File
--------------
A Python smoke test is created in the local ``_UnitTest`` folder. This script

- imports the new module
- sets up a process and task to run the module at a 0.5-second interval
- creates blank input messages and subscribes every module input
- records every module output after each update
- checks that the module executes three times, at 0, 0.5, and 1 second
- checks that every output was written and that recording and write timestamps
  match all three task times

The execution check also applies to modules without outputs. The generated test has no
placeholder parameters and can run through ``pytest`` or directly as a Python script.

These checks exercise scheduling and message publication. They do not validate numerical
payload values. Once the module algorithm is implemented, configure meaningful input
payloads and add assertions for the expected output values at the indicated location.


User Guide
----------
To run the script to make a C++ Basilisk module draft, edit the content of the ``fillCppInfo()`` method.
To create a C module, edit the method ``fillCInfo()``.  Next, in the main routine only run the needed method.

Set ``modulePathRelSrc`` to an existing directory relative to ``basilisk/src``, such as
``moduleTemplates`` or ``fswAlgorithms/attControl``. A trailing path separator is optional.
The generated test imports the module from the top-level Basilisk package, for example
``Basilisk.fswAlgorithms``, even when its source is in a nested directory. Generation does
not change the caller's working directory. Absolute paths, parent-directory traversal,
and destinations that are symbolic links are rejected.

If the module folder already exists, the script asks before replacing it. Setting
``cleanBuild = True`` enables replacement without a prompt, as used for the generated
build examples. The generator validates the specification and writes all draft files to
a temporary directory before replacing an existing module. Validation or file-generation
errors leave the existing module intact; a failed installation attempts to restore the
original directory. If restoration also fails, the exception identifies the backup
directory containing the original files. Errors raise Python exceptions and produce a
nonzero exit status when unhandled by the calling script.

A successful replacement still discards previous edits in that module folder. This
script cannot merge new input or output messages into an implementation that you have
already started. It is intended to create the initial draft before you edit the module code.
