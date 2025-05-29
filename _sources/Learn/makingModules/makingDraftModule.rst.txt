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
executive summary.  Further, a table is created listing all input and output messages, their type and also
their description

Unit Test File
--------------
A draft unit test python file is created in the local ``_UnitTest`` folder.  This script

- imports the new module
- sets up a proces and task to run this module
- creates empty input messages object for all the module input messages
- sets up a recorder for all the module output messages
- run the script

The user can then expand this script to properly configure the module variables and input message contents
to create a fully implemented module unit test.


User Guide
----------
To run the script to make a C++ Basilisk module draft, edit the content of the ``fillCppInfo()`` method.
To create a C module, edit the method ``fillCInfo()``.  Next, in the main routine only run the needed method.

Note, once created, if you make edits and re-run this script it will delete the older module folder and create a new
one.  Thus, this script cannot be used to add input or output messages if you have already started
to flesh out the rest of the module code.  It is intended to be run only once before you start to edit the
module code.
