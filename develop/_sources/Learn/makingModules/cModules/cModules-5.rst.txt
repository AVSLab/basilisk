.. _cModules-5:

.. warning::

    The following content refers to the deprecated way of creating SWIG interface files for C modules. See :ref:`cModules-4` for the non-deprecated way.

Deprecated SWIG Interface File
==============================

The swig interface file makes it possible to create, setup and manipulate the Basilisk module from python.  This ``*.i`` file is in the module folder with the ``*.h`` and ``*.c`` files.

The basic swig interface file looks like this:

.. code-block:: cpp
    :linenos:

    %module someModule
    %{
       #include "someModule.h"
    %}

    %include "swig_conly_data.i"

    %constant void Update_someModule(void*, uint64_t, uint64_t);
    %ignore Update_someModule;
    %constant void SelfInit_someModule(void*, uint64_t);
    %ignore SelfInit_someModule;
    %constant void Reset_someModule(void*, uint64_t, uint64_t);
    %ignore Reset_someModule;

    %include "someModule.h"

    %include "architecture/msgPayloadDefC/SomeMsgPayload.h"
    struct SomeMsg_C;

    %pythoncode %{
    import sys
    protectAllClasses(sys.modules[__name__])
    %}

In contrast to the C++ swig interface files, note that here extra lines are required regarding the ``Update``, ``SelfInit`` and ``Reset`` methods.

Regarding what swig interfaces to include, see C++ :ref:`cppModules-4` for additional options.
