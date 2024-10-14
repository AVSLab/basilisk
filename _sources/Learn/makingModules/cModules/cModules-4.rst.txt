.. _cModules-4:

Swig Interface File
===================

The swig interface file makes it possible to create, setup and manipulate the Basilisk module from python.  This ``*.i`` file is in the module folder with the ``*.h`` and ``*.c`` files.

The basic swig interface file looks like this:

.. code-block:: cpp
    :linenos:

    %module someModule
    %{
       #include "someModule.h"
    %}

    %include "swig_c_wrap.i"
    %c_wrap(someModule);

    %include "architecture/msgPayloadDefC/SomeMsgPayload.h"
    struct SomeMsg_C;

    %pythoncode %{
    import sys
    protectAllClasses(sys.modules[__name__])
    %}

Regarding what swig interfaces to include, see C++ :ref:`cppModules-4` for additional options.
