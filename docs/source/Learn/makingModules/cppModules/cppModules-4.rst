.. _cppModules-4:

Swig Interface File
===================

The swig interface file makes it possible to create, setup and manipulate the Basilisk module from python.  This file ``*.i`` file is in the module folder with the ``*.h`` and ``*.cpp`` files.

The basic swig interface file looks like this:

.. code-block:: cpp
    :linenos:

    %module someModule
    %{
       #include "someModule.h"
    %}

    %pythoncode %{
    from Basilisk.architecture.swig_common_model import *
    %}

    %include "sys_model.i"
    %include "swig_conly_data.i"

    %include "someModule.h"

    %include "architecture/msgPayloadDefC/SomeMsgPayload.h"
    struct SomeMsg_C;

    %pythoncode %{
    import sys
    protectAllClasses(sys.modules[__name__])
    %}

The first line declare the module name through the ``module`` command.  This sets the module name within in the Basilisk python package.  This name typically is written in lower camel case format.  For example, if this is a simulation package, then the module is imported using::

    from Basilisk.simulation import someModule

Adjust this sample interface code as follows:

- Replace the any references of ``someModule`` to the actual module name.
- Adjust and repeat the message definition inclusion in lines 15-16 for each input and output message type used.
- The line ``struct SomeMsg_C`` is critical if the message type is C.  If the message type is C++ then this line is not required.
- If you miss including ``struct SomeMsg_C`` for your message types, the code will still compile without issue.  However, when you set a message variable from python the C++ module variable will not reflect this value and show 0.0 instead.


The line ``%include "swig_conly_data.i"`` enables the python interface to read and set basic integer and float values. The following additional includes can be made if a python interface is required for additional types.

``%include "std_string.i"``
    Interface with module string variables

``%include "std_vector.i"``
    Interface with module standard vectors of variables

``%include "swig_eigen.i"``
    Interface with module Eigen vectors and matrices


If you have to interact with a standard vector of input or output messages, running ``python conanfile.py`` will
auto-create the required python interfaces to vectors of output messages, vector of output message pointers,
as well as vectors of input messages. Assume the message is of type ``SomeMsg``. After running
``python conanfile.py`` the folling swig interfaces are defined:

.. code:: cpp

    %template(SomeMsgOutMsgsVector) std::vector<Message<SomeMsgPayload>>;
    %template(SomeMsgOutMsgsPtrVector) std::vector<Message<SomeMsgPayload>*>;
    %template(SomeMsgInMsgsVector) std::vector<ReadFunctor<SomeMsgPayload>>;

These message definitions can all be access via ``messaging`` package.
