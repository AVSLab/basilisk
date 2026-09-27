.. _cppModules-4:

Swig Interface File
===================

The swig interface file makes it possible to create, setup and manipulate the Basilisk module from python.  This file ``*.i`` file is in the module folder with the ``*.h`` and ``*.cpp`` files.

The basic swig interface file looks like this:

.. code-block:: cpp
    :linenos:

    %module someModule

    %include "architecture/utilities/bskException.swg"
    %default_bsk_exception();

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
``python conanfile.py`` the following swig interfaces are defined:

.. code:: cpp

    %template(SomeMsgOutMsgsVector) std::vector<Message<SomeMsgPayload>>;
    %template(SomeMsgOutMsgsPtrVector) std::vector<Message<SomeMsgPayload>*>;
    %template(SomeMsgInMsgsVector) std::vector<ReadFunctor<SomeMsgPayload>>;

These message definitions can all be access via ``messaging`` package.

.. _bskModuleInputMessageLifetime:

Retaining Sources in Configuration Methods
------------------------------------------

Python ``subscribeTo()`` and ``addSubscriber()`` calls retain the source message
through the native reader. A C++ configuration method that calls
``tmpMsg->addSubscriber()`` internally does not pass through those Python
wrappers. Its SWIG interface must attach the source reference to the reader that
the method stores.

For the :ref:`addMsgToModule() example <bskOutputMessageCreation>`, place the
following hook before including the module header:

.. code-block:: cpp

    %include "std_vector.i"
    %pythonappend SomeModule::addMsgToModule %{
        self.moreInMsgs[-1]._install_keepalive(tmpMsg)
    %}
    %include "someModule.h"

This example assumes that each successful call appends exactly one reader.
Use the argument name declared in the header. If a method stores several input
readers, attach each source to its corresponding reader. The hook runs only after
the C++ method returns successfully; discard a partially configured module if
setup throws an exception.

For private reader storage, or methods that copy the reader during configuration,
add a C++ overload accepting a ``ReadFunctor<SomeMsgPayload>``. Store that reader
by value, and have the original message-pointer overload delegate to it using
``tmpMsg->addSubscriber()``. The Python entry point can then create a reader that
already retains its source:

.. code-block:: cpp

    %rename(_addMsgReader) SomeModule::addMsgToModule(ReadFunctor<SomeMsgPayload>);
    %pythonprepend SomeModule::addMsgToModule(Message<SomeMsgPayload>* tmpMsg) %{
        return self._addMsgReader(tmpMsg.addSubscriber())
    %}
    %include "someModule.h"

This preserves the existing Python message argument and the native C++ pointer
overload. The reader overload must preserve the original validation and return
value. If registration rejects duplicates without appending a reader, do not use
an unconditional ``[-1]`` hook: it would change the retention on a different
subscription. See :ref:`downlinkHandling` for a reader overload that preserves
duplicate detection.

The reference belongs to the native ``ReadFunctor``, so it survives vector growth
and reader copies. Unsubscribing, replacing the subscription, or destroying the
last reader releases the source. This avoids retaining old messages for the
entire lifetime of the module.

In :ref:`facetedSpacecraftModel`, pending articulation readers are also retained
for later reconfiguration. Unsubscribing an active reader leaves its pending
copy connected; the source remains alive until all connected copies are released.

The updated configuration methods cover spacecraft inputs in the atmosphere,
magnetic-field, wind, eclipse, location, charging, MSM, and formation-barycenter
models; planet inputs in eclipse, ephemeris conversion, albedo, and simple antenna
models; power and data storage inputs; transmitter, downlink, and mapping inputs;
articulated facets; thruster attached-body and small-body navigation inputs; and
Vizard camera configuration inputs. Their Python call signatures are unchanged.
A standalone Python-owned source can leave local scope after these calls. For
an output message owned by another module, keep that producing module alive as
described in :ref:`bskOutputMessageLifetime`. Direct C++ callers remain responsible
for the lifetime of their borrowed message sources.
