.. _makingModules-2:

Creating New Message Definitions
================================

Msg File Location
-----------------
The preferred message definition uses only C code to define a ``struct`` which contains the message data.  This is written in a stand-alone ``*.h`` file located in the ``src/architecture/msgPayloadDefC`` folder.

If the message ``struct`` definition needs to include C++ code, then the stand-alone message definition file is stored in ``src/architecture/msgPayloadDefCpp`` folder.

The ``*.h`` file name need to be upper camel case (start with a capital letter) and end with ``...MsgPayload.h``.  An example is ``SomeMsgPayload.h``.

Msg Data Definition
-------------------
The message data is defined through a ``struct`` definition.  For example, assume the new message is to be of type ``SomeMsg``:

.. code:: cpp

    #ifndef SOME_MSG_H
    #define SOME_MSG_H

    /*! @brief Brief description of what this message contains */
    typedef struct {
        int variable1;              //!< [units] variable description
        double variable2[3];        //!< [units] variable description
    }SomeMsgPayload;

    #endif

The ``#ifndef`` statement ensures this header file is only included once when compiling.  The ``struct`` definition needs to have the message type name followed by ``MsgPayload``.  In this case it is ``SomeMsgPayload``.  The msg payload is the data of the message.  This is in contrasts to the message object which contains a copy of this message payload, as well as a message header.


Creating the Msg C/C++ Swig Interface Files
-------------------------------------------
To create message objects in C and C++ modules, build Basilisk after adding the message definition. The initial
``python3 conanfile.py`` build auto-creates the corresponding C module interface files and stores them in
``basilisk/dist3/autoSource/cMsgCInterface``. The C++ message objects are automatically created as template classes
using the ``*.h`` definitions in ``msgPayloadDefC``. Naturally, no C module interface files are created for ``*.h``
files in ``msgPayloadDefCpp``.

Once ``dist3`` is configured, a normal incremental build detects added, renamed, or deleted payload definitions and
automatically reconfigures CMake as needed::

    cmake --build dist3 --parallel 12

Running ``python3 conanfile.py`` again is also valid when the Conan or CMake configuration needs to be refreshed. It
preserves unchanged generated and compiled products, so it does not turn an otherwise incremental build into a clean
build.

The generated Python messaging package is written from the current payload manifest. When a ``*Payload.h`` file is
renamed or deleted, the build removes that payload's stale generated source, Python wrapper, metadata, and compiled
extension artifacts. A clean build is therefore not required to prevent an obsolete message type from remaining
importable.

For the Basilisk module to have python interfaces to the module messages, you must include the message definition
``...MsgPayload.h`` file in the module swig interface file ``*.i``.  See :ref:`cppModules-4` for an example
on how to do this in a C++ module, and :ref:`cModules-4` on how to do this for a C module.
