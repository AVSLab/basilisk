.. _cppModules-5:

C++ Module with C-Wrapped Output Message
========================================

C++ Basilisk modules typically use C++ message input and output objects. However, if the module is a flight
software module, it is convenient to provide both a C++ message object and a C-wrapped message output.  The reason
is that some flight software scripts have multiple modules write to a single external gateway message.
For this to work, all modules must write to a message object that is of the same wrapper type (either C or C++).
Thus, if using both C++ and C Basilisk module, we need the module output to be provided in both formats.

How to add a C++ message output object is already covered in :ref:`cppModules-2`.  This page
describes how to add a C-wrapped output message.  This requires very little extra overhead.  The
module is already compute the output message content.  We simply have to write this output to
a second message type.


Changes in the Module Header File
---------------------------------
The C output messages setup steps are mostly the same as outlined in :ref:`cModules-1`.  Below we highlight
the steps and reasons.

Assume the output message is of type ``SomeMsg``.  Replace the import statement::

    #include "architecture/msgPayloadDefC/SomeMsgPayload.h"

with::

    #include "cMsgCInterface/SomeMsg_C.h"

This single import will now include the message payload definition that the C++ message will need, and it
includes the C message structure definition.

In the Basilisk module class definition, we will need to add the `SelfInit` method which is required
to self-initialized the C wrapped output message::

    void SelfInit();

Next, add the C-wrapped output message variable as a public variable::

    SomeMsg_C someOutMsgC = {};

Note that this line is slightly different than when setup in a C module in :ref:`cModules-1`.  First,
use the ``={}`` statement to create a zero'd copy of the C message structure.  Otherwise the msg payload
and header pointers in this structure are not zero on setup, and the message initialization will
not function properly.

Second, to distinguish between the regular module C++ message object ``someOutMsg``, we end the
message name with ``C``.


Changes in the Module Definition File
-------------------------------------
First, we need to define the module ``SelfInit()`` method to initialize the C-wrapped output message:

.. code-block:: cpp
    :linenos:

    void ModuleClass::SelfInit()
    {
        SomeMsg_C_init(&this->someOutMsgC);
    }

Second, when you write the output message payload ``this->someOutBuffer`` to the C++ message object, add this line
to write the same content to the C-wrapped output message::

    SomeMsg_C_write(&this->someOutBuffer, &this->someOutMsgC, this->moduleID, CurrentClock);

That is it.  You now have a C++ FSW module that can write to both message container types.
