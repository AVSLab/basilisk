.. _cModules-2:

C Message Object
================

This page describes the C functions to interact with C wrapped output and input message objects.  These methods are automatically created when running ``python3 conanfile.py`` and stored in ``basilisk/dist3/autoSource/cMsgCInterface``.

Input Message
-------------
The C wrapped message instance of type ``SomeMsg`` is created using

.. code:: cpp

    SomeMsg_C someInMsg;

The ``SomeMsg_C.h`` file contains a series of functions that emulated the C++ message object behaviors.  These function names all start with the message type, followed by ``_C_``, and finally have the desired method name added.  For example, to read a message you use:

.. code:: cpp

    SomeMsgPayload  localMsgBuffer;
    localMsgBuffer = SomeMsg_C_read(&configData->someInMsg);

Notice that the input message buffer doesn't have to be zero'd.  A complete copy of the incoming message is stored inside this variable.

.. note::

    The 2nd line above will return a compile time error if the read message is of a different type than the local message buffer variable.  This strong type checking makes it impossible to copy message data into the wrong message variable type.

The input message object has the following support methods.  The argument to each of these functions is the pointer to the input message variable as shown with the ``read`` example above.

``SomeMsg_C_isLinked(SomeMsg_C *msg)``
    Returns a ``int`` value depending if the input message reader is connected (1) to an output message object or not (0)

``SomeMsg_C_isWritten(SomeMsg_C *msg)``
    Returns a ``int`` value depending on if the connected output message has ever been written (1) or not (0)

``SomeMsg_C_timeWritten(SomeMsg_C *msg)``
    Returns the simulation time when the connected message was written as a ``uint64_t`` time value in nano-seconds.

``SomeMsg_C_moduleID(SomeMsg_C *msg)``
    Returns the ``int64_t`` ID value of the module which wrote the message.  Note that C/C++ module ID's are strictly positive, while Python module ID's are strictly negative.

``SomeMsg_C_zeroMsgPayload()``
    Returns a zero'd structure copy of the message type associated with this input message.


Output Message
--------------
To create an output message of type ``SomeMsg`` in C, the same command is used.  The only difference is that the output message variable name needs to end in ``...OutMsg``.

.. code:: cpp

    SomeMsg_C someOutMsg;

The message function  ``..._C_write()`` is used to write message data to the message object.  Assume ``localMsgOutBuffer`` is local variable of type ``SomeMsgPayload`` and it has been populated with the desired values.  This message data is then written to the output message using:

.. code:: cpp

    SomeMsgPayload localMsgOutBuffer;
    localMsgOutBuffer = this->someOutMsg.zeroMsgPayload;
    ...
    SomeMsg_C_write(&localMsgOutBuffer, &configData->someOutMsg, moduleID, callTime);

Here ``callTime`` is the integer simulation time in nano-seconds.  Note that the output message data buffer should be zero'd after creation.  This avoids uninitialized variables being written to a message.

.. note::

    If the message data buffer type and output message type are different, then the message ``.._C_write()`` method will result in a compile time error due to a type error.  This avoid the wrong message type being written to the output message container.

The output message object has the following support functions.  Again the argument of each function is a pointer to the output message variable.

``SomeMsg_C_isLinked(SomeMsg_C *msg)``
    Returns a ``int`` value depending if the output message object has been connected (1) to an input message or not (0).  This includes being connected to a ``recorder()`` module.

``SomeMsg_C_zeroMsgPayload()``
    Returns a zero'd structure copy of the message type associated with this output message.

``SomeMsg_C_addAuthor(SomeMsg_C *msg, SomeMsg_C *targetMsg)``
    This function makes the ``msg`` object write to ``targetMsg`` instead of writting to itself

``SomeMsg_C_init(SomeMsg_C *msg)``
    This method is required to initialize a C message object. If the message data pointer is not re-directed to write to another module, then his function will setup this message object to write to its own data container.
