.. _cppModules-2:

C++ Message Object
==================
This page describes the C++ methods to interact with output and input messages.  The message template classes are defined in :ref:`messaging`, found inside ``src/architecture/messaging/messaging.h``.  This header file must be imported to make use of any of the message methods and properties described on this page.


Input Message (ReadFunctor)
---------------------------
The C++ input message is a ``ReadFunctor`` template class.  An public input message instance of type ``SomeMsg`` is created in the module ``*.h`` header file using

.. code:: cpp

    ReadFunctor<SomeMsgPayload> someInMsg;

To read an input message, create a local copy of the message payload type and simply call in the input message (i.e. read functor) as a function:

.. code:: cpp

    SomeMsgPayload  localMsgBuffer;
    localMsgBuffer = this->someInMsg();

Notice that the input message buffer doesn't have to be zero'd.  A complete copy of the incoming message is stored inside this variable.

.. note::

    The 2nd line above will return a compile time error if the read message is of a different type than the local message buffer variable.  This strong type checking makes it impossible to copy message data into the wrong message variable type.

The input message object has the following support methods:

``isLinked()``
    Returns a ``bool`` value depending if the input message reader is connected to an output message object or not

``isWritten()``
    Returns a ``bool`` value depending on if the connected output message has ever been written

``timeWritten()``
    Returns the simulation time when the connected message was written as a ``uint64_t`` time value in nano-seconds.

``moduleID()``
    Returns the ``int64_t`` ID value of the module which wrote the message.  Note that C/C++ module ID's are strictly positive, while Python module ID's are strictly negative.

``zeroMsgPayload``
    Zero'd structure copy of the message type associated with this input message.

These methods are called using standard C++ class syntax, such as

.. code:: cpp

    this->someInMsg.isLinked()



Message Object
--------------
The ``Message`` template class create a smart message objects with contains the payload payload (i.e. data) structure.  To create an output message of type ``SomeMsg``, in the module ``*.h`` header file declare this public message variable using:

.. code:: cpp

    Message<SomeMsgPayload> someOutMsg;

The message class contains a write functor called ``write`` that allows the message object to write to the message data.  Assume ``localMsgOutBuffer`` is local variable of type ``SomeMsgPayload`` and it has been populated with the desired values.  This message data is then written to the output message using:

.. code:: cpp

    SomeMsgPayload localMsgOutBuffer;
    localMsgOutBuffer = this->someOutMsg.zeroMsgPayload;
    ...
    this->someOutMsg.write(&localMsbOutBuffer, this->moduleID, CurrentTime);

Here ``CurrentTime`` is the integer simulation time in nano-seconds.  Note that the output message data buffer should be zero'd after creation.  This avoids uninitialized variables being written to a message.

.. note::

    If the message data buffer type and output message type are different, then the message ``write()`` method will result in a compile time error due to a type error.  This avoid the wrong message type being written to the output message container.

The output message object has the following support methods:

``isLinked()``
    Returns a ``bool`` value depending if the output message object has been connected to an input message.  This includes being connected to a ``recorder()`` module.

``zeroMsgPayload``
    Zero'd structure copy of the message type associated with this output message.

``addSubscriber()``
    Returns an input message (i.e. read functor) that is able to read this output message.

``addAuthor()``
    Returns the message write functor, granting write access to this message object.