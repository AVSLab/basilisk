.. _cppModules-3:

Module Definition File
======================
The module function is defined in the ``SomeModule.cpp`` file.  This page outlines key expected behaviors.

Constructor
-----------
The constructor ensure that all class variables that require default values are setup correctly.  For example, in the tutorial ``SomeModule`` class we are using in this documenation, assume the class variable ``dummy`` must be ``42`` be default.  This is done using:

.. code:: cpp

    /*! Module Constructor */
    SomeModule::SomeModule()
    {
        self->dummy = 42.0;
    }

Destructor
----------
The module destructor should ensure the module is closed down properly. It might have to close a file handle, or free up the memory of dynamically allocated message objects to a vector of output messages.

Reset Method
------------
The ``Reset()`` method should be used to

- restore module variables if needed. For example, the integral feedback gain variable might be reset to 0.
- perform one-time message reads such as reading in the reaction wheel or spacecraft configuration message. etc.  Whenever ``Reset()`` is called the module should read in these messages again to use the latest values.
- check that required input messages are connected.  If a required input message is not connected when ``Reset()`` is called, then log a BSK error message.

The following sample code assumes that the class variable ``value`` should be re-set to 0 on ``Reset()``, and that ``someInMsg`` is a required input message:L

.. code:: cpp

    /*! Reset the module.
     @return void
     */
    void SomeModule::Reset(uint64_t CurrentSimNanos)
    {
        this->value = 0.0;

        if (!this->someInMsg.isLinked()) {
            bskLogger.bskLog(BSK_ERROR, "SomeModule does not have someInMsg connected!");
        }
    }

Update Method
-------------
The ``UpdateState()`` is the method that is called each time the Basilisk simulation runs the module.  This method needs to perform all the required BSK module function, including reading in input messages and writing to output messages.  In the sample code below the message reading and writing, as well as the module function is done directly within this ``UpdateState()`` method.  Some modules also create additional class method to separate out the various functions.  This is left up to the module developer as a code design choice.

.. code:: cpp

    void CppModuleTemplate::UpdateState(uint64_t CurrentSimNanos)
    {
        SomeMsgPayload outMsgBuffer;       /*!< local output message copy */
        SomeMsgPayload inMsgBuffer;        /*!< local copy of input message */

        // always zero the output buffer first
        outMsgBuffer = this->dataOutMsg.zeroMsgPayload;

        /*! - Read the input messages */
        inMsgBuffer = this->dataInMsg();

        /* As an example of a module function, here we simply copy input message content to output message. */
        v3Copy(inMsgBuffer.dataVector, outMsgBuffer.dataVector);

        /*! - write the module output message */
        this->dataOutMsg.write(&outMsgBuffer, this->moduleID, CurrentSimNanos);
    }

.. warning::

    It is critical that each module zeros the content of the output messages on each update cycle.  This way we are not writing stale or uninitialized data to a message.  When reading a message BSK assumes that each message content has been either zero'd or written to.


Vector of Input/Output Messages
-------------------------------
If the module contains a vector of input messages called ``moreInMsgs``, you most likely will need to write a public method for the user to add input reader message objects to this vector variables.  Below a sample ``addMsgToModule()`` method is illustrated that receives a pointer to a message object, stores a copy of the reader object to this message in the standard vector, and expands the vector of read message value buffer with a new message paylod copy.

.. code:: cpp

    /*! Method description
     @param tmpMsg The message object pointer
     @return void
     */
    void SomeModule::addMsgToModule(Message<SomeMsgPayload> *tmpMsg)
    {
        /* add the message reader to the vector of input messages */
        this->moreInMsgs.push_back(tmpMsg->addSubscriber());

        /* expand vector of message data copies with another element */
        SomeMsgPayload tmpMsg;
        this->moreInMsgsBuffer.push_back(tmpMsg);

        /* create output message */
        Message<SomeMsgPayload> *msg;
        msg = new Message<SomeMsgPayload>;
        this->moreOutMsgs.push_back(msg);
    }

If the module contains a vector of output messages, then a public module method needs to be written to create these output vector message instances.  The above sample code illustrate a common scenario where the number of input and output messages is the same.  For example, in :ref:`eclipse` for each spacecraft state input message added a corresponding eclipse output message must be created.

Note that with the ``new`` call above the memory associated with this output message object instance is retained after the method is exited.  In this case the module deconstructor needs to free up the associated message memory.  For the above example this could be done using:

.. code:: cpp

    SomeModule::~SomeModule()
    {
        for (long unsigned int c=0; c<this->moreOutMsgs.size(); c++) {
            delete this->moreOutMsgs.at(c);
        }
    }

