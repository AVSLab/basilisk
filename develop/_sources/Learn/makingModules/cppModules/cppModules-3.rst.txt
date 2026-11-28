.. _cppModules-3:

Module Definition File
======================
The module function is defined in the ``SomeModule.cpp`` file.  This page outlines key expected behaviors.

Constructor
-----------
The constructor sets up class variables that require default values. For example, initialize the
sample runtime counter ``updateCounter`` to zero:

.. code:: cpp

    /*! @brief Initialize the sample runtime counter. */
    SomeModule::SomeModule()
    {
        this->updateCounter = 0.0;  // [-]
    }

Instead of declaring default values in the constructor, this can also be done in the module ``*.h`` file
by using the default value in the variable definition.

Destructor
----------
The module destructor should release any resources that need explicit cleanup.
Output messages held in the private smart-pointer storage described in
:ref:`bskOutputMessageOwnership` are released automatically. Do not delete the
borrowed pointers in the public output vector. If no other cleanup is needed,
the destructor can be defaulted:

.. code:: cpp

    SomeModule::~SomeModule() = default;

Reset Method
------------
The ``Reset()`` method should be used to

- Restore module variables if needed. For example, the integral feedback gain variable might be reset to 0.
- Perform one-time message reads such as reading in the reaction wheel or spacecraft configuration message. etc.
  Whenever ``Reset()`` is called the module should read in these messages again to use the latest values.
- Check that required input messages are connected.  If a required input message is not connected when
  ``Reset()`` is called, then log a BSK error message.
- Ensure that required variables are set to acceptable values.  For example, assume the gain variable
  is defaulted to zero.  The  ``Reset()`` method would check that the gain value is no longer the default value.

The following sample code assumes that the class variable ``value`` should be re-set to 0
on ``Reset()``, and that ``someInMsg`` is a required input message:L

.. code:: cpp

    /*! Reset the module.
     @return void
     */
    void SomeModule::Reset(uint64_t CurrentSimNanos)
    {
        this->value = 0.0;

        if (!this->someInMsg.isLinked()) {
            bskLogger.bskError("SomeModule does not have someInMsg connected!");
        }

        if (this->gain <= 0) {
            bskLogger.bskError("The required gain value was not set to a positive value.");
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


.. _bskOutputMessageCreation:

Vector of Input/Output Messages
-------------------------------
Use a public configuration method to add related input readers, payload buffers,
and output messages together. The following ``addMsgToModule()`` example uses
the vectors declared in :ref:`cppModules-1`. It subscribes to the supplied input,
adds an initialized payload buffer, and creates the corresponding output message.
Include the owned-message helper in the module's C++ implementation file:

.. code:: cpp

    #include "architecture/messaging/ownedMessage.h"

    /*! @brief Add an input subscription and its corresponding output message.
     * @param tmpMsg Input message that must remain alive while the module uses it.
     */
    void SomeModule::addMsgToModule(Message<SomeMsgPayload> *tmpMsg)
    {
        /* add the message reader to the vector of input messages */
        this->moreInMsgs.push_back(tmpMsg->addSubscriber());

        /* expand vector of message data copies with another element */
        SomeMsgPayload inputBuffer{};
        this->moreInMsgsBuffer.push_back(inputBuffer);

        /* own the output message and expose a borrowed pointer */
        addOwnedMessage(this->ownedMoreOutMsgs, this->moreOutMsgs);
    }

The example gives each input one output. For example, :ref:`eclipse` creates an
eclipse output message for each spacecraft state input added to the module.
``addOwnedMessage()`` creates the message in the private owner vector and appends
its borrowed pointer to the public vector. If construction or either insertion
throws, the helper releases the new message and preserves both vectors' previous
sizes and entries; their capacities may change. No manual deletion loop is needed.

This guarantee covers one message insertion. It does not undo earlier changes
to input readers, payload buffers, or other module configuration. If setup fails,
discard the partially configured module as described in
:ref:`bskOutputMessageLifetime`. Multiple groups of borrowed pointers may share
one owner vector, as in nested wheel or thruster output collections. Include the
helper only where the C++ implementation needs it; no SWIG declaration is needed.

The SWIG interface also needs the source-retention hook described in
:ref:`bskModuleInputMessageLifetime` so that standalone Python input messages
remain alive while these readers use them.

Setters
-------
Assume the module has a user configurable variable called ``gain``.  This variable
should be a private variable and be set through the ``setGain()`` method.
If possible, the setter method should check that valid values are provided
or throw an error.
