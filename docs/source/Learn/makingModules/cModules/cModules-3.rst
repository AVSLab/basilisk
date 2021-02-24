.. _cModules-3:

Module Definition File
======================
The module function is defined in the ``SomeModule.c`` file.  This page outlines key expected behaviors.

SelfInit Method
---------------
A C code does not have constructors.  Thus, the Basilisk C module must define a self initialization function to ensure the output messages are setup to write to themselves.  This is shown in the sample code below.

.. code:: cpp

    /*!
        This method initializes the output messages for this module.
     @return void
     @param configData The configuration data associated with this module
     @param moduleID The module identifier
     */
    void SelfInit_someModule(someModuleConfig *configData, int64_t moduleID)
    {
        SomeMsg_C_init(&configData->someOutMsg);
    }

.. warning::

    The ``SelfInit`` function should only do the above steps.  Don't try to set default module variables here as the ``configData`` structure is zero'd in Swig when this structure copy is created in Python.  Thus, for C modules all default config structure variables are zero!


Reset Method
------------
The ``Reset_....()`` method should be used to

- restore module variables if needed. For example, the integral feedback gain variable might be reset to 0.
- perform one-time message reads such as reading in the reaction wheel or spacecraft configuration message etc.  Whenever ``Reset()`` is called the module should read in these messages again to use the latest values.
- check that required input messages are connected.  If a required input message is not connected when ``Reset()`` is called, then log a BSK error message.

The following sample code assumes that the class variable ``value`` should be re-set to 0 on ``Reset()``, and that ``someInMsg`` is a required input message:L

.. code:: cpp

    /*! This method performs a complete reset of the module.  Local module variables that retain
     time varying states between function calls are reset to their default values.
     @return void
     @param configData The configuration data associated with the module
     @param callTime [ns] time the method is called
     @param moduleID The module identifier
    */
    void Reset_someModule(someModuleConfig *configData, uint64_t callTime, int64_t moduleID)
    {
        configData->value = 0.0;

        if (!SomeMsg_C_isLinked(&configData->someInMsg) {
            char info[MAX_LOGGING_LENGTH];
            sprintf(info, "SomeModule does not have someInMsg connected!);
            _bskLog(configData->bskLogger, BSK_ERROR, info);
        }
    }

Note that doing a Basilisk log in a C module is different and than within a C++ module.


Update Method
-------------
The ``Update_....()`` function is called each time the Basilisk simulation runs the C module.  This method needs to perform all the required BSK module function, including reading in input messages and writing to output messages.  In the sample code below the message reading and writing, as well as the module function is done directly within this function.  Some modules also create additional functions to separate out the various functions.  This is left up to the module developer as a code design choice.

.. code:: cpp

    /*! Add a description of what this main Update() routine does for this module
     @return void
     @param configData The configuration data associated with the module
     @param callTime The clock time at which the function was called (nanoseconds)
     @param moduleID The module identifier
    */
    void Update_someModule(someModuleConfig *configData, uint64_t callTime, int64_t moduleID)
    {
        SomeMsgPayload outMsgBuffer;       /*!< local output message copy */
        SomeMsgPayload inMsgBuffer;        /*!< local copy of input message */

        // always zero the output buffer first
        outMsgBuffer = SomeMsg_C_zeroMsgPayload();

        /*! - Read the input messages */
        inMsgBuffer = SomeMsg_C_read(&configData->someInMsg);

        /* As an example of a module function, here we simply copy input message content to output message. */
        v3Copy(inMsgBuffer.dataVector, outMsgBuffer.dataVector);

        /*! - write the module output message */
        SomeMsg_C_write(&outMsgBuffer, &configData->someOutMsg, moduleID, callTime);
    }

.. warning::

    It is critical that each module zeros the content of the output messages on each update cycle.  This way we are not writing stale or uninitialized data to a message.  When reading a message BSK assumes that each message content has been either zero'd or written to.


Array of Messages
-----------------
Assume the module has an array called ``moreInMsgs`` which contain input message objects of type ``SomeMsg_C``.  Interfacing with these messages is done in a standard C manner with how elements of an array are retrieved.  For example, to read the 3-slot of the input array the code could read as

.. code:: cpp

    inMsgBuffer = SomeMsg_C_read(&configData->moreInMsgs[3]);

