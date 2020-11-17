
.. _migratingModuleToBsk2:

Migrating Basilisk Modules from Version 1.X to 2.X
==================================================

Motivation
----------
This document discusses what coding related changes occurred with the new messaging
system in Basilisk version 2.0
and higher.  However, nothing is for free.  Making these changes was not possible without breaking existing code.
This migration
help page outlines all the Basilisk coding related changes that have occurred.
This facilities the process of upgrading legacy
Basilisk C and C++ code to function with the new message system, etc.

For changes related to updating Basilisk python scripts, see :ref:`migratingToBsk2`.


Message Names
-------------
See :ref:`migratingToBsk2` for the discussion on how module input and output messages no longer
have name and ID variables.

Creating New Message Definitions
--------------------------------
See :ref:`bsk2MessageDefinition` for how new message C structures can be created and added to the
Basilisk project.

Step-By-Step Guide to Upgrade a Basilisk Modules
------------------------------------------------
All the existing BSK 1.x messaging functionality exists with the new BSK 2.0 messaging system.
This facilitates the migration to 2.0 as it requires some commands to be changed, but
none of the code logic should have to change.  As before, the message interfaces vary between
C and C++ modules.

Moved Support Files
^^^^^^^^^^^^^^^^^^^
If your module uses::

    #include "simulation/simFswInterfaceMessages/macroDefinitions.h"

this file has now moved to::

    #include "simulation/utilities/macroDefinitions.h"



Updating a C Module
^^^^^^^^^^^^^^^^^^^

#. Updating the ``module.h`` file:

    - Remove the import of the C interface to the old messaging system, delete this line

      .. code:: cpp

         #include "messaging/static_messaging.h"

      The message wrappers being included below replace this step.

    - Update the ``#include`` statement to read in the message definition through

      .. code:: cpp

         #include "../dist3/autoSource/cMsgCInterface/ModuleMsg_C.h"

      This import provides access to the message object as well as the C message structure definition.

    - Replace the ``char`` message name variable ``moduleOutMsgName`` and associated
      ``int_32t`` message ID variable ``moduleOutMsgId`` with

      .. code:: cpp

         ModuleMsg_C moduleOutMsg;    // sensor output message
         ModuleMsg_C moduleInMsg;     // sensor input message

      Note that the ``Fsw``, ``Int`` and ``Sim`` message distinctions are now dropped in the new
      messaging system.  This step is the same regardless if this message connects to/from a C or
      C++ Basilisk module.

#. Updating the ``module.c`` file:

    - To initialize the module output message, replace

      .. code:: cpp

         configData->moduleOutMsgId = CreateNewMessage(configData->moduleOutMsgName,
          sizeof(ModuleFswOutMsg), "ModuleFswOutMsg", moduleID);

      with

      .. code:: cpp

         ModuleMsg_C_init(&configData->moduleOutMsg);

    - To check if an output message has been linked to other input message, use

      .. code:: cpp

         ModuleMsg_C_isLinked(&configData->moduleOutMsg);

    - To connect to an input message, delete

      .. code:: cpp

         configData->moduleInMsgId = subscribeToMessage(configData->moduleInMsgName,
                                                 sizeof(ModuleFswMsg), moduleID);

      The input messages are connected when then Basilisk simulation is scripted in python.  No
      additional code is required in your C code.

    - To create a local copy of the message content (payload) itself, use

      .. code:: cpp

        ModuleFswMsgPayload msgBuffer;

    - To read in a message, replace

      .. code:: cpp

         ModuleFswMsg msgBuffer;
         memset(&msgBuffer, 0x0, sizeof(ModuleFswMsg));
         ReadMessage(configData->moduleInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                     sizeof(ModuleFswMsg), (void*) &(sc), msgBuffer);

      with

      .. code:: cpp

         ModuleMsgPayload msgBuffer;
         msgBuffer = ModuleMsg_C_read(&configData->moduleInMsg);

      - To check is an input message has been connected to, check the value of ``ModuleMsg_C_isLinked()``
      - To check if a message has ever been written to, check the value of ``ModuleMsg_C_isWritten()``
      - To get the time when a message was written, use ``ModuleMsg_C_timeWritten()``



    - To write to an output message, assuming ``outputMsgBuffer`` is a local variable holding
      the message content (payload), replace

      .. code:: cpp

         memset(&outputMsgBuffer, 0x0, sizeof(ModuleFswMsg));
         outputMsgBuffer.variable = 42;     // specify output msg values
         WriteMessage(configData->moduleOutMsgId, callTime, sizeof(ModuleIntMsg),
                 (void*) &(outputMsgBuffer), moduleID);

      with

      .. code:: cpp

         memset(&outputMsgBuffer, 0x0, sizeof(ModuleMsgPayload));
         outputMsgBuffer.variable = 42;      // specify output msg values
         ModuleMsg_C_write(&outputMsgBuffer, &configData->moduleOutMsg, callTime);

      Note that you should still zero the local ``outputMsgBuffer`` structure in C Modules such that the message
      has zero default values if some fields are note set.

#. Updating the ``module.i`` file:

    - In the ``GEN_SIZEOF()`` commands used to be used to get the size of a message in Python.  This is no longer
      required with the new message system.  Thus, these ``GEN_SIZEOF()`` commands can be removed.  To create and access
      messages from Python the ``message2`` package is now used.
    - Update the ``#include`` statement and add the ``struct`` statement to red

      .. code:: cpp

         %include "cMsgPayloadDef/ModuleMsgPayload.h"
         struct ModuleMsg_C;
    - Any custom Swig'd interfaces to access message content, such as

      .. code:: cpp

         ARRAYASLIST(FSWdeviceAvailability)

      should be removed the ``module.i`` file and moved to ``src/simulation/architecture/messaging2/messaging2.i``
      file instead.  These interfaces can now be used by any module by importing ``messages2`` in the
      Basilisk python script.

#. Updating the ``module.rst`` documentation file:

    - In the table of module messages, update any message variable names that were changed
      as well as the message definition from ``SomeFswMsgPayload`` to ``SomeMsgPayload``.
    - If applicable, update the module msg I/O illustration

Updating a C++ Module
^^^^^^^^^^^^^^^^^^^^^

#. Updating the ``module.h`` file:

    - Update the ``#include`` statement to read in the message definition through

      .. code:: cpp

         #include "cMsgPayloadDef/OutputMsgPayload.h"

    - Add the include statement for the new message system using:

      .. code:: cpp

         #include "architecture/messaging2/messaging2.h"

    - For output messages, replace the ``std::string`` message name variable
      ``moduleOutMsgName`` and associated
      ``int_32t`` message ID variable ``moduleOutMsgId`` with the ``public`` variable:

      .. code:: cpp

         SimMessage<OutputMsg>  moduleOutMsg;    //!< sensor output message

      This creates an instance of the output message object that is contained within this module.

    - For input messages, replace the ``std::string`` message name variable
      ``moduleInMsgName`` and associated
      ``int_32t`` message ID variable ``moduleInMsgId`` with the ``public`` functor:

      .. code:: cpp

         ReadFunctor<InputMsg>   moduleInMsg;     //!< sensor input message

    - If the module writes to an output message, then add the write functor as a ``private`` functor:

      .. code:: cpp

         WriteFunctor<OutputMsg> writeModuleOutMsg;     //!< interface to writing to output message

#. Updating the ``module.cpp`` file:

    - In the constructor, connect the output message object to the write functor through:

      .. code:: cpp

         this->writeModuleOutMsg = this->moduleOutMsg.addAuthor();

    - There is no need for additional code to create an output connector.  Thus, delete old message
      creation code such as:

      .. code:: cpp

         this->moduleOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->moduleOutMsgName,
                                                                             sizeof(ModuleSimMsg),
                                                                             this->numOutMsgBuffers,
                                                                             "ModuleSimMsg", this->moduleID);

      The new message object is automatically created through the above process in the ``module.h`` file.

    - To check is an output message has been connected to, check the value of ``this->moduleOutMsg.isLinked()``


    - To subscribe to an input message, this is now accomplished in the Basilisk Python script
      where the message to module connections are setup now.  Thus, delete code such as this:

      .. code:: cpp

         this->moduleInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->moduleInMsgName,
                                                                                sizeof(ModuleFswMsg), moduleID);

    - To read an input message, replace old code such as:

      .. code:: cpp

         InputFswMsg moduleInMsgBuffer;
         memset(&moduleInMsgBuffer, 0x0, sizeof(InputFswMsg));
         this->moduleInMsg =
            SystemMessaging::GetInstance()->ReadMessage(this->moduleInMsgID, &LocalHeader,
                                                     sizeof(InputFswMsg),
                                                     reinterpret_cast<uint8_t*> (&(moduleInMsgBuffer)),
                                                     moduleID);

      with this new code:

      .. code:: cpp

         InputFswMsg moduleInMsgBuffer;
         moduleInMsgBuffer = this->moduleInMsg();

      Take a moment to marvel at the simplicity of this message reading!

      - To check is an input message has been connected to, check the value of ``this->moduleInMsg.isLinked()``
      - To check if a message has ever been written to, check the value of ``this->moduleInMsg.isWritten()``
      - To get the time when a message was written, use ``this->moduleInMsg.timeWritten()``


    - To check if an input message has been connected to, check the status of
      ``this->moduleInMsg.linked()``

    - To write to an output message, replace this old code:

      .. code:: cpp

         SystemMessaging::GetInstance()->WriteMessage(this->moduleOutMsgId, clockTime, sizeof(OutputSimMsg),
                                                 reinterpret_cast<uint8_t*> (&outMsgBuffer), this->moduleID);

      with this new code:

      .. code:: cpp

         this->writeModuleOutMsg(outMsgBuffer, clockTime);

      Again, stop and marvel.

#. Updating the ``module.i`` file:

    - In the ``GEN_SIZEOF()`` commands used to be used to get the size of a message in Python.  This is no longer
      required with the new message system.  Thus, these ``GEN_SIZEOF()`` commands can be removed.  To create and access
      messages from Python the ``message2`` package is now used.

    - Update the message definition include statement from

      .. code:: cpp

         %include "simMessages/OutputSimMsg.h"

      to use the new common message folder location

      .. code:: cpp

         %include "cMsgPayloadDef/OutputMsgPayload.h"

    - Any custom Swig'd interfaces to access message content, such as

      .. code:: cpp

         %template(RWConfigVector) vector<RWConfigSimMsg>;

      should be removed the ``module.i`` file and moved to ``src/simulation/architecture/messaging2/messaging2.i``
      file instead.  These interfaces can now be used by any module by importing ``messages2`` in the
      Basilisk python script.

#. Updating the ``module.rst`` documentation file:

    - In the table of module messages, update any message variable names that were changed
      as well as the message definition from ``SomeFswMsgPayload`` to ``SomeMsgPayload``.
    - If applicable, update the module msg I/O illustration
    - If there are links to message types in the source method descriptions, update these
      to use the new message payload declaration.
