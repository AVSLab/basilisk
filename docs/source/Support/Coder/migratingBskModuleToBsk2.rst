
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

    - Update the ``#include`` statement to read in the message definition through

      .. code:: cpp

         #include "architecture/messaging2/cMsgCInterface/ModuleMsg_C.h"

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

    - To create the module output message, replace

      .. code:: cpp

         configData->moduleOutMsgId = CreateNewMessage(configData->moduleOutMsgName,
          sizeof(ModuleFswOutMsg), "ModuleFswOutMsg", moduleID);

      with

      .. code:: cpp

         ModuleMsg_C_addAuthor(&configData->moduleOutMsg, &configData->moduleOutMsg);

    - To connect to an input message, delete

      .. code:: cpp

         configData->moduleInMsgId = subscribeToMessage(configData->moduleInMsgName,
                                                 sizeof(ModuleFswMsg), moduleID);

      The input messages are connected when then Basilisk simulation is scripted in python.  No
      additional code is required in your C code.

    - To read in a message, replace

      .. code:: cpp

         ModuleFswMsg msgBuffer;
         memset(&msgBuffer, 0x0, sizeof(ModuleFswMsg));
         ReadMessage(configData->moduleInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                     sizeof(ModuleFswMsg), (void*) &(sc), msgBuffer);

      with

      .. code:: cpp

         ModuleFswMsg msgBuffer;
         msgBuffer = ModuleMsg_C_read(&configData->moduleInMsg);

      - To check is an input message has been connected to, check the value of ``ModuleMsg_C_isLinked()``
      - To check if a message has ever been written to, check the value of ``ModuleMsg_C_isWritten()``
      - To get the time when a message was written, use ``ModuleMsg_C_timeWritten()``



    - To write to an output message, assuming ``outputMsgBuffer`` is a local variable holding
      the message content, replace

      .. code:: cpp

         WriteMessage(configData->moduleOutMsgId, callTime, sizeof(ModuleIntMsg),
                 (void*) &(outputMsgBuffer), moduleID);

      with

      .. code:: cpp

         ModuleMsg_C_write(&outputMsgBuffer, &configData->moduleOutMsg);

#. Updating the ``module.i`` file:

    - In the ``GEN_SIZEOF()`` commands, update the message from ``ModuleFswMsg`` to ``ModuleMsg``
    - Add this structure definition

      .. code:: cpp

         struct ModuleMsg_C;

    - Update the ``#include`` statement to

      .. code:: cpp

         %include "cMsgDefinition/ModuleMsg.h"


Updating a C++ Module
^^^^^^^^^^^^^^^^^^^^^

#. Updating the ``module.h`` file:

    - Update the ``#include`` statement to read in the message definition through

      .. code:: cpp

         #include "cMsgDefinition/OutputMsg.h"

    - Add the include statement for the new message system using:

      .. code:: cpp

         #include "architecture/messaging2/messaging2.h"

    - For output messages, replace the ``std::string`` message name variable
      ``moduleOutMsgName`` and associated
      ``int_32t`` message ID variable ``moduleOutMsgId`` with the public variable:

      .. code:: cpp

         SimMessage<OutputMsg>  moduleOutMsg;    //!< sensor output message

      This creates an instance of the output message object that is contained within this module.

    - For input messages, replace the ``std::string`` message name variable
      ``moduleInMsgName`` and associated
      ``int_32t`` message ID variable ``moduleInMsgId`` with the public functor:

      .. code:: cpp

         ReadFunctor<InputMsg>   moduleInMsg;     //!< sensor input message

    - If the module writes to an output message, then add the write functor as a private functor:

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

    - In the ``GEN_SIZEOF()`` commands, update the message from ``ModuleFswMsg`` to ``ModuleMsg``
    - Update the message definition include statement from

      .. code:: cpp

         %include "simMessages/OutputSimMsg.h"

      to use the new common message folder location

      .. code:: cpp

         %include "cMsgDefinition/OutputMsg.h"

