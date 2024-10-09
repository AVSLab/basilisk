
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

Changed Importing of Some Files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If your module uses::

    #include "simulation/simFswInterfaceMessages/macroDefinitions.h"

this file has now moved to::

    #include "architecture/utilities/macroDefinitions.h"

Further, all includes should be done relative to the ``Basilisk/src`` directory.  This means::

    #include "sensors/magnetometer/magnetometer.h"

should be changed to::

    #include "simulation/sensors/magnetometer/magnetometer.h"


Updating a C Module
^^^^^^^^^^^^^^^^^^^

#. Updating the ``module.h`` file:

    - Remove the import of the C interface to the old messaging system, delete this line

      .. code:: cpp

         #include "messaging/static_messaging.h"

      The message wrappers being included below replace this step.

    - Update the ``#include`` statement to read in the message definition through

      .. code:: cpp

         #include "cMsgCInterface/ModuleMsg_C.h"

      This import provides access to the message object as well as the C message structure definition.

    - Replace the ``char`` message name variable ``moduleOutMsgName`` and associated
      ``int_32t`` message ID variable ``moduleOutMsgId`` with

      .. code:: cpp

         ModuleMsg_C moduleOutMsg;    // sensor output message
         ModuleMsg_C moduleInMsg;     // sensor input message

      Note that the ``Fsw``, ``Int`` and ``Sim`` message distinctions are now dropped in the new
      messaging system.  This step is the same regardless if this message connects to/from a C or
      C++ Basilisk module.

    - If you want to create an array of output messages, this can be done with

      .. code:: cpp

         SomeMsg_C descriptionOutMsgs[10];

    - Remove the ``CrossInit_xxxx()`` method, it is no longer used.  If you initialized any code in this function, move that code to the module ``Reset_xxxx()`` function.

    - Clean up the ``SelfInit_xxxx()`` method to only initialize the output messages.  All other code and setup should be moved to the ``Reset_xxxx()`` function.


#. Updating the ``module.c`` file:

    - To initialize the module output message in ``SelfInit_xxxx()``, replace

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
      additional code is required in your C code.  Remove the ``CrossInit_xxxx()`` method,
      it is no longer used.  If you initialized any code in this function, move that code
      to the module ``Reset_xxxx()`` function.


    - To create a local variable of the message content structure (payload) itself, use

      .. code:: cpp

        ModuleMsgPayload msgBuffer;

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

      - To check is a message has been connected to, check the value of ``ModuleMsg_C_isLinked()``
      - To check if a message has ever been written to, check the value of ``ModuleMsg_C_isWritten()``
      - To get the time when a message was written, use ``ModuleMsg_C_timeWritten()``
      - To get the ID of the module who wrote the message, use ``ModuleMsg_C_moduleID()``

    - To zero a message payload variable ``someMsgBuffer`` of type ``SomeMsgPayload``,
      while enjoying strong type checking, you can remove the use of ``memset()`` and use instead

      .. code:: cpp

         SomeMsgPayload someMsgBuffer;
         someMsgBuffer = SomeMsg_C_zeroMsgPayload();

    - To write to an output message, assuming ``outputMsgBuffer`` is a local variable holding
      the message content (payload), replace

      .. code:: cpp

         memset(&outputMsgBuffer, 0x0, sizeof(ModuleFswMsg));
         outputMsgBuffer.variable = 42;     // specify output msg values
         WriteMessage(configData->moduleOutMsgId, callTime, sizeof(ModuleIntMsg),
                 (void*) &(outputMsgBuffer), moduleID);

      with

      .. code:: cpp

         outputMsgBuffer = ModuleMsg_C_zeroMsgPayload();
         outputMsgBuffer.variable = 42;      // specify output msg values
         ModuleMsg_C_write(&outputMsgBuffer, &configData->moduleOutMsg, moduleID, callTime);

      Note that you should still zero the local ``outputMsgBuffer`` structure in C Modules such that the message
      has zero default values if some fields are note set.

#. Updating the ``module.i`` file:

    - In the ``GEN_SIZEOF()`` commands used to be used to get the size of a message in Python.  This is no longer
      required with the new message system.  Thus, these ``GEN_SIZEOF()`` commands can be removed.  To create and access
      messages from Python the ``message2`` package is now used.
    - Update the ``#include`` statement and add the ``struct`` statement to read

      .. code:: cpp

         %include "architecture/msgPayloadDefC/ModuleMsgPayload.h"
         struct ModuleMsg_C;
    - Any custom Swig'd interfaces to access message content, such as

      .. code:: cpp

         ARRAYASLIST(FSWdeviceAvailability)

      should be removed from the ``module.i`` file and moved to ``src/architecture/messaging/messaging.i``
      file instead.  These interfaces can now be used by any module by importing ``messages2`` in the
      Basilisk python script.

    - If you want to create an array of output messages ``SomeMsg_C``, the array of messages must be swig'd to be
      accessible from python.  In the module ``*.i`` file, add this statement

      .. code:: cpp

         STRUCTASLIST(SomeMsg_C)

    - The location of ``swig_common_model`` has moved to ``architecture``.  Thus, replace::

         from Basilisk.simulation.swig_common_model import *

      with::

         from Basilisk.architecture.swig_common_model import *

#. Updating the ``module.rst`` documentation file:

    - In the table of module messages, update any message variable names that were changed
      as well as the message definition from ``SomeFswMsgPayload`` to ``SomeMsgPayload``.
    - If applicable, update the module msg I/O illustration

Updating a C++ Module
^^^^^^^^^^^^^^^^^^^^^

#. Updating the ``module.h`` file:

    - Update the ``#include`` statement to read in a C message definition through

      .. code:: cpp

         #include "architecture/msgPayloadDefC/SomeMsgPayload.h"

      To include a C++ message definition use

      .. code:: cpp

         #include "architecture/msgPayloadDefCpp/SomeMsgPayload.h"

    - Replace the include statement for the old message system

      .. code:: cpp

        #include "architecture/messaging/system_messaging.h"

      with the include for the new message system

      .. code:: cpp

         #include "architecture/messaging/messaging.h"

    - Remove both the ``SelfInit()`` and ``CrossInit()`` methods, they are longer used.  If you initialized any code in these functions, move that code to the module ``Reset()`` method.

    - For output messages, replace the ``std::string`` message name variable
      ``moduleOutMsgName`` and associated
      ``int_32t`` message ID variable ``moduleOutMsgId`` with the ``public`` variable:

      .. code:: cpp

         Message<OutputMsgPayload>  moduleOutMsg;    //!< sensor output message

      This creates an instance of the output message object that is contained within this module.

    - For input messages, replace the ``std::string`` message name variable
      ``moduleInMsgName`` and associated
      ``int_32t`` message ID variable ``moduleInMsgId`` with the ``public`` functor:

      .. code:: cpp

         ReadFunctor<InputMsgPayload>   moduleInMsg;     //!< sensor input message

    - It is possible to create a vector of output message pointers of type ``SomeMsgPayload`` using

      .. code:: cpp

         std::vector<Message<SomeMsgPayload>*> descriptionOutMsgs;

    - Similarly, you can create a vector of input message reader objects of type ``SomeMsgPayload``
      using the following statement.  Note that you can directly create message reader instances,
      and not pointers to such objects as with a vector of output messages.

      .. code:: cpp

        std::vector<ReadFunctor<SomeMsgPayload>> descriptionInMsgs;

#. Updating the ``module.cpp`` file:

    - There is no need for additional code to create an output connector.  Thus, delete old message
      creation code such as:

      .. code:: cpp

         this->moduleOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->moduleOutMsgName,
                                                                             sizeof(ModuleSimMsg),
                                                                             this->numOutMsgBuffers,
                                                                             "ModuleSimMsg", this->moduleID);

      The new message object is automatically created through the above process in the ``module.h`` file.
      In fact, deleted the ``SelfInit()`` method as it is no longer needed with C++ modules.  The output
      message object automatically connect to themselves in their constructors.  Any other code in
      the ``SelfInit()`` method should be moved to the ``Reset()`` method.

    - If a ``std::vector`` of output message pointers of type ``SomeMsgPayload`` was created in the module ``*.h`` file
      then these message objects must be created dynamically in the ``*.cpp`` code using

      .. code:: cpp

         Message<SomeMsgPayload> *msg;
         msg = new Message<SomeMsgPayload>;
         this->descriptionOutMsgs.push_back(msg);

      Don't forget to delete these message allocation in the module deconstructor.

    - If you have a ``std::vector`` of input message objects, these are typically provided
      to the BSK module in Python through
      a support function.  For example, consider the case where the module has a vector of planet state input messages
      that can be configured.  The method ``addPlanet()`` then receives the message object pointer as shown below.
      Next, the vector ``planetInMsgs`` must have a reader to the provided message object added.  The message
      method ``.addSubscriber()`` returns a reader object, essentially an input message linked to this output
      message.  The code below assumes the module also has a ``std::vector`` of the planet state payload structure
      to act as the input buffer variables.  This ``addPlanet`` routine below creates such buffer variables and adds
      them to the ``planetMsgData`` vector each time a planet is added.

      .. code:: cpp

         void BskModuleName::addPlanet(Message<SpicePlanetStateMsgPayload> *planetSpiceMsg)
        {
            /* add a message reader to the vector of input messages */
            this->planetInMsgs.push_back(planetSpiceMsg->addSubscriber());

            /* expand the planet state input buffer vector */
            SpicePlanetStateMsgPayload plMsg;
            this->planetMsgData.push_back(plMsg);
        }


    - To check is an output message has been connected to, check the value of ``this->moduleOutMsg.isLinked()``


    - To subscribe to an input message, this is now accomplished in the Basilisk Python script
      where the message to module connections are setup now.  Thus, delete code such as this:

      .. code:: cpp

         this->moduleInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->moduleInMsgName,
                                                                                sizeof(ModuleFswMsg), moduleID);

      Remove the ``CrossInit()`` method, it is longer used.  If you initialized any code in this method,
      move that code to the module ``Reset()`` method.

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

         InputMsgPayload moduleInMsgBuffer;
         moduleInMsgBuffer = this->moduleInMsg();

      Take a moment to marvel at the simplicity of this message reading!

      - To check is an input message has been connected to, check the value of ``this->moduleInMsg.isLinked()``
      - To check if a message has ever been written to, check the value of ``this->moduleInMsg.isWritten()``
      - To get the time when a message was written, use ``this->moduleInMsg.timeWritten()``
      - To get the ID of the module who wrote the message, use ``this->moduleInMsg.moduleID()``

    - To zero a local message structure variable ``someMsgBuffer`` of type ``SomeMsgPayload``, remove
      the use of ``memset()`` and rather use the following.  If the msg buffer variable is for use
      with an output message ``someOutMsg``, then use

      .. code:: cpp

         SomeMsgPayload someMsgBuffer;
         someMsgBuffer = this->someOutMsg.zeroMsgPayload;

      If the buffer is related to an input message ``someInMsg``, the same basic syntax works.
      Just replace ``someOutMsg`` with ``someInMsg`` above.  This ensures the correct message type is zero'd
      and assigned to the local buffer variable.

    - To write to an output message, replace this old code:

      .. code:: cpp

         SystemMessaging::GetInstance()->WriteMessage(this->moduleOutMsgId, clockTime, sizeof(OutputSimMsg),
                                                 reinterpret_cast<uint8_t*> (&outMsgBuffer), this->moduleID);

      with this new code:

      .. code:: cpp

         this->moduleOutMsg.write(&outMsgBuffer, this->moduleID, clockTime);

      Again, stop and marvel.

#. Updating the ``module.i`` file:

    - The location of ``swig_common_model`` has moved to ``architecture``.  Thus, replace::

            from Basilisk.simulation.swig_common_model import *

      with::

            from Basilisk.architecture.swig_common_model import *

    - In the ``GEN_SIZEOF()`` commands used to be used to get the size of a message in Python.  This is no longer
      required with the new message system.  Thus, these ``GEN_SIZEOF()`` commands can be removed.  To create and access
      messages from Python the ``message2`` package is now used.

    - Update the C message definition include statement from

      .. code:: cpp

         %include "simMessages/OutputSimMsg.h"

      to use the new common message folder location

      .. code:: cpp

         %include "architecture/msgPayloadDefC/OutputMsgPayload.h"
         struct OutputMsg_C;

      If including a C++ message payload definition, then only use:

      .. code:: cpp

         %include "architecture/msgPayloadDefCpp/OutputMsgPayload.h"

    - Any custom Swig'd interfaces to access message content, such as

      .. code:: cpp

         %template(RWConfigVector) vector<RWConfigSimMsg>;

      should be removed the ``module.i`` file and moved to ``src/architecture/messaging/messaging.i``
      file instead.  These interfaces can now be used by any module by importing ``messaging`` in the
      Basilisk python script.

    - To create the swig interface to a vector of output message pointers of type ``SomeMsgPayload``,
      near the bottom of the ``messaging.i`` file add this line::

         %template(SomeOutMsgsVector) std::vector<Message<SomeMsgPayload>*>;

#. Updating the ``module.rst`` documentation file:

    - In the table of module messages, update any message variable names that were changed
      as well as the message definition from ``SomeFswMsgPayload`` to ``SomeMsgPayload``.
    - If applicable, update the module msg I/O illustration
    - If there are links to message types in the source method descriptions, update these
      to use the new message payload declaration.


.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/vAJ7G-ELDWA" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
