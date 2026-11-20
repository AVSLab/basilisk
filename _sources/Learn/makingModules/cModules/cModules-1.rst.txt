.. _cModules-1:

Module Header File
==================

Basic C Module Setup
--------------------
This page describes how to create the C module functions.  Because C is not an object oriented language it is necessary to recreate some concepts from C++ such as data encapsulation.  Thus, while a C++ module has its variables defined within the class, with a C module we need to create a module configuration structure which contains all the module variables.   The module config structure name should be the module name ending with ``....Config``.

When a C module is setup in python, the module data is created first by making an instance of the module configuration data structure, and then wrapping this with module specific code to make a functioning Basilisk module.  The module wrapped is a sub-class of :ref:`sys_model` and thus has the usual ``ModelTag`` and ``moduleID`` variables, as well as the ability to respond to ``Reset()`` and ``UpdateState()`` requests.


Sample Code
-----------
Let us assume the module is to be called ``SomeModule``.  The input and output message type is ``SomeMsg``.  The following example is analogous to how the sample C++ module is setup.

.. code-block:: cpp
    :linenos:

    #ifndef SOME_MODULE_H
    #define SOME_MODULE_H

    #include <stdint.h>
    #include "architecture/utilities/bskLogging.h"
    #include "cMsgCInterface/SomeMsg_C.h"

    /*! @brief Top level structure for the sub-module routines. */
    typedef struct {
        /* declare module variables */
        double dummy;                                   //!< [units] sample module variable declaration

        /* declare module IO interfaces */
        SomeMsg_C dataOutMsg;              //!< sample output message
        SomeMsg_C dataInMsg;               //!< sample input message

        BSKLogger *bskLogger;              //!< BSK Logging
    }someModuleConfig;

    #ifdef __cplusplus
    extern "C" {
    #endif

        void SelfInit_someModule(someModuleConfig *configData, int64_t moduleID);
        void Update_someModule(someModuleConfig *configData, uint64_t callTime, int64_t moduleID);
        void Reset_someModule(someModuleConfig *configData, uint64_t callTime, int64_t moduleID);

    #ifdef __cplusplus
    }
    #endif

Let's break down the above sample code.  The ``#include`` statements import all required support files, including ``bskLogging.h`` to log module status messages, as well as all input and output message definitions.  Note that while C++ uses template classes to take a message payload definition and create a message object, in C code the message object is create by wrapping the corresponding message behavior code about the message payload structure definition.  This step is done automatically when using ``python3 conanfile.py`` and the result is stored in ``basilisk/dist3/autoSource/cMsgCInterface/``.  The path to ``autoSource`` is automatically included in the ``cmake`` file, thus the shown shorter include can be used.

The C message wrapper functions, for example for ``SomeMsg`` type, are stored in the ``SomeMsg_C.h`` file.  Importing the ``..._C.h`` file will automatically also include the corresponding ``SomeMsgPayload.h`` file.  Thus, this single include provides access essentially to the C message object methods as well as the message data definition.

Next the module configure structure is defined.  In the above sample code this is the ``someModuleConfig`` definition.  Think of this module configuration file as the equivalent of class variables with a C++ module.  However, as this is a simple C structure, all config variables are public variables.  There is no concept of private module variables with C modules.

Finally, the required module functions must be defined.  You can define additional support functions.  Note that this code must be wrapped so the C code will properly compile within this C++ project.

Required Module Methods
-----------------------
Each C module should define ``SelfInit``, ``Reset`` and ``Update`` functions.  To tie these functions to this module, the function name is expanded with the module name as shown above.  Note that C++ modules do not require the ``SelfInit`` function as the message objects are automatically connected to their data copy in their constructor.  In C we have to do this with an extra step.

Note that each of these required module functions receives a pointer to the module configuration data structure.  This structure is typically called ``configData`` in each C module.

Module Variables
----------------
The module configuration structure defines all module variables.  This includes variables and message buffers needed to implement the module, the ``BSKLogger`` variable for module status logging, as well as all input and output messages.

Regarding C message objects, the sample ``SomeMsg_C` type can acts as either an input or output message.  Thus, in the above example the input and output messages are both of type ``SomeMsg_C``.

In contrasts to C++ modules were some class variables are public and some private, in the C module all module variables are public.

Array of Messages
-----------------
If the C module can receive a variable number of input messages, this is done by creating an array of of the desired message type.  As C modules can also be used in flight code, dynamic memory allocation should be avoided.  Thus, create the array with a size that is large enough to handle all cases.

For example, assume the module needs an array of input messages of type ``SomeMsg``.  In the module configuration structure define:

.. code:: cpp

    SomeMsg_C moreInMsgs[10]

The module needs to implement separate logic to determine how many messages have been set.  For example, the reset function could loop over this array and up to what slot the associate message object has been linked.

As the C wrapped message object can act as either input or output messages, the above example can readily be converted to an outpout message example by renaming the array variable ``moreOutMsgs``.

