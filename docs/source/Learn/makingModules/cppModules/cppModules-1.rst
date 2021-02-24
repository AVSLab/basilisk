.. _cppModules-1:

Module Header File
==================

Parent Class
------------
Every Basilisk module is a sub-class of :ref:`sys_model`.  This parent class provides common modules variables such as ``ModelTag`` and ``moduleID``.

Module Class Name
-----------------
The Basilisk module class name should be descriptive and unique.  Its spelling has to be upper camel case and thus start with a capital letter.

Sample Code
-----------
Let us assume the module is to be called ``SomeModule``.  The input and output message type is ``SomeMsg``.  A basic C++ module header file would contain for example:

.. code-block:: cpp
    :linenos:

    #ifndef SOME_MODULE_H
    #define SOME_MODULE_H

    #include "architecture/_GeneralModuleFiles/sys_model.h"
    #include "architecture/msgPayloadDefC/SomeMsgPayload.h"
    #include "architecture/messaging/messaging.h"
    #include "architecture/utilities/bskLogging.h"

    /*! @brief basic Basilisk C++ module class */
    class SomeModule: public SysModel {
    public:
        SomeModule();
        ~SomeModule();

        void Reset(uint64_t CurrentSimNanos);
        void UpdateState(uint64_t CurrentSimNanos);

    public:

        double dummy;                           //!< [units] sample module variable declaration

        Message<SomeMsgPayload> dataOutMsg;     //!< attitude navigation output msg
        ReadFunctor<SomeMsgPayload> dataInMsg;  //!< translation navigation output msg

        BSKLogger bskLogger;                    //!< -- BSK Logging

    private:
        double internalVariable;                //!< [units] variable description

    };

    #endif

Some quick comments about what is included here:

- The ``#ifndef`` statement is used to avoid this header file being imported multiple times during compiling.

- All ``include`` statements should be made relative to ``basilisk/src``

- The ``sys_model.h`` file must be imported as the BSK modules are a subclass of :ref:`sys_model`.

- All the message payload definition files need to be included.  In the above example there is only one.

- Include the Basilisk ``messaging.h`` file to have access to creating message objects

- Include the ``bskLogging.h`` file to use the BSK logging functions.

Be sure to add descriptions to both the module and to the class variables used.

Required Module Methods
-----------------------
Each module should define the ``Reset()`` method that is called when initializing the BSK simulation, and the ``UpdateState()`` method which is called every time the task to which the module is added is updated.

Module Variables
----------------
The public class variables can contain any variables that you want to have access to from the python layer.  For example, you might want to set a gain variable, the spacecraft area for solar radiation pressure evaluation, etc.

The output messages are defined through the ``Message<>`` template class as shown above.  This creates a message object instance which is also able to write to its own message data copy.

The input message object is defined through the ``ReadFunctor<>`` template class.

Finally, the ``bskLogger`` variable is defined to allow for BSK message logging with variable verbosity.  See :ref:`scenarioBskLog` for an example of how to set the logging verbosity.

Variables and methods that don't need to be accessible from the python layer can be defined as private variables.


Vector of Input Messages
------------------------
To define a vector of input messages, you can define:

.. code:: cpp

    public:
        std::vector<ReadFunctor<SomeMsgPayload>> moreInMsgs;    //!< variable description
    private:
        std::vector<SomeMsgPayload> moreInMsgsBuffer;           //!< variable description

Note that the vector of input messages is defined as a public variable.  In contrast, the vector of message definition structures (i.e. the message buffer variable) can be defined as a private variable as it is only used within the module and not accessed outside.

Vector of Output Messages
-------------------------
To define a vector of output messages, we define a vector of message pointer using:

.. code:: cpp

    public:
        std::vector<Message<SomeMsgPayload>*> moreOutMsgs;      //!< variable description

