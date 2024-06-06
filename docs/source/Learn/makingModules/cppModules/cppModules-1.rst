.. _cppModules-1:

Module Header File
==================

Parent Class
------------
Every Basilisk module is a sub-class of :ref:`sys_model`.  This parent class provides common modules
variables such as ``ModelTag`` and ``moduleID``.

Module Class Name
-----------------
The Basilisk module class name should be descriptive and unique.  Its spelling has to be upper camel
case and thus start with a capital letter.

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

        Message<SomeMsgPayload> dataOutMsg;     //!< attitude navigation output msg
        ReadFunctor<SomeMsgPayload> dataInMsg;  //!< translation navigation output msg

        BSKLogger bskLogger;                    //!< -- BSK Logging

        /** setter for `varDouble` property */
        void setVarDouble(double);
        /** getter for `varDouble` property */
        double getVarDouble() const {return this->varDouble;}

    private:
        // user configurable variables
        double VarDouble;                       //!< [units] sample module variable declaration

        // general private module variables
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
Each module should define the ``Reset()`` method that is called when initializing the BSK simulation,
and the ``UpdateState()`` method which is called every time the task to which the module is added is updated.

Module Variables
----------------
The user configurable module variables should be either private or protected variables that are set and retrieved
through setter and getter methods.  Protected variables make sense if the variable is defined in a parent
class and the sub-class should have ready access to this variable.
The setter function should check that the value is
valid or return an error.  For example, you might want to set a gain variable, or the
spacecraft area for solar radiation pressure evaluation, and the setter method
should check that these values are strictly positive.  The benefit of using setter
and getter methods is that if a variable use is depreciated, then this variable
can be gracefully depreciated while introducing the new module functionality.
See :ref:`deprecatingCode` for information on how to depreciate code.

.. note::

    Having C++ Basilisk module use setter and getter methods is a recent requirements.
    Older modules mostly have the module variables as public variables which can be
    set directly.  Read the module documentation on how to setup and configure
    a Basilisk module.

The output messages are defined through the ``Message<>`` template class as shown above.  This
creates a message object instance which is also able to write to its own message data copy.

The input message object is defined through the ``ReadFunctor<>`` template class.

Finally, the ``bskLogger`` variable is defined to allow for BSK message logging with variable
verbosity.  It must be a public module variable.
See :ref:`scenarioBskLog` for an example of how to set the logging verbosity.

Vector of Input Messages
------------------------
To define a vector of input messages, you can define:

.. code:: cpp

    public:
        std::vector<ReadFunctor<SomeMsgPayload>> moreInMsgs;    //!< variable description
    private:
        std::vector<SomeMsgPayload> moreInMsgsBuffer;           //!< variable description

Note that the vector of input messages is defined as a public variable.  In contrast, the
vector of message definition structures (i.e. the message buffer variable) can be defined
as a private variable as it is only used within the module and not accessed outside.

Vector of Output Messages
-------------------------
To define a vector of output messages, we define a vector of message pointer using:

.. code:: cpp

    public:
        std::vector<Message<SomeMsgPayload>*> moreOutMsgs;      //!< variable description
