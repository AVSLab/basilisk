.. _makingModules-2:

Creating New Message Definitions
================================

Msg File Location
-----------------
The preferred message definition uses only C code to define a ``struct`` which contains the message data.  This is written in a stand-alone ``*.h`` file located in the ``src/architecture/msgPayloadDefC`` folder.

If the message ``struct`` definition needs to include C++ code, then the stand-alone message definition file is stored in ``src/architecture/msgPayloadDefCpp`` folder.

The ``*.h`` file name need to be upper camel case (start with a capital letter) and end with ``...MsgPayload.h``.  An example is ``SomeMsgPayload.h``.

Msg Data Definition
-------------------
The message data is defined through a ``struct`` definition.  For example, assume the new message is to be of type ``SomeMsg``:

.. code:: cpp

    #ifndef SOME_MSG_H
    #define SOME_MSG_H

    /*! @brief Brief description of what this message contains */
    typedef struct {
        int variable1;              //!< [units] variable description
        double variable2[3];        //!< [units] variable description
    }SomeMsgPayload;

    #endif

The ``#ifndef`` statement ensures this header file is only included once when compiling.  The ``struct`` definition needs to have the message type name followed by ``MsgPayload``.  In this case it is ``SomeMsgPayload``.  The msg payload is the data of the message.  This is in contrasts to the message object which contains a copy of this message payload, as well as a message header.


Creating the Msg C/C++ Interface Files
--------------------------------------
To create message objects in C and C++ modules, you need to run ``python3 conanfile.py`` again from in the Terminal or command line window.  This script auto-creates the corresponding C module interface files and stores them in ``basilisk/dist3/autoSource/cMsgCInterface``.  The C++ message objects are automatically created as a template class using the ``*.h`` definitions in ``msgPayloadDefC``.  Naturally no C module interface files are created for ``*.h`` files in ``msgPayloadDefCpp``.

Running ``python3 conanfile.py`` re-creates the IDE project file that will now include access to the new message definition.

.. caution::

    Compiled message objects (built by ``python3 conanfile.py``) are added to the installed Basilisk package, such that they can be imported from Python. If you rename or delete a message's ``*Payload.h`` file, you should run a "clean" build to delete its previously compiled message objects, to avoid accidentally importing and using an old message type that should no longer exist.

    "Clean" builds can be done by deleting the ``dist3`` directory, or by running ``python3 conanfile.py --clean``.
