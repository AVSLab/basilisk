.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/e_938BFwtiI" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

.. _bskPrinciples-7:

Advanced: Redirecting Module Output to Stand-Alone Message
==========================================================


.. sidebar:: Source Code

    The python code shown below can be downloaded :download:`here </../../docs/source/codeSamples/bsk-7.py>`.

Consider a more advanced Basilisk simulation setup where you have two modules that both need to write to the same stand-alone message.  The motivation here is to simultaneously run two or more flight guidance algorithm modules, but only one gets executed depending on the flight mode.  Regardless of which guidance module is executed, the guidance output message must be fed to the same control module.  This cannot be accomplished if the third module subscribes either to the output message of module 1 or 2.  To avoid trying to re-subscribe to different module output messages when switching flight modes, we can choose to have both modules 1 and 2 write to the same stand-alone message as illustrated below.

.. image:: ../../_images/static/qs-bsk-7a.svg
   :align: center

The benefit is that the 3rd module can subscribe its input message to this one stand-alone message.  To be clear, this sample application assumes either module 1 or 2 is executed, but not both.  Otherwise, one would overwrite the others' message output.

The sample simulation script creates both a C and C++ module which have their individual output messages redirected to a stand-alone message. The process is different for both programming languages.

.. image:: ../../_images/static/qs-bsk-7.svg
   :align: center

.. warning::

    Basilisk C modules contain C wrapped message objects and thus can only write to a stand-alone C wrapped
    message interface.  Similarly, a C++ module contains C++ message objects and can only write to a C++
    stand-alone message.  You can't have a C module write to a C++ stand-alone message.

In the following sample code, a C and C++ Basilisk module are created.  To create a C wrapped stand-alone
message the ``messaging`` package must be imported from ``Basilisk.architecture``.  Next, assume a message
of type ``SomeMsg`` needs to be created.  This is done using::

    cStandAloneMsg = messaging.SomeMsg_C()
    cStandAloneMsg.write(messaging.SomeMsgPayload())

Be sure to provide an empty payload structure to the C-wrapped message object.  Otherwise a ``read()``
operation on this stand-alone msg object will cause a segmentation fault.
To enable a C module ``someCModule`` to redirect its output message ``dataOutMsg`` writing to this stand-alone
message use::

    messaging.SomeMsg_C_addAuthor(someCModule.dataOutMsg, cStandAloneMsg)

Now the module ``someCModule`` will not write to its own internal output message, but rather it will write into this stand-alone message.

.. literalinclude:: ../../codeSamples/bsk-7.py
   :language: python
   :linenos:
   :lines: 18-

For the C++ Basilisk module it is simpler to re-direct the output message.  The stand-alone message is created as before::

    cppStandAloneMsg = messaging.SomeMsg()

To redirect the output of a C++ module ``someCppModule`` to this stand-alone message, simply set::

    someCppModule.dataOutMsg = cppStandAloneMsg


.. note::

    If you want to record the output of ``someCModule`` be sure to record ``cStandAloneMsg``
    instead of ``someCModule.dataOutMsg``.  The later is no longer being written to
    unless you use the ``.read()`` method which sync's up the payload content.  In C++
    we are setting ``cppStandAloneMsg`` equal to ``someCppModule.dataOutMsg``.  Here recording either
    will give the same result.

To see the message states of both the module internal message objects and the stand-alone messages,
the sample script shows how to use ``.read()`` to read the current state of the message object.
This will return a copy of the message payload structure.  The same method can be used to access both
C and C++ wrapped messages. For the C-wrapped message object, the ``.read()`` command will also copy
the content from the stand-alone message to the module message.  This is why the ``.read()`` command
below on the module output message returns the correct value.
After executing the script you should see the following terminal output:

.. code-block::

    source/codeSamples % python bsk-7.py
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: C Module ID 1 ran Update at 0.000000s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 0.000000s
    BSK_INFORMATION: C Module ID 1 ran Update at 1.000000s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 1.000000s
    mod1.dataOutMsg:
    [2.0, 0.0, 0.0]
    cMsg:
    [2.0, 0.0, 0.0]
    mod2.dataOutMsg:
    [2.0, 0.0, 0.0]
    cppMsg:
    [2.0, 0.0, 0.0]


