.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/XzimNVJm8t8" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

.. _bskPrinciples-5:

Creating Stand-Alone Messages
=============================


.. sidebar:: Source Code

    The python code shown below can be downloaded :download:`here </../../docs/source/codeSamples/bsk-5.py>`.

The prior example showed how to connect messages that are embedded within the Basilisk modules.  However, there are times where you might need to create a stand-alone copy of such a message.  Some flight algorithm modules require the input of a stand-alone message that provides information about the spacecraft mass and inertia properties, or thruster or reaction wheel configuration information.  For example, the module unit test ideally just runs the module being tested. Any input messages that this module needs should be created as stand-alone messages.  This avoids the unit test script depending on other modules output messages, but makes the module test function on its own.

This tutorial shows you how to create a stand-alone message and connect the :ref:`cppModuleTemplate` input message to it.  The syntax is identical to connect a stand-alone message to a C module.  Further, this example illustrates how the simulation can be started and stopped multiple times while the message or module variables are changed between runs.

.. image:: ../../_images/static/qs-bsk-5.svg
   :align: center

To create a stand-alone message, the message payload (i.e. data) container must be created first.  Let us assume the message is of type ``someMsg``.  The corresponding payload is called ``someMsgPayload``.  Thus, the payload container is created using::

    msgData = messaging.someMsgPayload()

Essentially this is a python instance of the message structure definition found in ``architecture/msgPayloadDefC/SomeMsg.h``.  The content of the message payload is zero'd on creating it.  If there is a ``variable`` in the structure that we want to change, this is done simply with::

    msgData.variable = .....

Next, a message object is created and the message data is written to it.  The message object is created using::

    msg = messaging.someMsg()

The payload is written to the message using::

    msg.write(msgData)

These steps can also be combined into a single line using::

    msg = messaging.someMsg().write(msgData)

The simulation code below creates a stand-alone message that is then connected to the module input message.

.. literalinclude:: ../../codeSamples/bsk-5.py
   :language: python
   :linenos:
   :lines: 18-

After the simulation runs for 10s, the stand-alone message data is changed and written into the message object.  Note that the stand-alone message object itself doesn't have to be re-created as this is still working and connected to the desired modules.  Rather, we only have to update the content of the message.

Next, the simulation stop time is extended for an additional 10s to 20s total and the simulation is executed again.  The resulting plot of the module output message is shown below.


.. image:: /_images/Scenarios/bsk-5.svg
   :align: center


