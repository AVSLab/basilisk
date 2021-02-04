Creating Stand-Alone Messages
=============================


.. sidebar:: Source Code

    The python code shown below can be downloaded :download:`here </../../codeSamples/bsk-5.py>`.

The prior example showed how to connect messages that are embedded within the Basilisk modules.  However, there are times where you might need to create a stand-alone copy of such a message.  This tutorial shows you how to create a stand-alone message and contect the :ref:`fswModuleTemplate` input message to it.  Further, this example illustrates how the simulation can be started and stopped multiple times while the message or module variables are changed between runs.

.. image:: ../../_images/static/qs-bsk-5.svg
   :align: center
   :scale: 50 %

To create a stand-alone message first the message payload (i.e. data) container must be created.  Let us assume the message is of type ``someMsg``.  The corresponding payload is called ``someMsgPayload``.  Thus, the payload container is created using::

    msgData = messaging.someMsgPayload()

Essentially this is a python instance of the message structure definition found in ``architecture/msgPayloadDefC/SomeMsg.h``.  The content of the message payload is zero'd on creating.  If there is a ``variable`` in the structure that we can to change, this is done simply with::

    msgData.variable = .....

Next a message object is created and the message data is written to it.  The message object is created using::

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

After the simulation runs for 10s, the stand-alone message data is changed and written into the message object.  After this the simulation stop time is extended for an additional 10s to 20s total and the simulation is executed again.  The resulting plot of the module output message is shown below.


.. image:: /_images/Scenarios/bsk-5.svg
   :align: center



.. ** Tutorial Review Video **

    .. raw:: html

        <iframe width="560" height="315" src="https://www.youtube.com/embed/6YmZyu0f-qI" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
