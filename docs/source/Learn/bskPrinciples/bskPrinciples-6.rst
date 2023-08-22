.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/XuaSmG4wYlk" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


.. _bskPrinciples-6:

Setting and Recording Module Variables
======================================


.. sidebar:: Source Code

    The python code shown below can be downloaded :download:`here </../../docs/source/codeSamples/bsk-6.py>`.

Sometimes it is convenient to record a Basilisk module variable, not just the input or output message.  This can be done via a python command as shown below.  However, note that such module variable recording will slow down the simulation as this is done in the python layer.

The simulation setup is shown in the figure below.  Both a C and C++ module are created and added to the single task.  However, no messages are connected here.  Rather, this sample code illustrates how to record module internal variables.  The variables are either public C++ class variables, or they are variables with the C module configuration structure.  Both :ref:`cModuleTemplate` and :ref:`cppModuleTemplate` have the exact same public variables for easy comparison.

.. image:: ../../_images/static/qs-bsk-6.svg
   :align: center

The sample code is shown below.  The C and C++ modules are set up as before.  The variable ``someVariable`` of module ``someModule`` is set in python using ``someModule.someVariable = ...``.

.. literalinclude:: ../../codeSamples/bsk-6.py
   :language: python
   :linenos:
   :lines: 18-

The :ref:`SimulationBaseClass` method to record a module variable is::

    scSim.AddVariableForLogging( variableString, recordingTime, indexStart==0, indexStop==0)

Here ``variableString`` must be composed of the module tag string, a period and the variable name.  The examples above illustrate how to apply this method for a C or C++ module variable. 

The ``recordingTime`` variable is the minimum time that must pass, in nano-seconds again, before the module variable is recorded.

The optional integer arguments ``indexStart`` and ``indexStop`` are defaulted to zero, resulting in a single value being recorded.  As this example is also recording a 3-dimensional array ``dumVector``, it is recorded by setting the start and end index to 0 and 2 respectively.

After executing the script, the recorded variables are retrieved in general using the :ref:`SimulationBaseClass` method::

    scSim.GetLogVariableData(variableString)

Here ``variableString`` is again composed of the ``ModelTag`` and variable name as before.  Note that the returned array has a first column that represents the time where the variable is recorded in nano-seconds.  Executing the script you should thus see the following output:

.. code-block::

    source/codeSamples % python bsk-6.py
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: C Module ID 1 ran Update at 0.000000s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 0.000000s
    BSK_INFORMATION: C Module ID 1 ran Update at 1.000000s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 1.000000s
    mod1.dummy:
    [[0.e+00 1.e+00]
     [1.e+09 2.e+00]]
    mod1.dumVector:
    [[0.e+00 1.e+00 2.e+00 3.e+00]
     [1.e+09 1.e+00 2.e+00 3.e+00]]
    mod2.dummy:
    [[0.e+00 1.e+00]
     [1.e+09 2.e+00]]
    mod2.dumVector:
    [[0.e+00 1.e+00 2.e+00 3.e+00]
     [1.e+09 1.e+00 2.e+00 3.e+00]]

Note that both the C and C++ module variables are correctly being recorded.
