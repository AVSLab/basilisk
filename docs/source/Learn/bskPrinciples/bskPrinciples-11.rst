

.. _bskPrinciples-11:

.. warning:: 

    This section refers to a deprecated way of logging variables. Refer to previous documentation pages for the updated way.

Deprecated: Setting and Recording Module Variables
==================================================

The deprecated :ref:`SimulationBaseClass` method to record a module variable is::

    scSim.AddVariableForLogging( variableString, recordingTime, indexStart==0, indexStop==0)

Here ``variableString`` must be composed of the module tag string, a period and the variable name.
The examples above illustrate how to apply this method for a C or C++ module variable.

The ``recordingTime`` variable is the minimum time that must pass, in nano-seconds again, before the
module variable is recorded.

The optional integer arguments ``indexStart`` and ``indexStop`` are defaulted to zero, resulting in a
single value being recorded.  As this example is also recording a 3-dimensional array ``dumVector``,
it is recorded by setting the start and end index to 0 and 2 respectively.

After executing the script, the recorded variables are retrieved in general using
the :ref:`SimulationBaseClass` method::

    scSim.GetLogVariableData(variableString)

Here ``variableString`` is again composed of the ``ModelTag`` and variable name as before.  Note that the returned array has a first column that represents the time where the variable is recorded in nano-seconds.  Executing the script you should thus see the following output:
