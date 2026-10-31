.. _bskPrinciples-10:

Advanced: Handling ``BasiliskError`` in Simulations
===================================================
In Basilisk, when an error occurs during the simulation, it is raised as a ``BasiliskError`` exception in Python. This exception can be caught and handled from Python scripts, in order to gracefully manage errors. However, the simulation that raised the error should not continue to be used.

A short example on how to handle the ``BasiliskError`` exception is shown below. In this case, the Python logger's ``error()`` method raises the error. It is more likely that this error would be raised from a C++ module, but the example here is simplified to show how the exception can be caught and handled.

.. code-block:: python

    from Basilisk.architecture import bskLogging
    from Basilisk.architecture.bskLogging import BasiliskError

    try:
        bskLogging.BSKLogger().error("Uh oh! That isn't good!")
    except BasiliskError as e:
        print(f"Caught a BasiliskError: {e}")
