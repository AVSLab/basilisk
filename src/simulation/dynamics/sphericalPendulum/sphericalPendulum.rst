
Executive Summary
-----------------

This class in an instantiation of the state effector class and implements an effector representing spherical pendulum.

The module
:download:`PDF Description </../../src/simulation/dynamics/sphericalPendulum/_Documentation/Basilisk-SPHERICALPENDULUM-20180518.pdf>`
contains further information on this module's function,
how to run it, as well as testing.

Message Connection Descriptions
-------------------------------
This state effector does not have any input or output messages.


Initialization and Reset
------------------------
``massInit`` must be finite and non-negative. Zero initial mass and depletion to zero during
integration remain supported. The damping matrix ``D`` must be finite, symmetric, and positive
semidefinite; both zero damping and singular positive-semidefinite damping are permitted.

These checks run before state registration during spacecraft initialization, including when
the effector is attached without being added to a task. Invalid configurations raise
``BasiliskError``. ``Reset()`` repeats the checks without accessing parent states or changing
integrated angles, angular rates, or mass. In particular, it does not restore depleted mass
from ``massInit``. Initial values are applied during state registration.
See :ref:`effectorInitialization`.
