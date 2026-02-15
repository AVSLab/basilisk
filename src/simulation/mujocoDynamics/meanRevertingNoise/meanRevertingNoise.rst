Executive Summary
-----------------
The ``MeanRevertingNoise`` module is an abstract MuJoCo dynamics-task base class that implements a scalar
mean-reverting stochastic process (Ornstein-Uhlenbeck). It provides state registration, drift/diffusion setup, and
parameter accessors. Derived classes define how the scalar stochastic state is mapped into output messages.

Module Description
------------------
The internal scalar state :math:`x` follows

.. math::
   dx = -\frac{1}{\tau}x\,dt + \sqrt{\frac{2}{\tau}}\sigma_{st}\,dW

where:

- :math:`\tau` is the time constant
- :math:`\sigma_{st}` is the stationary standard deviation
- :math:`dW` is Wiener process increment

At each update, the base class computes:

.. math::
   \dot{x} = -\frac{1}{\tau}x

and configures one scalar diffusion source with amplitude

.. math::
   \sigma = \sqrt{\frac{2}{\tau}}\sigma_{st}

The base class then calls the virtual hook:

.. code-block:: cpp

   writeOutput(CurrentSimNanos, x)

which must be implemented by subclasses.

Message Interfaces
------------------
This base class does not define concrete input or output messages.

Subclasses are responsible for defining and publishing message interfaces in their ``writeOutput`` implementation.

Detailed Behavior
-----------------
At each update step, the module performs the following operations:

1. Ensures the scalar state was registered.
2. Reads current state value :math:`x`.
3. Sets deterministic derivative :math:`\dot{x} = -x/\tau`.
4. Sets scalar diffusion magnitude :math:`\sqrt{2/\tau}\sigma_{st}` for one noise source.
5. Calls subclass ``writeOutput(CurrentSimNanos, x)``.

During state registration, the module:

- registers a 1x1 state named ``meanRevertingState``,
- assigns one noise source,
- initializes the state to zero.

Module Assumptions and Limitations
----------------------------------
- The class is abstract and cannot be used directly without subclassing.
- ``setTimeConstant(t)`` requires :math:`t > 0`.
- ``setStationaryStd(s)`` requires :math:`s \ge 0`.
- Calling state getter/setter methods before state registration raises an error.

Verification and Testing
------------------------
The OU process implementation is verified in
``src/simulation/mujocoDynamics/meanRevertingNoise/_UnitTest/test_meanRevertingNoise.py`` through a subclass usage
path. The test validates empirical mean, variance, and estimated time constant against expected OU statistics.
