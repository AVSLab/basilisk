Executive Summary
-----------------
The ``LinearTimeInvariantSystem`` module provides a reusable, stateful continuous-time linear time-invariant (LTI) model of the form :math:`\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}` and :math:`\mathbf{y} = \mathbf{C}\mathbf{x} + \mathbf{D}\mathbf{u}`. It is a base class intended to be subclassed with application-specific message mappings for the input vector :math:`\mathbf{u}` and output vector :math:`\mathbf{y}`.

Module Description
------------------
The base model computes

.. math::
   \dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}, \qquad
   \mathbf{y} = \mathbf{C}\mathbf{x} + \mathbf{D}\mathbf{u}

where :math:`\mathbf{x} \in \mathbb{R}^{n}`, :math:`\mathbf{u} \in \mathbb{R}^{m}`, and :math:`\mathbf{y} \in \mathbb{R}^{p}`.

The class registers a dynamic state block named ``"x"`` through the ``StatefulSysModel`` interface and initializes it to zero when :math:`n>0`. During each update step, subclasses provide :math:`\mathbf{u}` through ``readInput(CurrentSimNanos)``, and the base class computes :math:`\dot{\mathbf{x}}` and :math:`\mathbf{y}` before delegating output publication to ``writeOutput(CurrentSimNanos, y)``.

If no state is registered (for example, :math:`n=0`), the module evaluates only the direct-feedthrough term :math:`\mathbf{y} = \mathbf{D}\mathbf{u}` when dimensions permit.

The module includes convenience helpers for standard second-order models:

- ``configureSecondOrder(wn, zeta, k)`` for SISO systems with transfer function

  .. math::
     G(s) = \frac{k\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}

- ``configureSecondOrder(wn, zeta, k)`` (vector overload) for decoupled MIMO second-order channels with block-diagonal dynamics.

Message Interfaces
------------------
This base class does not define concrete simulation messages.

Subclasses must provide message mappings by overriding:

- ``readInput(CurrentSimNanos)`` to construct :math:`\mathbf{u}` from input messages
- ``writeOutput(CurrentSimNanos, y)`` to publish :math:`\mathbf{y}` to output messages

Module Assumptions and Limitations
----------------------------------
- The system is continuous-time and integrated by the simulation framework.
- Matrix dimensions must be consistent with inferred or overridden input/state/output sizes; inconsistencies are reported during ``Reset``.
- When :math:`n=0`, only direct feedthrough :math:`\mathbf{y}=\mathbf{D}\mathbf{u}` is evaluated.
- Size inference is matrix-driven in the base class unless overridden by derived classes.

Testing
-------
Unit tests are in ``_UnitTest/test_linearTimeInvariantSystem.py`` and are skipped when Basilisk is compiled without MuJoCo support (``--mujoco``). Coverage includes first-order and second-order response checks for C++ and Python-subclass usage.
