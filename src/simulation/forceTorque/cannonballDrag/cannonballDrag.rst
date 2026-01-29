Executive Summar-----------------
The ``CannonballDrag`` module computes quasi-steady aerodynamic drag force and the associated torque acting at a site fixed on a body. It outputs force and torque expressed in a site-fixed reference frame. The model follows the classical cannonball drag formulation and is intended for low-fidelity aerodynamic disturbance modeling.

Module Description
------------------
This module evaluates aerodynamic drag with force magnitude computed as

.. math::
   F = \tfrac{1}{2} \rho v^2 C_D A

where :math:`\rho` is the local neutral atmospheric density, :math:`v` is the magnitude of the relative flow velocity at the site, :math:`C_D` is the drag coefficient, and :math:`A` is the projected area. The drag force direction is opposite the relative velocity vector.

The force is applied at a specified center-of-pressure location relative to the site frame origin, producing a torque given by

.. math::
   \boldsymbol{\tau} = \mathbf{r}_{CP/S} \times \mathbf{F}

Both force and torque are expressed in the site-fixed reference frame.

Message Interfaces
------------------

Input Messages
^^^^^^^^^^^^^^
- ``dragGeometryInMsg`` (:ref:`DragGeometryMsgPayload`)

  Provides the aerodynamic geometry parameters:

  - ``dragCoeff``: Drag coefficient :math:`C_D`
  - ``projectedArea``: Reference area :math:`A`
  - ``r_CP_S``: Center-of-pressure vector relative to the site frame origin, expressed in the site frame

- ``atmoDensInMsg`` (:ref:`AtmoPropsMsgPayload`)

  Provides atmospheric properties at the site location:

  - ``neutralDensity``: Local atmospheric density :math:`\rho`

- ``referenceFrameStateInMsg`` (:ref:`SCStatesMsgPayload`)

  Provides the inertial state of the site reference frame:

  - ``sigma_BN``: Modified Rodrigues Parameters defining the site frame attitude relative to inertial
  - ``v_BN_N``: Inertial velocity of the site frame origin, expressed in inertial coordinates

Output Messages
^^^^^^^^^^^^^^^
- ``forceOutMsg`` (:ref:`ForceAtSiteMsgPayload`)

  Aerodynamic drag force expressed in the site reference frame.

- ``forceOutMsgC`` (:ref:`ForceAtSiteMsgPayload`)

  Aerodynamic drag force expressed in the site reference frame (C-wrapped output message).

- ``torqueOutMsg`` (:ref:`TorqueAtSiteMsgPayload`)

  Aerodynamic torque about the site frame origin, expressed in the site reference frame.

- ``torqueOutMsgC`` (:ref:`TorqueAtSiteMsgPayload`)

  Aerodynamic torque about the site frame origin, expressed in the site reference frame (C-wrapped output message).

Detailed Behavior
-----------------
At each update step, the module performs the following operations:

1. Reads the atmospheric density from ``atmoDensInMsg``.
2. Reads the drag coefficient, projected area, and center-of-pressure location from ``dragGeometryInMsg``.
3. Reads the site frame attitude and inertial velocity from ``referenceFrameStateInMsg``.
4. Transforms the inertial velocity into the site frame using the provided attitude.
5. Computes the drag force magnitude and direction in the site frame.
6. Computes the resulting torque about the site frame origin.
7. Publishes the force and torque through ``forceOutMsg`` and ``torqueOutMsg`` (and ``forceOutMsgC`` and ``torqueOutMsgC``).

The model assumes quasi-steady aerodynamics and does not account for lift, shadowing, free-molecular effects, or flow angular rates.

Verification and Testing
------------------------
The ``CannonballDrag`` module is verified through automated unit and integration tests that exercise both isolated force computation and coupled orbital dynamics behavior.

A unit-level test initializes the module with prescribed atmospheric density, drag geometry, site attitude, and inertial velocity. The resulting force and torque outputs are compared against analytically computed reference values obtained by explicitly rotating the inertial velocity into the site frame and applying the cannonball drag equations. The test passes if the force and torque vectors match the reference solutions within numerical tolerance.

An integration-level test embeds the module in a MuJoCo-based orbital simulation with gravity and an exponential atmosphere model. The spacecraft is propagated over one orbital period, and the drag force output is compared at each time step against an independent reference implementation of the same drag law evaluated using the recorded state and atmospheric density. Successful verification requires agreement between the module output and the reference drag force history to within prescribed tolerances.
