.. _effectorBranching:

Enabling Branching of Basilisk Effectors
========================================

Back-substitution is the default method that manages the dynamics under the hood of Basilisk. The
backsubstitution formulation assumes that all dynamical modules, a.k.a. "effectors", influencing the
dynamics of the spacecraft are attached directly to its central hub in parallel. These effectors are
sub-classed as either :ref:`dynamicEffector` (provide external forces and torques acting on the
body, but doesn't have state differential equations to integrate) or :ref:`stateEffector` (has
internal state differential equations to integrate that couple with the spacecraft rigid hub).

This parallel coupling assumption can be relaxed to allow select configurations of effectors to be
attached in series to each other as opposed to being attached directly to the hub. See
:ref:`bskPrinciples-11` for a description on using this added functionality.

State Effector Augmentation
---------------------------
For a state effector, three changes must be made.

#. Connect the state effector's properties to the :ref:`dynParamManager`

    - Ensure the effector's inertial properties are of an appropriate data type to be accepted by
      the parameter manager (ex. pointer to ``Eigen::MatrixXd``). These properties should already be
      computed at each timestep by a method named something like
      ``compute<EffectorName>InertialStates()``
    - Define names for these inertial properties to be registered with in the parameter manager
    - Define an ``assignStateParamNames()`` method to hand these property names to the dependent
      effector
    - Override the ``registerProperties()`` method of the state effector base class, and inside it
      register the properties and call the corresponding ``linkInProperties()`` method of dependent
      effectors

#. Add a way to attach dynamic effectors to the state effector

    - Add a variable to the effector's class to store a list of its dependent effectors
    - Override the ``addDynamicEffector()`` method of the state effector base class, and inside it
      call ``assignStateParamNames()`` on the new dependent effector, and add it to the list

#. Augment the equations of motion to apply forces and torques of dependent effectors
    - Inside of ``updateContributions()``, loop over the list of dependent effectors, calling their
      ``computeForceTorque()`` methods, and collecting ``forceExternal_B`` and
      ``torqueExternalPntB_B``
    - Insert this force and torque into the effector equation of motion, ensuring to transform it
      into the appropriate frame
    - Insert the force and torque into the vecTrans and vecRot contributions, ensuring to transform
      it into the appropriate frame

Dynamic Effector Augmentation
-----------------------------
For a dynamic effector, three changes must also be made.

#. Connect the dynamic effector to its parents properties

    - Override the ``linkInProperties()`` method of the base class, and inside it collect the
      relevant properties from the parameter manager. Different effectors require various
      combinations of position, velocity, attitude, and angular velocity of their parent, including
      requiring none.
    - In places where hub states are used, add a conditional to use effector properties instead if
      the parent is a state effector instead of the hub.

#. If the dynamic effector has a setup factory, such as the thruster's :ref:`simIncludeThruster` or
   the reaction wheel's :ref:`simIncludeRW`, define a method there for adding to a state effector
   instead of the hub.

#. Set dynamic effector base class variable ``isAttachableToStateEffector`` to True in the
   constructor.

Additional Resources
--------------------
.. important::
    The force and torque ouput by dependent effectors as variables forceExternal_B and
    ``torqueExternalPntB_B`` is computed relative to its parent frame, not the body frame as they
    are labeled. For example if attached to a :ref:`spinningBodyOneDOFStateEffector`,
    ``torqueExternalPntB_B`` is actually the torque about point S expressed in the S frame
    (``torqueExternalPntS_S``) as its parent frame is the spinning body's frame.

    This also requires that the state effector output inertial properties to this frame origin
    (``r_SN_N``) as opposed to outputting inertial properties to its center of mass (``r_ScN_N``).
    This is important because dynamic effectors handle converting their applied forces to torques
    with respect to their parent frame origin. If the state effector passes it's center of mass
    position, this torque computed by the attached dynamic effector would need to be corrected, but
    the state effector does not get knowledge of where dynamic effectors are attached to make such a
    correction.

A more complete description of back-substitution, :ref:`stateEffector`, and :ref:`dynamicEffector`
can be found in :ref:`spacecraft` and its linked PDF description, particularly the Equations of
Motion section. Specifics on setting up the corresponding integrated tests for any newly enabled
branching effectors can be found in :ref:`test_effectorBranching_integrated`. A more detailed
description of this augmentation of the back-substitution method can be found in the following
conference paper.

.. note::

    A. Morell and H. Schaub, `"“Expanded Back-Substitution Dynamics Modeling For Branched Force And
    Torque Based Spacecraft Components" <https://hanspeterschaub.info/Papers/Morell2025.pdf>`_,
    AAS Spaceflight Mechanics Meeting, Kauai, HI, Jan. 19-23, 2025
