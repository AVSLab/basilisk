
Executive Summary
-----------------
The prescribed motion state effector class is an instantiation of the state effector abstract class. This module
is used to connect a prescribed motion component to the spacecraft hub. The states of the prescribed component are
purely prescribed, meaning there are no free degrees of freedom associated with its motion and hence there are no
states integrated in this module. Instead, the states of the prescribed component are externally profiled using
a kinematic profiler module and are provided as input messages to the prescribed state effector class.

The body frame for the prescribed body is designated by the frame :math:`\mathcal{P}`. The prescribed body is attached
to the hub through a hub-fixed mount interface described by the mount frame :math:`\mathcal{M}`. The prescribed body
hub-relative motion is profiled with respect to the mount frame :math:`\mathcal{M}`. The prescribed states are:
``r_PM_M``, ``rPrime_PM_M``, ``rPrimePrime_PM_M``, ``omega_PM_P``, ``omegaPrime_PM_P``, and ``sigma_PM``.

The states of the prescribed body are not defined in this module. Therefore, separate kinematic profiler modules must
be connected to this module's :ref:`PrescribedTranslationMsgPayload` and :ref:`PrescribedRotationMsgPayload`
input messages to profile the prescribed body's states as a function of time. If neither message is connected to the
module, the prescribed body will not move with respect to the hub. If the prescribed body should only rotate, then only
the rotational message needs to be subscribed to the module and the translational message can be ignored. Similarly,
if the prescribed component should only translate, the rotational message can be ignored. Note that either a single
profiler module can be used to connect both messages to this module, or two separate profiler modules can be used;
where one profiles the translational states and the other profiles the rotational states.
See the example script :ref:`scenarioDeployingSolarArrays` for more information about how to set up hub-relative
multi-body prescribed motion using this state effector module and the associated kinematic profiler modules.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - prescribedTranslationInMsg
      - :ref:`PrescribedTranslationMsgPayload`
      - Input message for the effector's translational prescribed states
    * - prescribedRotationInMsg
      - :ref:`PrescribedRotationMsgPayload`
      - Input message for the effector's rotational prescribed states
    * - prescribedTranslationOutMsg
      - :ref:`PrescribedTranslationMsgPayload`
      - Output message for the effector's translational prescribed states
    * - prescribedRotationOutMsg
      - :ref:`PrescribedRotationMsgPayload`
      - Output message for the effector's rotational prescribed states
    * - prescribedMotionConfigLogOutMsg
      - :ref:`SCStatesMsgPayload`
      - Output message containing the effector's inertial position and attitude states


Module Overview
---------------

Module Basic Capability
^^^^^^^^^^^^^^^^^^^^^^^
The basic capability of this module is to connect a prescribed motion component to the spacecraft hub. An in-depth
discussion of the functionality of this module and the dynamics derivation required to simulate this type of component
motion is provided in the following journal paper

.. note::

    `"Spacecraft Backsubstitution Dynamics with General Multibody Prescribed Subcomponents" <https://www.researchgate.net/publication/392662137_Spacecraft_Backsubstitution_Dynamics_with_General_Multibody_Prescribed_Subcomponents>`_,
    Leah Kiner, Hanspeter Schaub, and Cody Allard
    Journal of Aerospace Information Systems 2025 22:8, 703-715

Branching Capability
^^^^^^^^^^^^^^^^^^^^
An additional feature of this module is the ability to attach other state effectors to the prescribed motion component.
Doing so creates a chain, where the prescribed component is wedged between the hub and its attached state effector.
This capability enables select component branching relative to the spacecraft hub. A detailed discussion of this
capability is provided in the following conference paper

.. note::

    L. Kiner, and H. Schaub, `“Backsubstitution Method For Prescribed Motion Actuators With Attached Dynamic Sub-Components”  <https://www.researchgate.net/publication/394150919_Backsubstitution_Method_for_Prescribed_Motion_Actuators_with_Attached_Dynamic_Sub-Components>`_,
    AAS Astrodynamics Specialist Conference, Boston, Massachusetts, August 10–14, 2025

Currently, this branching capability has only been configured for the spinningBodyOneDOF, spinningBodyTwoDOF, and
linearTranslationOneDOF state effectors. See the User Guide section for how to connect these effectors to the
prescribed motion effector. The scenario scripts :ref:`scenarioPrescribedMotionWithRotationBranching`
and :ref:`scenarioPrescribedMotionWithTranslationBranching` are more complex examples demonstrating how to connect
multiple state effectors to the prescribed motion effector.


Module Testing
--------------
There are two unit test scripts for this module. The first tests the basic functionality of the module, while
the second test checks the branching capability of the module. Both tests are integrated tests with the
:ref:`prescribedRotation1DOF` and :ref:`prescribedLinearTranslation` profiler modules.

Basic Test
^^^^^^^^^^
The basic test script for this module profiles simultaneous translation and rotation for the prescribed body relative
to the hub using the :ref:`prescribedRotation1DOF` and :ref:`prescribedLinearTranslation` profiler modules. The initial
and reference attitude and displacement of the prescribed body relative to the hub are varied in the test. The test
checks that the quantities of spacecraft orbital angular momentum, rotational angular momentum, and orbital energy
are conserved for the duration of the simulation to verify the module dynamics.

Branching Test
^^^^^^^^^^^^^^
The branching test script for this module has three separate test functions. All tests configure an identical spacecraft
hub an actuating prescribed motion component. The first test connects a free :ref:`spinningBodyOneDOFStateEffector`
to the prescribed component. The second test connects a free :ref:`spinningBodyTwoDOFStateEffector` to the
prescribed component. The third test connects a free :ref:`linearTranslationOneDOFStateEffector` to the prescribed
component. All tests check that the quantities of spacecraft orbital energy, orbital angular momentum, rotational
angular momentum are conserved for the duration of the simulation.


User Guide
----------
This section outlines how to set up the prescribed motion module in python using Basilisk.

#. Import the prescribedMotionStateEffector class::

    from Basilisk.simulation import prescribedMotionStateEffector

#. Create the prescribed motion state effector::

    prescribed_body = prescribedMotionStateEffector.PrescribedMotionStateEffector()

#. Set the prescribed motion module parameters::

    prescribed_body.setMass(100.0)
    prescribed_body.setIPntPc_P([[50.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]])
    prescribed_body.setR_MB_B(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setR_PcP_P(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setR_PM_M(np.array([1.0, 0.0, 0.0]))
    prescribed_body.setRPrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setRPrimePrime_PM_M(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setOmega_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setOmegaPrime_PM_P(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setSigma_PM(np.array([0.0, 0.0, 0.0]))
    prescribed_body.setSigma_MB(np.array([0.0, 0.0, 0.0]))
    prescribed_body.ModelTag = "prescribedBody"


#. Add the prescribed state effector to your spacecraft::

    scObject.addStateEffector(prescribed_body)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Make sure to connect the required messages for this module.

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, prescribed_body)

See the example script :ref:`scenarioDeployingSolarArrays` for more information about how to set up hub-relative
multi-body prescribed motion using this state effector module and the associated kinematic profiler modules.

If you'd like to connect a state effector to the prescribed state effector instead of the spacecraft hub,
the states of the attached effector must be written with respect to the prescribed body frame instead of the hub
body frame. After creating the attached state effector, make sure to connect it to the prescribed motion effector
rather than the hub::

    prescribed_body.addStateEffector(attached_state_effector)

See the example scripts :ref:`scenarioPrescribedMotionWithRotationBranching`
and :ref:`scenarioPrescribedMotionWithTranslationBranching` for more information about how to connect
multiple state effectors to the prescribed motion effector.
