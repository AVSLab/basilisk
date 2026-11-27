
Executive Summary
-----------------
This state effector connects a component with prescribed motion to the spacecraft hub.
It extrapolates the component's relative position and angular velocity from the prescribed
rates and accelerations and integrates its relative attitude from the angular velocity.

The body frame for the prescribed body is designated by the frame :math:`\mathcal{P}`. The prescribed body is attached
to the hub through a hub-fixed mount interface described by the mount frame :math:`\mathcal{M}`. The prescribed body
hub-relative motion is profiled with respect to the mount frame :math:`\mathcal{M}`. The prescribed states are:
``r_PM_M``, ``rPrime_PM_M``, ``rPrimePrime_PM_M``, ``omega_PM_P``, ``omegaPrime_PM_P``, and ``sigma_PM``.

Translation and rotation can be configured independently. Without input messages the component
propagates from its configured initial state. With the default zero rates and accelerations it
remains fixed relative to the hub. One or more kinematic profilers can supply updated states
through the :ref:`PrescribedTranslationMsgPayload` and :ref:`PrescribedRotationMsgPayload` inputs.
See the example script :ref:`scenarioDeployingSolarArrays` for more information about how to set up hub-relative
multi-body prescribed motion using this state effector module and the associated kinematic profiler modules.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.


.. bsk-module-io:: prescribedMotionStateEffector
    :caption: Module I/O Messages

    input prescribedTranslationInMsg PrescribedTranslationMsgPayload
        Input message for the effector's translational prescribed states.
    input prescribedRotationInMsg PrescribedRotationMsgPayload
        Input message for the effector's rotational prescribed states.
    output prescribedTranslationOutMsg PrescribedTranslationMsgPayload
        Output message for the effector's translational prescribed states.
    output prescribedRotationOutMsg PrescribedRotationMsgPayload
        Output message for the effector's rotational prescribed states.
    output prescribedMotionConfigLogOutMsg SCStatesMsgPayload
        Output message containing the effector's inertial position and attitude states.


Initialization and Reset
------------------------
``setMass()`` requires a finite, non-negative mass. Invalid input raises ``BasiliskError``
before replacing the previous mass. Zero mass remains supported for a massless prescribed
reference frame.

Mass is also validated before state registration during spacecraft initialization and from
``Reset()``. Thus validation does not depend on adding the effector to a task. ``Reset()``
does not restore initial states or clear prescribed motion values, and it does not access
parent states. See :ref:`effectorInitialization`.

The initial position, translational velocity, and angular velocity apply at the spacecraft's
reset epoch. Prescribed accelerations propagate from that epoch until the effector reads a
translation or rotation command. Each scheduled effector update reapplies the latest written
sample for each connected input, using the current task time as that component's new epoch.
On each command tick, let the spacecraft advance to that time before the effector reads the
new sample. The effector's inertial output at that tick includes the new command, which
also supplies the initial state for the following integration interval.
Schedule the profiler to provide a fresh sample before the effector runs. A slower publisher
causes the same sampled state to restart on intervening ticks.


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

.. _prescribedMotionBranching:

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
multiple state effectors to the prescribed motion effector. Nested
variable-mass state effectors are not supported and must instead be attached
directly to :ref:`spacecraft`.


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

#. Both motion inputs are optional. Connect ``prescribedTranslationInMsg`` for a translation profile and
   ``prescribedRotationInMsg`` for a rotation profile. Each unconnected component propagates from its initial state.

#. Schedule the effector at the spacecraft module's rate, after the spacecraft and its input publishers.
   If they share a task, give the effector a lower model priority or add it last at the same priority::

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
