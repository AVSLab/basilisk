
Executive Summary
-----------------
The general single body state effector class is an instantiation of the state effector abstract class. This effector
simulates a single general rigid component with up to 6 sequential degrees of freedom (3 translation, 3 rotation).
Each component degree of freedom must configured individually and added to the effector in the order of intended
sequential motion, starting at the hub base and extending outward.

.. note::

    While this effector supports up to 6 sequential degrees of freedom, the current module has only been tested
    and verified for the one degree-of-freedom cases: rotation, translation, and helical screw motion. Future work
    involves full 6-DOF verification of this effector.

There are four options for configuring a degree-of-freedom. A rotational degree of freedom can be configured and added
using the ``addRotDOF()`` method. A translational degree of freedom can be added using the ``addTransDOF()`` method.
A screw constant variable can be set for either of these base degrees of freedom to enable helical screw motion.
By default helical screw motion is turned off and the screw constant variable is set to zero. For nonzero values,
the screw constant scales the base degree of freedom for the coupled 1-DOF motion.

.. note::

    All configured degree of freedom must be unique, and can only be added to the general effector once. If the user
    attempts to add more than three rotational or three translational degrees of freedom, an error is thrown.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: prescribedMotionStateEffector
    :caption: Module I/O Messages

    input spinningBodyRefInMsg HingedRigidBodyMsgPayload
        Input message vector for the rotational degree of freedom reference angles
    input motorTorqueInMsg ArrayMotorTorqueMsgPayload
        Input message vector for the rotational degree of freedom motor torques
    input translatingBodyRefInMsgs LinearTranslationRigidBodyMsgPayload
        Input message vector for the translational degree of freedom reference displacements
    input motorForceInMsg ArrayMotorForceMsgPayload
        Input message vector for the translational degree of freedom motor forces
    output spinningBodyOutMsgs HingedRigidBodyMsgPayload
        Output vector of messages for the rotational degree of freedom scalar states
    output translatingBodyOutMsgs LinearTranslationRigidBodyMsgPayload
        Output vector of messages for the translational degree of freedom scalar states
    output generalSingleBodyConfigLogOutMsg SCStatesMsgPayload
        Output message for the effector inertial states

Module Overview
---------------
The basic capability of this module is to connect a generalized rigid body component to the spacecraft hub. An in-depth
discussion of the functionality of this module and the dynamics derivation required to simulate this type of component
motion is provided in the following conference paper

.. note::

    `"Spacecraft Backsubstitution Dynamics for Generalized Rigid Body Component Motion" <https://www.researchgate.net/publication/411054045_Spacecraft_Backsubstitution_Dynamics_for_Generalized_Rigid_Body_Component_Motion>`_,
    Leah Kiner, Hanspeter Schaub
    AAS/AIAA Astrodynamics Specialist Conference, July 26-30, Whistler, British Columbia, Canada

Module Testing
--------------
Currently there is a single unit test script for this module. There are two separate tests within the script which
individually test the rotational and translational functionality of the general effector. Earth's gravity is added to
all simulations. The first test sets up a spacecraft with a single-axis generally rotating rigid body attached to a
rigid hub and configures either pure rotational motion or coupled screw motion where the general body also translates
relative to the rigid hub. The effector is provided with an optional command torque or a reference rotational angle.
The second test sets up a spacecraft with a single-axis generally translating rigid body attached to a rigid hub and
configures either pure translational motion, or coupled screw motion where the general body also rotates relative
to the rigid hub. The effector is provided with an optional command force or a reference displacement. The principles
of conservation of energy and angular momentum are used to verify the general effector dynamics.

Future work involves adding additional scripts to test the effector multi-degree-of-freedom functionality.

User Guide
----------
This section outlines how to set up the general effector in python using Basilisk.

#. Import the generalSingleBodyStateEffector class::

    from Basilisk.simulation import generalSingleBodyStateEffector

#. Create the general state effector::

    general_body = generalSingleBodyStateEffector.GeneralSingleBodyStateEffector()
    general_body.ModelTag = "generalBody"
    general_body.setMass(20.0)
    general_body.setIPntGc_G([[50.0, 0.0, 0.0],
                              [0.0, 80.0, 0.0],
                              [0.0, 0.0, 60.0]])
    general_body.setR_GcG_G(np.array([0.1, -0.1, 0.1]))
    general_body.setR_G0B_B(np.array([-0.1, 0.1, 0.1]))
    general_body.setDCM_G0B(np.array([[0.0, -1.0, 0.0],
                                      [0.0, 0.0, -1.0],
                                      [1.0, 0.0, 0.0]]))

#. Create the general effector degree of freedom. For a translational degree of freedom::

    one_dof_translation = generalSingleBodyStateEffector.DOF()
    one_dof_translation.setDOFAxis(np.array([0.0, 0.0, 1.0]))
    one_dof_translation.setBetaInit(0.1)
    one_dof_translation.setBetaDotInit(0.0)
    one_dof_translation.setSpringConstantK(100.0)

#. Add the translational degree of freedom to the general effector::

    general_body.addTransDOF(one_dof_translation)

#. Add the general effector to the spacecraft::

    sc_object.addStateEffector(general_body)

#. Make sure to connect the required messages for this module.

#. Add the module to the task list::

    test_sim.AddModelToTask(task_name, general_body)

See the example script :ref:`scenarioGeneralSingleBodies` for more information about how to set up general hub-relative
multi-body motion using this state effector module.
