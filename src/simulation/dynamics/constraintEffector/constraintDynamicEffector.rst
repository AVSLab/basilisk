
Executive Summary
-----------------

The constraint effector class is used to define a physical connection between two separate
spacecraft. This class takes in the connection location on each spacecraft in their respective
body frames as well as a relative vector between these two connection points. It also takes in
two gain tuning parameters to tailor the stiffness and damping of the connection between the
two spacecraft.

The constraint effector class is an instantiation of the dynamic effector abstract class. The
included test is validating the interaction between two spacecraft rigid body hubs that are
attached through a constraint effector. In this case, two identical spacecraft are connected
by a 0.1 meter long arm which is enforced with high stiffness and damping to be virtually rigid.

Detailed Module Description
---------------------------

A constraint effector is a combination of two separate 3-degree-of-freedom holonomic constraints: a
direction constraint which enforces the position of the connection point on each spacecraft
relative to the other, and an attitude constraint which enforces the attitude of each spacecraft
relative to the other. The constraint effector works by correcting violations of these constraints
as well as their derivatives.

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
See the following conference paper for a detailed description of the mathematical model.

.. note::

    J. Vaz Carneiro, A. Morell and H. Schaub, `"Post-Docking Complex
    Spacecraft Dynamics Using Baumgarte Stabilization" <https://hanspeterschaub.info/Papers/VazCarneiro2023b.pdf>`_,
    AAS/AIAA Astrodynamics Specialist Conference, Big Sky, Montana, Aug. 13-17, 2023

User Guide
----------
This section outlines the steps needed to setup a Constraint Dynamic Effector in Python using Basilisk.

#. Import the constraintDynamicEffector class::

    from Basilisk.simulation import constraintDynamicEffector

#. Create an instantiation of a holonomic constraint::

    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()

#. Define all physical parameters for the constraint::

    constraintEffector.setR_P1B1_B1(r_P1B1_B1)
    constraintEffector.setR_P2B2_B2(r_P2B2_B2)
    constraintEffector.setR_P2P1_B1Init(r_P2P1_B1Init)

#. Define the stiffness and damping of the connection. See the recommended starting gains here::

    constraintEffector.setAlpha(1E2)
    constraintEffector.setBeta(1e3)

#. (Optional) Define exact gains for the direction and attitude constraints separately. These are internally set based on alpha and beta during reset, but can be overridden in this way::

    constraintEffector.setK_d(alpha**2)
    constraintEffector.setC_d(2*beta)
    constraintEffector.setK_a(alpha**2)
    constraintEffector.setC_a(2*beta)

#. Add the effector to both spacecraft::

    scObject1.addDynamicEffector(constraintEffector)
    scObject2.addDynamicEffector(constraintEffector)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Add the module to the task list::

    Sim.AddModelToTask(TaskName, constraintEffector)

#. Ensure that the dynamics integration is synced between the connected spacecraft. For best results use a variable timestep integrator::

    integratorObject1 = svIntegrators.svIntegratorRKF45(scObject1)
    scObject1.setIntegrator(integratorObject1)
    scObject1.syncDynamicsIntegration(scObject2)

#. (Optional) Retrieve the constraint effector's physical parameters, gain tuning parameters, or exact gains for the direction and attitude constraints::

    constraintEffector.getR_P1B1_B1(r_P1B1_B1)
    constraintEffector.getR_P2B2_B2(r_P2B2_B2)
    constraintEffector.getR_P2P1_B1Init(r_P2P1_B1Init)
    constraintEffector.getAlpha(1E2)
    constraintEffector.getBeta(1e3)
    constraintEffector.getK_d(alpha**2)
    constraintEffector.getC_d(2*beta)
    constraintEffector.getK_a(alpha**2)
    constraintEffector.getC_a(2*beta)

#. (Optional) Define a input device status message.(1 means constraintEffector is connected.0 means constraintEffector is disconnected). If not set, it defaults to being connected::

    effectorStatusMsgPayload = messaging.DeviceStatusMsgPayload()
    effectorStatusMsgPayload.deviceStatus = 1
    effectorStatusMsg = messaging.DeviceStatusMsg().write(effectorStatusMsgPayload)
    constraintEffector.effectorStatusInMsg.subscribeTo(effectorStatusMsg)

#. (Optional) Setup Low Pass Filtering for the Constraint Forces and Torques acting on the two satellites. Define a positive cut-off frequency wc for the low-pass filter. If not set, defaults to 0::

    constraintEffector.setFilterData(wc)

#. The constraintEffector output message records the raw and filtered constraint forces and torques acting on the two spacecraft using the variable ``constraintElements``.
