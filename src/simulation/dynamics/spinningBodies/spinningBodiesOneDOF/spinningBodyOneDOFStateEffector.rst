
Executive Summary
-----------------

The spinning body class is an instantiation of the state effector abstract class. The integrated test is validating the interaction between the spinning body module and the rigid body hub that it is attached to. In this case, a 1-DoF spinning body has an inertia tensor and is attached to the hub by a single degree of freedom axis. This module can represent multiple different effectors, such as a hinged solar array, a reaction wheel or a single-gimbal. The spinning axis is fixed in the body frame and the effector is rigid, which means that its center of mass location does not move in the spinning frame S. No damping is included in this module, although an optional motor torque can be applied on the spinning axis.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - spinningBodyOutMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Output message containing the spinning body state angle and angle rate
    * - motorTorqueInMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - (Optional) Input message of the motor torque value
    * - spinningBodyConfigLogOutMsg
      - :ref:`SCStatesMsgPayload`
      - Output message containing the spinning body inertial position and attitude states


Detailed Module Description
---------------------------

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
See the following conference paper
for a detailed description of this model. A 1 DoF spinning body has 2 states: ``theta`` and ``thetaDot``.

.. note::

    J. Vaz Carneiro, C. Allard and H. Schaub, “Rotating Rigid Body Dynamics
    Architecture for Spacecraft Simulation Software Implementation,” AAS Rocky
    Mountain GN&C Conference, Breckenridge, CO, Feb. 2–8, 2023

User Guide
----------
This section is to outline the steps needed to setup a Spinning Body State Effector in Python using Basilisk.

#. Import the spinningBodyStateEffector class::

    from Basilisk.simulation import spinningBodyStateEffector

#. Create an instantiation of a Spinning body::

    spinningBody = spinningBodyStateEffector.SpinningBodyStateEffector()

#. Define all physical parameters for a Spinning Body. For example::

    spinningBody.mass = 100.0
    spinningBody.k = 2
    spinningBody.c = 0.5
    spinningBody.IPntSc_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    spinningBody.dcm_S0B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.r_ScS_S = [[0.5], [0.0], [1.0]]
    spinningBody.r_SB_B = [[1.5], [-0.5], [2.0]]
    spinningBody.sHat_S = [[0], [0], [1]]


#. (Optional) Define initial conditions of the effector.  Default values are zero states::

    spinningBody.thetaInit = 5 * macros.D2R
    spinningBody.thetaDotInit = 1 * macros.D2R

#. (Optional) Define a unique name for each state.  If you have multiple spinning bodies, they each must have a unique name.  If these names are not specified, then the default names are used which are incremented by the effector number::

    spinningBody.nameOfThetaState = "spinningBodyTheta"
    spinningBody.nameOfThetaDotState = "spinningBodyThetaDot"

#. The angular states of the body are created using an output message ``spinningBodyOutMsg``.

#. The spinning body config log state output message is ``spinningBodyConfigLogOutMsg``.

#. Add the panel to your spacecraft::

    scObject.addStateEffector(spinningBody)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, spinningBody)

