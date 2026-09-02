
Executive Summary
-----------------

The N-DoF linear translation body class is an instantiation of the state effector abstract class
with :math:`N` degrees of freedom. The integrated test is validating the interaction between the
linear translation body module and the rigid body hub that it is attached to. In this case, a 4-DoF
linear translation body has an inertia tensor and is attached to the hub by four
single-degree-of-freedom axes. Each translating axis is fixed in the parent's body frame and each
body is rigid, which means that its center of mass location does not move in the :math:`F` frame. An
optional motor force can be applied on each translating axis, and the user can also lock an axis
through a command. Moreover, the user can input a displacement reference that the effector will
track through a spring and damper.

Nominally, each degree of freedom corresponds to an additional rigid body link. However, by setting
the mass and the inertia of a body to 0, several single axis bodies can compose one multiple degree
of freedom joint instead.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. bsk-module-io:: linearTranslationNDOFStateEffector
    :caption: Module I/O Messages

    output translatingBodyOutMsgs LinearTranslationRigidBodyMsgPayload
        Output vector of messages containing the linear translation body state displacement and displacement rate.
    input motorForceInMsg ArrayMotorForceMsgPayload
        (Optional) Input message of the motor force value for every axis.
    input motorLockInMsg ArrayEffectorLockMsgPayload
        (Optional) Input message for locking each axis.
    input translatingBodyRefInMsgs LinearTranslationRigidBodyMsgPayload
        (Optional) Input vector of messages for prescribing the displacement and displacement rate.
    output translatingBodyConfigLogOutMsgs SCStatesMsgPayload
        Output vector of messages containing the translating body inertial states. The position and
        velocity are those of the body center of mass, and the attitude and angular velocity are
        those of the body frame F.


Detailed Module Description
---------------------------

For each degree of freedom, the user must create a translating body inside the :math:`N`-DoF module. Each body represents a link and has 2 states: ``rho`` and ``rhoDot``. The displacement and displacement rate can change due to the interaction with the hub, but also because of applied forces (control, spring and damper). The displacement remains fixed and the displacement rate is set to zero when the axis is locked.

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
See the following conference paper for a detailed description of this model.

.. note::

    P. Johnson and J. Vaz Carneiro, "`Backsubstitution Method For Spacecraft With Generally Translating Appendages <https://hanspeterschaub.info/Papers/VazCarneiro2024b.pdf>`_,"
    AAS Astrodynamics Specialist Conference, Broomfield, CO, Aug. 11-15, 2024

User Guide
----------
This section is to outline the steps needed to setup a Translating Body State Effector in Python using Basilisk.

#. Import the linearTranslationNDOFStateEffector class::

    from Basilisk.simulation import linearTranslationNDOFStateEffector

#. Create an instantiation of a Translating body::

    translatingBodyEffector = linearTranslationNDOFStateEffector.LinearTranslationNDOFStateEffector()

#. For each degree of freedom, create and set the properties of a translating body.  A body may carry zero mass and zero inertia, subject to the condition in the note below::

    translatingBody = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody.setMass(50.0)
    translatingBody.setIPntFc_F([[100.0, 0.0, 0.0],
                                 [0.0, 80.0, 0.0],
                                 [0.0, 0.0, 50.0]])
    translatingBody.setDCM_FP([[0.0, -1.0, 0.0],
                               [0.0, 0.0, -1.0],
                               [1.0, 0.0, 0.0]])
    translatingBody.setR_FcF_F([[0.8],
                                [0.5],
                                [-0.3]])
    translatingBody.setR_F0P_P([[0.1],
                                [-0.2],
                                [0.4]])
    translatingBody.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBodyEffector.addTranslatingBody(translatingBody)

#. (Optional) Define initial conditions of the effector.  Default values are zero states::

    translatingBody.setRhoInit(1.0)
    translatingBody.setRhoDotInit(0.05)

#. (Optional) Define spring and damper coefficients.  Default values are zero::

    translatingBody.setK(100.0)
    translatingBody.setC(0.0)

   .. note::

       Initialization rejects a chain whose joint mass matrix is singular, which happens exactly
       when some nonzero combination of joint rates leaves every body carrying mass stationary. A
       massless outermost body and a massless body sharing its axis with the body outboard of it are
       the simplest cases, but the condition is collective rather than pairwise. Massless stages
       along :math:`\hat{x}` and :math:`\hat{y}` followed by a massive stage along
       :math:`\hat{x} + \hat{y}` have pairwise independent axes and are still rejected, because the
       three axes span only two dimensions.

#. (Optional) Define a unique name for each state.  If you have multiple effectors, they each must have a unique name.  If these names are not specified, then the default names are used which are incremented by the effector number::

    translatingBodyEffector.setNameOfRhoState("translatingBodyRho")
    translatingBodyEffector.setNameOfRhoDotState("translatingBodyRhoDot")

#. (Optional) Connect a command force message, which carries one force per degree of freedom::

    cmdArray = messaging.ArrayMotorForceMsgPayload()
    cmdArray.motorForce = [cmdForce]  # [N]
    cmdMsg = messaging.ArrayMotorForceMsg().write(cmdArray)
    translatingBodyEffector.motorForceInMsg.subscribeTo(cmdMsg)

#. (Optional) Connect an axis-locking message, which carries one flag per degree of freedom (0 means the axis is free to move and 1 locks the axis)::

    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.effectorLockFlag = [1]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    translatingBodyEffector.motorLockInMsg.subscribeTo(lockMsg)

#. (Optional) Connect a displacement and displacement rate reference message to any degree of freedom::

    translationRef = messaging.LinearTranslationRigidBodyMsgPayload()
    translationRef.rho = 0.2
    translationRef.rhoDot = 0.0
    translationRefMsg = messaging.LinearTranslationRigidBodyMsg().write(translationRef)
    translatingBodyEffector.translatingBodyRefInMsgs[0].subscribeTo(translationRefMsg)

#. The linear states of each body are created using the output message vector ``translatingBodyOutMsgs``.

#. The translating body config log state output message vector is ``translatingBodyConfigLogOutMsgs``.

#. Add the effector to your spacecraft::

    scObject.addStateEffector(translatingBodyEffector)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

Hosting a Dynamic Effector
--------------------------
This effector supports the branching described in :ref:`bskPrinciples-11`, so a compatible
dynamic effector can be carried by one of the translating bodies rather than by the hub::

    translatingBodyEffector.addDynamicEffector(childEffector, segment)

Here ``segment`` is the one-based translating body number, counting outward from the hub, so ``1``
is the translating body attached to the hub.

The child then reads this effector's inertial position, velocity, attitude, and angular velocity
in place of the hub's, and any geometry given to the child is expressed in that translating body's
frame rather than the hub body frame. Both this effector and the child are still added to the task
in the usual way.
