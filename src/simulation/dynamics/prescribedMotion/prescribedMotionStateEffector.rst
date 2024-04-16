
Executive Summary
-----------------
The prescribed motion class is an instantiation of the state effector abstract class. This module describes the dynamics
of a rigid sub-component whose three-dimensional motion can be kinematically prescribed relative to a central rigid
spacecraft hub whose body frame is denoted as the frame :math:`\mathcal{B}`. The body frame for the prescribed body is
designated by the frame :math:`\mathcal{F}`. The prescribed body must be commanded to translate and rotate in
three-dimensional space relative to a hub-fixed interface designated as the mount frame :math:`\mathcal{M}`.
Accordingly, the prescribed states for the sub-component are written with respect to the
mount frame :math:`\mathcal{M}`:
``r_FM_M``, ``rPrime_FM_M``, ``rPrimePrime_FM_M``, ``omega_FM_F``, ``omegaPrime_FM_F``, and ``sigma_FM``.

The states of the prescribed body are not defined or integrated in this module. No equations of motion exist that need
to be integrated for this type of state effector. Therefore, separate kinematic profiler modules must
be connected to this module's prescribed motion :ref:`PrescribedTranslationMsgPayload` and
:ref:`PrescribedRotationMsgPayload` input messages to profile the prescribed sub-component's states as a function of
time. These message connections are required to provide the sub-component's states to this dynamics module.
Note that either a single profiler can be connected to both of these input messages, or two separate profiler modules
can be used; where one profiles the sub-component's translational states and the other profiles the sub-component's
rotational states. See the example script :ref:`scenarioDeployingSolarArrays` for more information about how to set up
hub-relative multi-body prescribed motion using this state effector module and the associated kinematic profiler modules.

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


Detailed Module Description
---------------------------

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
See Kiner et al.'s paper: `Spacecraft Simulation Software Implementation of General Prescribed Motion Dynamics of Two Connected Rigid Bodies <http://hanspeterschaub.info/Papers/Kiner2023.pdf>`__
for a detailed description of the derived prescribed motion spacecraft dynamics.

The translational equations of motion are:

.. math::
    m_{\text{sc}} \left [ \ddot{\boldsymbol{r}}_{B/N} + \boldsymbol{c}^{''} + 2 \left ( \boldsymbol{\omega}_{\mathcal{B}/\mathcal{N}} \times \boldsymbol{c}^{'} \right ) + \left ( \dot{\boldsymbol{\omega}}_{\mathcal{B}/\mathcal{N}} \times \boldsymbol{c} \right ) + \boldsymbol{\omega}_{\mathcal{B}/\mathcal{N}} \times \left ( \boldsymbol{\omega}_{\mathcal{B}/\mathcal{N}} \times \boldsymbol{c} \right ) \right ] = \sum \boldsymbol{F}_{\text{ext}}

The rotational equations of motion are:

.. math::
    m_{\text{sc}} [\tilde{\boldsymbol{c}}] \ddot{\boldsymbol{r}}_{B/N} + [I_{\text{sc},B}] \dot{\boldsymbol{\omega}}_{\mathcal{B}/\mathcal{N}} =  \boldsymbol{L}_B -  m_{\text{P}} [\tilde{\boldsymbol{r}}_{F_c/B}] \boldsymbol{r}^{''}_{F_c/B} - \left ( [I^{'}_{\text{sc},B}] + [\tilde{\boldsymbol{\omega}}_{\mathcal{B}/\mathcal{N}}][I_{\text{sc},B}] \right ) \boldsymbol{\omega}_{\mathcal{B}/\mathcal{N}} \\ - \left ( [I^{'}_{\text{P},F_c}] + [\tilde{\boldsymbol{\omega}}_{\mathcal{B}/\mathcal{N}}] [I_{\text{P},F_c}] \right ) \boldsymbol{\omega}_{\mathcal{F}/\mathcal{B}} \ - \ [I_{\text{P},F_c}] \boldsymbol{\omega}^{'}_{\mathcal{F}/\mathcal{B}} \ - \ m_{\text{P}} [\tilde{\boldsymbol{\omega}}_{\mathcal{B}/\mathcal{N}}] [\tilde{\boldsymbol{r}}_{F_c/B}] \boldsymbol{r}^{'}_{F_c/B}

Module Testing
^^^^^^^^^^^^^^

There are two integrated unit test scripts written for this module.

The first integrated test uses the :ref:`prescribedRotation1DOF` kinematic profiler module to prescribe a
1-degree-of-freedom (1-DOF) rotation for the prescribed state effector relative to the spacecraft hub.
The second integrated test uses the :ref:`prescribedLinearTranslation` kinematic profiler module to prescribe linear
translational motion for the prescribed state effector relative to the spacecraft hub.

Both integrated test scripts verify the prescribed motion state effector dynamics by checking to ensure that the
orbital angular momentum, orbital energy, and spacecraft rotational angular momentum quantities are reasonably
conserved.


User Guide
----------
This section outlines the steps needed to set up the prescribed motion state effector module in python using Basilisk.

#. Import the prescribedMotionStateEffector class::

    from Basilisk.simulation import prescribedMotionStateEffector

#. Create the prescribed motion state effector::

    prescribed_motion_body = prescribedMotionStateEffector.PrescribedMotionStateEffector()

#. Define the prescribed motion state effector module parameters::

    prescribed_motion_body.setMass(10.0)  # [kg]
    prescribed_motion_body.setIPntFc_F([[50.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]])  # [kg-m^2]
    prescribed_motion_body.setR_MB_B([1.0, 0.0, 0.0])  # [m]
    prescribed_motion_body.setR_FcF_F([0.1, 0.0, -0.5])  # [m]
    prescribed_motion_body.setR_FM_M([0.0, 0.0, 0.0])  # [m]
    prescribed_motion_body.setRPrime_FM_M([0.0, 0.0, 0.0])  # [m/s]
    prescribed_motion_body.setRPrimePrime_FM_M([0.0, 0.0, 0.0])  # [m/s^2]
    prescribed_motion_body.setOmega_FM_F([0.0, 0.0, 0.0])  # [rad/s]
    prescribed_motion_body.setOmegaPrime_FM_F([0.0, 0.0, 0.0])   # [rad/s^2]
    prescribed_motion_body.setSigma_FM([0.0, 0.0, 0.0])
    prescribed_motion_body.setOmega_MB_M([0.0, 0.0, 0.0])  # [rad/s]
    prescribed_motion_body.setOmegaPrime_MB_B([0.0, 0.0, 0.0])  # [rad/s^2]
    prescribed_motion_body.setSigma_MB([0.0, 0.0, 0.0])
    prescribed_motion_body.ModelTag = "prescribedMotionBody"

Note that if these parameters are not set by the user, the vector quantities are set to zero and the matrix and scalar
quantities are set to identity by default.

#. Add the prescribed state effector to the spacecraft object::

    scObject.addStateEffector(prescribed_motion_body)

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, prescribed_motion_body)

Make sure to connect the required messages for this module using the kinematic profiler modules if the prescribed motion
body is to be actuated relative to the spacecraft hub. See the example script :ref:`scenarioDeployingSolarArrays` for
more information about how to set up hub-relative multi-body prescribed motion using this state effector module and the
associated kinematic profiler modules.


