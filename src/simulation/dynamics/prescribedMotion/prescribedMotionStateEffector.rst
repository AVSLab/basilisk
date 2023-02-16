
Executive Summary
-----------------
The prescribed motion class is an instantiation of the state effector abstract class. This module describes the dynamics
of a six-degree of freedom (6 DOF) prescribed rigid body connected to a central rigid spacecraft hub. The body frame
for the prescribed body is designated by the frame :math:`\mathcal{F}`. The prescribed body is mounted onto a hub-fixed
interface described by a mount frame :math:`\mathcal{M}` that is fixed with respect to the hub. The prescribed body may
be commanded to translate and rotate in three-dimensional space with respect to the interface it is mounted on.
Accordingly, the prescribed states for the secondary body are written with respect to the mount frame, :math:`\mathcal{M}`. The
prescribed states are: ``r_FM_M``, ``rPrime_FM_M``, ``rPrimePrime_FM_M``, ``omega_FM_F``, ``omegaPrime_FM_F``, and
``sigma_FM``.

The states of the prescribed body are not defined in this module. Therefore, a flight software profiler module must be
connected to this module's :ref:`PrescribedMotionMsgPayload` input message to profile the prescribed body's states as a
function of time. This message connection is required to provide the prescribed body's states to this dynamics module.

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
    * - prescribedMotionInMsg
      - :ref:`PrescribedMotionMsgPayload`
      - Input message for the effector's prescribed states
    * - prescribedMotionOutMsg
      - :ref:`PrescribedMotionMsgPayload`
      - Output message for the effector's prescribed states
    * - prescribedMotionConfigLogOutMsg
      - :ref:`SCStatesMsgPayload`
      - Output message containing the effector's inertial position and attitude states


Detailed Module Description
---------------------------

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
See Kiner et al.'s paper: `Spacecraft Simulation Software Implementation of General Prescribed Motion Dynamics of Two Connected Rigid Bodies <http://hanspeterschaub.info/Papers/Kiner2023.pdf>`__
for a detailed description of the derived prescribed dynamics.

The translational equations of motion are:

.. math::
    m_{\text{sc}} \left [ \ddot{\boldsymbol{r}}_{B/N} + \boldsymbol{c}^{''} + 2 \left ( \boldsymbol{\omega}_{\mathcal{B}/\mathcal{N}} \times \boldsymbol{c}^{'} \right ) + \left ( \dot{\boldsymbol{\omega}}_{\mathcal{B}/\mathcal{N}} \times \boldsymbol{c} \right ) + \boldsymbol{\omega}_{\mathcal{B}/\mathcal{N}} \times \left ( \boldsymbol{\omega}_{\mathcal{B}/\mathcal{N}} \times \boldsymbol{c} \right ) \right ] = \sum \boldsymbol{F}_{\text{ext}}

The rotational equations of motion are:

.. math::
    m_{\text{sc}} [\tilde{\boldsymbol{c}}] \ddot{\boldsymbol{r}}_{B/N} + [I_{\text{sc},B}] \dot{\boldsymbol{\omega}}_{\mathcal{B}/\mathcal{N}} =  \boldsymbol{L}_B -  m_{\text{P}} [\tilde{\boldsymbol{r}}_{F_c/B}] \boldsymbol{r}^{''}_{F_c/B} - \left ( [I^{'}_{\text{sc},B}] + [\tilde{\boldsymbol{\omega}}_{\mathcal{B}/\mathcal{N}}][I_{\text{sc},B}] \right ) \boldsymbol{\omega}_{\mathcal{B}/\mathcal{N}} \\ - \left ( [I^{'}_{\text{P},F_c}] + [\tilde{\boldsymbol{\omega}}_{\mathcal{B}/\mathcal{N}}] [I_{\text{P},F_c}] \right ) \boldsymbol{\omega}_{\mathcal{F}/\mathcal{B}} \ - \ [I_{\text{P},F_c}] \boldsymbol{\omega}^{'}_{\mathcal{F}/\mathcal{B}} \ - \ m_{\text{P}} [\tilde{\boldsymbol{\omega}}_{\mathcal{B}/\mathcal{N}}] [\tilde{\boldsymbol{r}}_{F_c/B}] \boldsymbol{r}^{'}_{F_c/B}

Module Testing
^^^^^^^^^^^^^^
The unit test for this module is an integrated test with two flight software profiler modules. This is required
because the dynamics module must be connected to a flight software profiler module to define the states of the
prescribed secondary body that is connected to the rigid spacecraft hub. The integrated test for this module has
two simple scenarios it is testing. The first scenario prescribes a 1 DOF rotational attitude maneuver for the
prescribed body using the :ref:`prescribedRot1DOF` flight software module. The second scenario prescribes a
translational maneuver for the prescribed body using the :ref:`prescribedTrans` flight software module.

The unit test ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
initial and reference PRV angles and maximum angular accelerations. The final prescribed angle ``theta_FM_Final``
and angular velocity magnitude ``thetaDot_Final`` are compared with the reference values ``theta_Ref`` and
``thetaDot_Ref``, respectively. The unit test also ensures that the profiled translational maneuver is properly computed for a
series of initial and reference positions and maximum accelerations. The final prescribed position magnitude
``r_FM_M_Final`` and velocity magnitude ``rPrime_FM_M_Final`` are compared with the reference values ``r_FM_M_Ref``
and ``rPrime_FM_M_Ref``, respectively. Additionally for each scenario, the conservation quantities of orbital angular momentum,
rotational angular momentum, and orbital energy are checked to validate the module dynamics.

User Guide
----------
This section is to outline the steps needed to setup a Prescribed Motion State Effector in python using Basilisk.

#. Import the prescribedMotionStateEffector class::

    from Basilisk.simulation import prescribedMotionStateEffector

#. Create an instantiation of a prescribed body::

    platform = prescribedMotionStateEffector.PrescribedMotionStateEffector()

#. Define all physical parameters for the state effector::

    platform.mass = 100.0
    platform.IPntFc_F = [[50.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    platform.r_MB_B = np.array([0.0, 0.0, 0.0])
    platform.r_FcF_F = np.array([0.0, 0.0, 0.0])
    platform.r_FM_M = np.array([1.0, 0.0, 0.0])
    platform.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    platform.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
    platform.omega_FM_F = np.array([0.0, 0.0, 0.0])
    platform.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
    platform.sigma_FM = np.array([0.0, 0.0, 0.0])
    platform.omega_MB_B = np.array([0.0, 0.0, 0.0])
    platform.omegaPrime_MB_B = np.array([0.0, 0.0, 0.0])
    platform.sigma_MB = np.array([0.0, 0.0, 0.0])
    platform.ModelTag = "Platform"

Do this for all of the public parameters in the prescribed motion state effector module. Note that if these parameters
are not set by the user, all scalar and vector quantities are set to zero and all matrices are set to identity by
default.

#. Add the prescribed state effector to your spacecraft::

    scObject.addStateEffector(platform)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Make sure to connect the required messages for this module.

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, platform)




