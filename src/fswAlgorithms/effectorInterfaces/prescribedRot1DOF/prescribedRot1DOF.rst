Executive Summary
-----------------
This module profiles a :ref:`PrescribedMotionMsgPayload` message for a specified 1 DOF rotational attitude maneuver
for a secondary rigid body connected to a rigid spacecraft hub at a hub-fixed location, :math:`\mathcal{M}`. The body
frame for the prescribed body is designated by the frame :math:`\mathcal{F}`. Accordingly, the prescribed states for the
secondary body are written with respect to the mount frame, :math:`\mathcal{M}`. The prescribed states are: ``r_FM_M``,
``rPrime_FM_M``, ``rPrimePrime_FM_M``, ``omega_FM_F``, ``omegaPrime_FM_F``, and ``sigma_FM``. Because this is a
purely rotational profiler, the translational states ``r_FM_M``, ``rPrime_FM_M``, and ``rPrimePrime_FM_M`` are held
constant in this module.

To use this module for prescribed motion purposes, it must be connected to the :ref:`PrescribedMotionStateEffector`
dynamics module in order to profile the states of the secondary body. The required maneuver is determined from the
user-specified scalar maximum angular acceleration for the attitude maneuver :math:`\alpha_{\text{max}}`, prescribed
body's initial attitude with respect to the mount frame as the Principal Rotation Vector ``prv_F0M``
:math:`(\Phi_0, \hat{\textbf{{e}}}_0)`, and the prescribed body's reference attitude with respect to the mount frame as
the Principal Rotation Vector ``prv_F1M`` :math:`(\Phi_1, \hat{\textbf{{e}}}_1)`.

The maximum scalar angular acceleration is applied constant and positively for the first half of the maneuver and
constant negatively for the second half of the maneuver. The resulting angular velocity of the prescribed body is
linear, approaching a maximum magnitude halfway through the maneuver and ending with zero residual velocity.
The corresponding angle the prescribed body moves through during the maneuver is parabolic in time.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - spinningBodyInMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - input msg with the spinning body reference states
    * - spinningBodyOutMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - output message with the scalar spinning body states
    * - prescribedMotionOutMsg
      - :ref:`PrescribedMotionMsgPayload`
      - output message with the prescribed spinning body states



Detailed Module Description
---------------------------
This 1 DOF rotational motion flight software module is written to profile spinning body motion with respect to a 
body-fixed mount frame. The inputs to the profiler are the scalar maximum angular acceleration for the attitude maneuver 
:math:`\alpha_{\text{max}}`, the prescribed body's initial attitude with respect to the mount frame as the Principal 
Rotation Vector ``prv_F0M`` :math:`(\Phi_0, \hat{\textbf{{e}}}_0)`, and the prescribed body's reference attitude with respect to the
mount frame as the Principal Rotation Vector ``prv_F1M`` :math:`(\Phi_1, \hat{\textbf{{e}}}_1)`. The prescribed body is
assumed to be non-rotating at the beginning of the attitude maneuver.
    
Subtracting the initial principal rotation vector from the reference principal rotation vector gives the required 
rotation angle and axis for the maneuver:

.. math::
    \Phi_{\text{ref}} = 2 \cos^{-1} \left ( \cos \frac{\Phi_1}{2} \cos \frac{\Phi_0}{2} + \sin \frac{\Phi_1}{2} \sin \frac {\Phi_0}{2} \hat{\textbf{{e}}}_1 \cdot \hat{\textbf{{e}}}_0 \right )

.. math::
    \hat{\textbf{{e}}} = \frac{\cos \frac{\Phi_0}{2} \sin \frac{\Phi_1}{2} \hat{\textbf{{e}}}_1 - \cos \frac{\Phi_1}{2} \sin \frac{\Phi_0}{2} \hat{\textbf{{e}}}_0 + \sin \frac{\Phi_1}{2} \sin \frac{\Phi_0}{2} \hat{\textbf{{e}}}_1 \times \hat{\textbf{{e}}}_0 }{\sin \frac{\Phi_{\text{ref}}}{2}}

During the first half of the attitude maneuver, the prescribed body is constantly accelerated with the given maximum 
angular acceleration. The prescribed body's angular velocity increases linearly during the acceleration phase and 
reaches a maximum magnitude halfway through the attitude maneuver. The switch time :math:`t_s` is the simulation time 
halfway through the maneuver:
    
.. math::
    t_s = t_0 + \frac{\Delta t}{2}

where the time required for the maneuver :math:`\Delta t` is determined using the inputs to the profiler:
    
.. math::
    \Delta t = t_f - t_0 = 2 \sqrt{ \Phi_{\text{ref}} / \ddot{\Phi}_{\text{max}}}

The resulting trajectory of the angle :math:`\Phi` swept during the first half of the maneuver is parabolic. The profiled 
motion is concave upwards if the reference angle :math:`\Phi_{\text{ref}}` is greater than zero. If the converse is true, 
the profiled motion is instead concave downwards. The described motion during the first half of the attitude maneuver 
is characterized by the expressions:
 
.. math::
    \omega_{\mathcal{F} / \mathcal{M}}(t) = \alpha_{\text{max}}

.. math::
    \dot{\Phi}(t) = \alpha_{\text{max}} (t - t_0)

.. math::
    \Phi(t) = c_1 (t - t_0)^2

where 

.. math::
    c_1 = \frac{\Phi_{\text{ref}}}{2(t_s - t_0)^2}

Similarly, the second half of the attitude maneuver decelerates the prescribed body constantly until it reaches a 
non-rotating state. The prescribed body angular velocity decreases linearly from its maximum magnitude back to zero. 
The trajectory swept during the second half of the maneuver is quadratic and concave downwards if the reference angle 
:math:`\Phi_{\text{ref}}` is positive. If :math:`\Phi_{\text{ref}}` is negative, the profiled motion is instead concave upwards. 
The described motion during the second half of the attitude maneuver is characterized by the expressions:
    
.. math::
    \ddot{\Phi}(t) = -\alpha_{\text{max}}

.. math::
    \dot{\Phi}(t) = \alpha_{\text{max}} (t - t_f)

.. math::
    \Phi(t) = c_2 (t - t_f)^2  + \Phi_{\text{ref}}

 where 

.. math::
    c_2 = \frac{\Phi_{\text{ref}}}{2(t_s - t_f)^2}

Module Testing
^^^^^^^^^^^^^^
The unit test for this module ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
initial and reference PRV angles and maximum angular accelerations. The final prescribed angle ``theta_FM_Final``
and angular velocity magnitude ``thetaDot_Final`` are compared with the reference values ``theta_Ref`` and
``thetaDot_Ref``, respectively.

User Guide
----------
The user-configurable inputs to the profiler are the scalar maximum angular acceleration for the attitude maneuver
:math:`\alpha_{\text{max}}`, the prescribed body's initial attitude with respect to the mount frame as the Principal
Rotation Vector ``prv_F0M`` :math:`(\Phi_0, \hat{\textbf{{e}}}_0)`, and the prescribed body's reference attitude with
respect to the mount frame as the Principal Rotation Vector ``prv_F1M`` :math:`(\Phi_1, \hat{\textbf{{e}}}_1)`.

This module provides two output messages in the form of :ref:`HingedRigidBodyMsgPayload` and
:ref:`PrescribedMotionMsgPayload`. The first guidance message, describing the spinning body's scalar states relative to
the body-fixed mount frame can be directly connected to an attitude feedback control module. The second prescribed
motion output message can be connected to the :ref:`PrescribedMotionStateEffector` dynamics module to directly profile
a state effector's rotational motion.

This section is to outline the steps needed to setup a prescribed 1 DOF rotational module in python using Basilisk.

#. Import the prescribedRot1DOF class::

    from Basilisk.fswAlgorithms import prescribedRot1DOF

#. Create an instantiation of a prescribed rotational 1 DOF C module and the associated C++ container::

    PrescribedRot1DOF = prescribedRot1DOF.prescribedRot1DOF()
    PrescribedRot1DOF.ModelTag = "prescribedRot1DOF"

#. Define all of the configuration data associated with the module. For example::

    thetaInit = 0.0  # [rad]
    rotAxis_M = np.array([1.0, 0.0, 0.0])
    prvInit_FM = thetaInit * rotAxisM
    PrescribedRot1DOF.r_FM_M = np.array([1.0, 0.0, 0.0])
    PrescribedRot1DOF.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOF.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOF.rotAxis_M = rotAxis_M
    PrescribedRot1DOF.thetaDDotMax = 0.01  # [rad/s^2]
    PrescribedRot1DOF.omega_FM_F = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOF.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOF.sigma_FM = rbk.PRV2MRP(prvInit_FM)

The user is required to set the above configuration data parameters, as they are not initialized in the module.

#. Make sure to connect the required messages for this module.

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, PrescribedRot1DOF)

