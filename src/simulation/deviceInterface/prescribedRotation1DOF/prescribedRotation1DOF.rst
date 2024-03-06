Executive Summary
-----------------
This module profiles a 1 DOF rotation for a spinning rigid body connected to a rigid spacecraft hub. The body frame
of the spinning body is designated by the frame :math:`\mathcal{F}`. The spinning body's states are profiled
relative to a hub-fixed frame :math:`\mathcal{M}`. The :ref:`PrescribedRotationMsgPayload` message
is used to output the prescribed rotational states from the module. The prescribed states profiled in this module
are: ``omega_FM_F``, ``omegaPrime_FM_F``, and ``sigma_FM``. This module has two options to profile the spinning body
rotation. The first option is a bang-bang acceleration profile that minimizes the time required for the rotation.
The second option is a bang-off-bang acceleration profile that adds a coast period of zero acceleration between the
acceleration ramp segments. The module defaults to the bang-bang option with no coast period. If the coast option is
desired, the user must set the ramp time module variable ``coastOptionRampDuration`` to a nonzero value.

.. important::
    Note that this module assumes the initial and final spinning body hub-relative angular rates are zero.

The general inputs to this module that must be set by the user are the spinning body rotation axis expressed as a
unit vector in Mount frame components ``rotHat_M``, the initial spinning body angle relative to the Mount frame
``thetaInit``, the reference spinning body angle relative to the Mount frame ``thetaRef``, the maximum scalar angular
acceleration for the rotation ``thetaDDotMax``, and the ramp time variable ``coastOptionRampDuration`` for specifying
the time the acceleration ramp segments are applied if the coast option is desired. This variable is defaulted to zero,
meaning that the module defaults to the bang-bang acceleration profile with no coast period.

.. important::
    To use this module for prescribed motion, it must be connected to the :ref:`PrescribedMotionStateEffector`
    dynamics module. This ensures the spinning body's states are correctly incorporated into the spacecraft dynamics.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what the message is used for.

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
    * - spinningBodyOutMsgC
      - :ref:`HingedRigidBodyMsgPayload`
      - C-wrapoped output message with the scalar spinning body states
    * - prescribedRotationOutMsg
      - :ref:`PrescribedRotationMsgPayload`
      - output message with the prescribed spinning body rotational states
    * - prescribedRotationOutMsgC
      - :ref:`PrescribedRotationMsgPayload`
      - C-wrapped output message with the prescribed spinning body rotational states

Detailed Module Description
---------------------------

Profiler With No Coast Period
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The first option to profile the spinning body rotation is a pure bang-bang acceleration profile. If the spinning
body reference angle is greater than the given initial angle, the user-specified maximum angular acceleration value
is applied positively to the first half of the rotation and negatively to the second half of the rotation.
However, if the reference angle is less than the initial spinning body angle, the acceleration is instead applied
negatively during the first half of the rotation and positively during the second half of the rotation. As a result
of this acceleration profile, the spinning body's angle rate changes linearly with time and reaches a maximum
in magnitude halfway through the rotation. Note that the angle rate is assumed to both start and end at zero
in this module. The resulting spinning body hub-relative angle for the rotation is parabolic in time.

To profile this spinning body motion, the spinning body's hub-relative scalar states :math:`\theta`,
:math:`\dot{\theta}`, and :math:`\ddot{\theta}` are prescribed as a function of time. During the first half of the
rotation the states are:

.. math::
    \ddot{\theta}(t) = \pm \ddot{\theta}_{\text{max}}

.. math::
    \dot{\theta}(t) = \ddot{\theta} (t - t_0) + \dot{\theta}_0

.. math::
    \theta(t) = a (t - t_0)^2 + \theta_0

where

.. math::
    a = \frac{ \theta_{\text{ref}} - \theta_0}{2 (t_s - t_0)^2}

During the second half of the rotation the states are:

.. math::
    \ddot{\theta}(t) = \mp \ddot{\theta}_{\text{max}}

.. math::
    \dot{\theta}(t) = \ddot{\theta} (t - t_f) + \dot{\theta}_0

.. math::
    \theta(t) = b (t - t_f)^2 + \theta_{\text{ref}}

where

.. math::
    b = - \frac{ \theta_{\text{ref}} - \theta_0}{2 (t_s - t_f)^2}

The switch time :math:`t_s` is the simulation time halfway through the rotation:

.. math::
    t_s = t_0 + \frac{\Delta t_{\text{tot}}}{2}

The total time required to complete the rotation :math:`\Delta t_{\text{tot}}` is:

.. math::
    \Delta t_{\text{tot}} = 2 \sqrt{ \frac{| \theta_{\text{ref}} - \theta_0 | }{\ddot{\theta}_{\text{max}}}} = t_f - t_0

Profiler With Coast Period
^^^^^^^^^^^^^^^^^^^^^^^^^^

The second option to profile the spinning body rotation is a bang-coast-bang acceleration profile with an added coast
period between the acceleration segments where the acceleration is zero. Similar to the previous profiler, if the
spinning body reference angle is greater than the given initial angle, the maximum angular acceleration value is applied
positively for the specified ramp time ``coastOptionRampDuration`` to the first segment of the rotation and negatively
to the third segment of the rotation. The second segment of the rotation is the coast period. However, if the reference
angle is less than the initial spinning body angle, the acceleration is instead applied negatively during the first
segment of the rotation and positively during the third segment of the rotation. As a result of this acceleration
profile, the spinning body's hub-relative angle rate changes linearly with time and reaches a maximum in magnitude
at the end of the first segment and is constant during the coast segment. The angle rate returns to zero during the third
segment. The resulting spinning body angle for the rotation is parabolic during the first and third segments and linear
during the coast segment.

To profile this spinning body motion, the spinning body's hub-relative scalar states :math:`\theta`,
:math:`\dot{\theta}`, and :math:`\ddot{\theta}` are prescribed as a function of time. During the first segment of the
rotation the states are:

.. math::
    \ddot{\theta}(t) = \pm \ddot{\theta}_{\text{max}}

.. math::
    \dot{\theta}(t) = \ddot{\theta} (t - t_0) + \dot{\theta}_0

.. math::
    \theta(t) = a (t - t_0)^2 + \theta_0

where

.. math::
    a = \frac{ \theta(t_r) - \theta_0}{2 (t_r - t_0)^2}

and :math:`\theta(t_r)` is the spinning body angle at the end of the first segment:

.. math::
    \theta(t_r) = \pm \frac{1}{2} \ddot{\theta}_{\text{max}} t_{\text{ramp}}^2
                                       + \dot{\theta}_0 t_{\text{ramp}} + \theta_0

.. important::
    Note the distinction between :math:`t_r` and :math:`t_{\text{ramp}}`. :math:`t_{\text{ramp}}` is the time duration
    of the acceleration segment configured by the user as the module variable ``coastOptionRampDuration``.
    :math:`t_r` is the simulation time at the end of the first acceleration segment. :math:`t_r = t_0 + t_{\text{ramp}}`

During the coast segment, the rotation states are:

.. math::
    \ddot{\theta}(t) = 0

.. math::
    \dot{\theta}(t) = \dot{\theta}(t_r) = \ddot{\theta}_{\text{max}} t_{\text{ramp}} + \dot{\theta}_0

.. math::
    \theta(t) = \dot{\theta}(t_r) (t - t_r) + \theta(t_r)

During the third segment, the rotation states are

.. math::
    \ddot{\theta}(t) = \mp \ddot{\theta}_{\text{max}}

.. math::
    \dot{\theta}(t) = \ddot{\theta} (t - t_f) + \dot{\theta}_0

.. math::
    \theta(t) = b (t - t_f)^2 + \theta_{\text{ref}}

where

.. math::
    b = - \frac{ \theta_{\text{ref}} - \theta(t_c) }{(t_c - t_f)^2}

Here :math:`\theta(t_c)` is the spinning body angle at the end of the coast segment:

.. math::
    \theta(t_c) = \theta(t_r) + \Delta \theta_{\text{coast}}

and :math:`\Delta \theta_{\text{coast}}` is the angle traveled during the coast segment:

.. math::
    \Delta \theta_{\text{coast}} = (\theta_{\text{ref}} - \theta_0) - 2 (\theta(t_r) - \theta_0)

:math:`t_c` is the simulation time at the end of the coast segment:

.. math::
    t_c = t_r + \frac{\Delta \theta_{\text{coast}}}{\dot{\theta}(t_r)}

Using the given rotation axis ``rotHat_M``, the scalar states are then transformed to the spinning body
rotational states ``omega_FM_F``, ``omegaPrime_FM_F``, and ``sigma_FM``. The states are then written to the
:ref:`PrescribedRotationMsgPayload` module output message.

Module Testing
^^^^^^^^^^^^^^
The unit test for this module ensures that the 1 DOF rotation is properly profiled for several different
simulation configurations. The unit test profiles two successive rotations for the spinning body to ensure the
module is correctly configured. The initial spinning body angle relative to the spacecraft hub is varied,
along with the two final reference angles and the maximum angular acceleration for the rotation.
The unit test also tests both methods of profiling the rotation, where either a pure bang-bang acceleration
profile can be selected for the rotation, or a bang-off-bang option can be selected where a coast period with zero
acceleration is added between the acceleration ramp segments. To validate the module, the final spinning body angle
at the end of each rotation are checked to match the specified reference angles.

User Guide
----------
The general inputs to this module that must be set by the user are the spinning body rotation axis expressed as a
unit vector in Mount frame components ``rotHat_M``, the initial spinning body angle relative to the Mount frame
``thetaInit``, the reference spinning body angle relative to the Mount frame ``thetaRef``, the maximum scalar angular
acceleration for the rotation ``thetaDDotMax``, and the ramp time variable ``coastOptionRampDuration`` for specifying
the time the acceleration ramp segments are applied if the coast option is desired. This variable is defaulted to zero,
meaning that the module defaults to the bang-bang acceleration profile with no coast period.

This section is to outline the steps needed to set up the prescribed rotational 1 DOF module in python using Basilisk.

#. Import the prescribedRot1DOF class::

    from Basilisk.simulation import prescribedRotation1DOF

#. Create an instantiation of the module::

    prescribedRot1DOF = prescribedRotation1DOF.prescribedRotation1DOF()

#. Define all of the configuration data associated with the module. For example, to configure the coast option::

    prescribedRot1DOF.ModelTag = "prescribedRotation1DOF"
    prescribedRot1DOF.setCoastOptionRampDuration(3.0)  # [s]
    prescribedRot1DOF.setRotHat_M(np.array([0.0, 1.0, 0.0]))
    prescribedRot1DOF.setThetaDDotMax(macros.D2R * 1.0)  # [rad/s^2]
    prescribedRot1DOF.setThetaInit(macros.D2R * 10.0)  # [rad]

#. Connect a :ref:`HingedRigidBodyMsgPayload` message for the spinning body reference angle to the module. For example, the user can create a stand-alone message to specify the reference angle::

    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    hingedRigidBodyMessageData.theta = macros.D2R * 90.0  # [rad]
    hingedRigidBodyMessageData.thetaDot = 0.0  # [rad/s]
    hingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(hingedRigidBodyMessageData)

#. Subscribe the spinning body reference message to the prescribedRotation1DOF module input message::

    prescribedRot1DOF.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage)

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, prescribedRot1DOF)

