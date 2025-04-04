Executive Summary
-----------------
This module profiles a 1 DOF rotation for a spinning rigid body connected to a rigid spacecraft hub. The body frame
of the spinning body is designated by the frame :math:`\mathcal{P}`. The spinning body's states are profiled
relative to a hub-fixed frame :math:`\mathcal{M}`. The :ref:`PrescribedRotationMsgPayload` message
is used to output the prescribed rotational states from the module. The prescribed states profiled in this module
are: ``omega_PM_P``, ``omegaPrime_PM_P``, and ``sigma_PM``. This module has four options to profile the spinning body
rotation. The first option is a bang-bang acceleration profile that minimizes the time required for the rotation.
The second option is a bang-coast-bang acceleration profile that adds a coast period of zero acceleration between the
acceleration ramp segments. The third option is a smoothed bang-bang acceleration profile that uses cubic splines to
construct a continuous acceleration profile across the entire rotation. The fourth option is a smoothed
bang-coast-bang acceleration profile.

The module defaults to the non-smoothed bang-bang option with no coast period. If the coast option is desired, the
user must set the module variable ``coastOptionBangDuration`` to a nonzero value. If smoothing is desired,
the module variable ``smoothingDuration`` must be set to a nonzero value.

.. important::
    Note that this module assumes the initial and final spinning body hub-relative angular rates are zero.

The general inputs to this module that must be set by the user are the spinning body rotation axis expressed as a
unit vector in mount frame components ``rotHat_M``, the initial spinning body angle relative to the hub-fixed
mount frame ``thetaInit``, the reference angle relative to the mount frame ``thetaRef``, and the maximum scalar angular
acceleration for the rotation ``thetaDDotMax``. The optional inputs ``coastOptionBangDuration`` and
``smoothingDuration`` can be set by the user to select the specific type of profiler that is desired. If these variables
are not set by the user, the module defaults to the non-smoothed bang-bang profiler. If only the variable
``coastOptionBangDuration`` is set to a nonzero value, the bang-coast-bang profiler is selected. If only the variable
``smoothingDuration`` is set to a nonzero value, the smoothed bang-bang profiler is selected. If both variables are
set to nonzero values, the smoothed bang-coast-bang profiler is selected.

.. important::
    To use this module for prescribed motion, it must be connected to the :ref:`PrescribedMotionStateEffector`
    dynamics module. This ensures the spinning body's states are correctly incorporated into the spacecraft dynamics.
    See the example script :ref:`scenarioDeployingSolarArrays` for more information about how to set up hub-relative
    multi-body prescribed motion using the state effector module together with this profiler module.

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

Non-Smoothed Bang-Bang Profiler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
    a = \frac{ \theta_{\text{ref}} - \theta_0}{2 (t_{b1} - t_0)^2}

During the second half of the rotation the states are:

.. math::
    \ddot{\theta}(t) = \mp \ddot{\theta}_{\text{max}}

.. math::
    \dot{\theta}(t) = \ddot{\theta} (t - t_f) + \dot{\theta}_0

.. math::
    \theta(t) = b (t - t_f)^2 + \theta_{\text{ref}}

where

.. math::
    b = - \frac{ \theta_{\text{ref}} - \theta_0}{2 (t_{b1} - t_f)^2}

The switch time :math:`t_{b1}` is the simulation time at the end of the first bang segment:

.. math::
    t_{b1} = t_0 + \frac{\Delta t_{\text{tot}}}{2}

The total time required to complete the rotation :math:`\Delta t_{\text{tot}}` is:

.. math::
    \Delta t_{\text{tot}} = 2 \sqrt{ \frac{| \theta_{\text{ref}} - \theta_0 | }{\ddot{\theta}_{\text{max}}}} = t_f - t_0

Non-Smoothed Bang-Coast-Bang Profiler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The second option to profile the spinning body rotation is a bang-coast-bang acceleration profile with an added coast
period between the acceleration segments where the acceleration is zero. Similar to the previous profiler, if the
spinning body reference angle is greater than the given initial angle, the maximum angular acceleration value is applied
positively for the specified ramp time ``coastOptionBangDuration`` to the first segment of the rotation and negatively
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
    a = \frac{ \theta(t_{b1}) - \theta_0}{2 (t_{b1} - t_0)^2}

and :math:`\theta(t_{b1})` is the spinning body angle at the end of the first bang segment:

.. math::
    \theta(t_{b1}) = \pm \frac{1}{2} \ddot{\theta}_{\text{max}} t_{\text{bang}}^2
                                       + \dot{\theta}_0 t_{\text{bang}} + \theta_0

.. important::
    Note the distinction between :math:`t_{b1}` and :math:`t_{\text{bang}}`. :math:`t_{\text{bang}}` is the time
    duration of the acceleration segment configured by the user as the module variable ``coastOptionBangDuration``.
    :math:`t_{b1}` is the simulation time at the end of the first acceleration segment.
    :math:`t_{b1} = t_0 + t_{\text{bang}}`

During the coast segment, the rotation states are:

.. math::
    \ddot{\theta}(t) = 0

.. math::
    \dot{\theta}(t) = \dot{\theta}(t_{b1}) = \ddot{\theta}_{\text{max}} t_{\text{bang}} + \dot{\theta}_0

.. math::
    \theta(t) = \dot{\theta}(t_{b1}) (t - t_{b1}) + \theta(t_{b1})

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
    \theta(t_c) = \theta(t_{b1}) + \Delta \theta_{\text{coast}}

and :math:`\Delta \theta_{\text{coast}}` is the angle traveled during the coast segment:

.. math::
    \Delta \theta_{\text{coast}} = (\theta_{\text{ref}} - \theta_0) - 2 (\theta(t_{b1}) - \theta_0)

:math:`t_c` is the simulation time at the end of the coast segment:

.. math::
    t_c = t_{b1} + \frac{\Delta \theta_{\text{coast}}}{\dot{\theta}(t_{b1})}

Using the given rotation axis ``rotHat_M``, the scalar states are then transformed to the spinning body
rotational states ``omega_PM_P``, ``omegaPrime_PM_P``, and ``sigma_PM``. The states are then written to the
:ref:`PrescribedRotationMsgPayload` module output message.

Smoothed Bang-Bang Profiler
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The third option to profile the rotation is a smoothed bang-bang acceleration profile. This option is selected
by setting the module variable ``smoothingDuration`` to a nonzero value. This profiler uses cubic splines to construct
a continuous acceleration profiler across the entire rotation. Similar to the non-smoothed bang-bang profiler,
this option smooths the acceleration between the given maximum acceleration values.
To profile this motion, the spinning body's hub-relative scalar states :math:`\theta`, :math:`\dot{\theta}`, and
:math:`\ddot{\theta}` are prescribed as a function of time and the rotational motion is split into five different
segments.

The first segment smooths the acceleration from zero to the user-specified maximum acceleration value in the given
time ``smoothingDuration``. If the given reference angle is greater than the given initial angle, the
acceleration is smoothed positively to the given maximum acceleration value. If the given reference angle is less
than the given initial angle, the acceleration is smoothed from zero to the negative maximum acceleration value.
During this phase, the scalar hub-relative states are:

.. math::
    \ddot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} \left( \frac{3 (t - t_0)^2}{t_{\text{smooth}}^2} - \frac{2 (t - t_0)^3}{t_{\text{smooth}}^3} \right)

.. math::
    \dot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} \left( \frac{(t - t_0)^3}{t_{\text{smooth}}^2} - \frac{(t - t_0)^4}{2 t_{\text{smooth}}^3} \right)

.. math::
    \theta(t) = \pm \ddot{\theta}_{\text{max}} \left( \frac{(t - t_0)^4}{4 t_{\text{smooth}}^2} - \frac{(t - t_0)^5}{10 t_{\text{smooth}}^3} \right) + \theta_0

The second segment is the first bang segment where the maximum acceleration value is applied either positively or
negatively as discussed previously. The scalar hub-relative states during this phase are:

.. math::
    \ddot{\theta}(t) = \pm \ddot{\theta}_{\text{max}}

.. math::
    \dot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} (t - t_{s1}) + \dot{\theta}(t_{s1})

.. math::
    \theta(t) = \pm \frac{\ddot{\theta}_{\text{max}} (t - t_{s1})^2}{2} + \dot{\theta}(t_{s1})(t - t_{s1}) + \theta(t_{s1})

where :math:`t_{s1}` is the time at the end of the first smoothing segment:

.. math::
    t_{s1} = t_0 + t_{\text{smooth}}

The third segment smooths the acceleration from the current maximum acceleration value to the opposite magnitude maximum
acceleration value. The scalar hub-relative states during this phase are:

.. math::
    \ddot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} \left( 1 - \frac{3 (t - t_{b1})^2}{2 t_{\text{smooth}}^2} + \frac{(t - t_{b1})^3}{2 t_{\text{smooth}}^3} \right)

.. math::
    \dot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} \left( (t - t_{b1}) - \frac{(t - t_{b1})^3}{2 t_{\text{smooth}}^2} + \frac{(t - t_{b1})^4}{8 t_{\text{smooth}}^3} \right) + \dot{\theta}(t_{b1})

.. math::
    \theta(t) = \pm \ddot{\theta}_{\text{max}} \left( \frac{(t - t_{b1})^2}{2} - \frac{(t - t_{b1})^4}{8 t_{\text{smooth}}^2} + \frac{(t - t_{b1})^5}{40 t_{\text{smooth}}^3} \right) + \dot{\theta}(t_{b1})(t - t_{b1}) + \theta(t_{b1})

where :math:`t_{b1}` is the time at the end of the first bang segment:

.. math::
    t_{b1} = t_{s1} + t_{\text{bang}}

The fourth segment is the second bang segment where the maximum acceleration value is applied either positively or
negatively as discussed previously. The scalar hub-relative states during this phase are:

.. math::
    \ddot{\theta}(t) = \mp \ddot{\theta}_{\text{max}}

.. math::
    \dot{\theta}(t) = \mp \ddot{\theta}_{\text{max}} (t - t_{s2}) + \dot{\theta}(t_{s2})

.. math::
    \theta(t) = \mp \frac{\ddot{\theta}_{\text{max}} (t - t_{s2})^2}{2} + \dot{\theta}(t_{s2})(t - t_{s2}) + \theta(t_{s2})

where :math:`t_{s2}` is the time at the end of the second smoothing segment:

.. math::
    t_{s2} = t_{b1} + t_{\text{smooth}}

The fifth segment is the third and final smoothing segment where the acceleration returns to zero. The scalar
hub-relative states during this phase are:

.. math::
    \ddot{\theta}(t) = \mp \ddot{\theta}_{\text{max}} \left ( -1 + \frac{3(t - t_{b2})^2}{t_{\text{smooth}}^2} - \frac{2 (t - t_{b2})^3}{t_{\text{smooth}}^3} \right )

.. math::
    \dot{\theta}(t) = \mp \ddot{\theta}_{\text{max}} \left ( -(t - t_{b2}) + \frac{(t - t_{b2})^3}{t_{\text{smooth}}^2} - \frac{(t - t_{b2})^4}{2 t_{\text{smooth}}^3} \right ) + \dot{\theta}(t_{b2})

.. math::
    \theta(t) = \mp \ddot{\theta}_{\text{max}} \left ( \frac{(t - t_{b2})^2}{2} + \frac{(t - t_{b2})^4}{4 t_{\text{smooth}}^2} - \frac{(t - t_{b2})^5}{10 t_{\text{smooth}}^3} \right ) + \dot{\theta}(t_{b2})(t - t_{b2}) + \theta(t_{b2})

where :math:`t_{b2}` is the time at the end of the second bang segment:

.. math::
    t_{b2} = t_{s2} + t_{\text{bang}}

Smoothed Bang-Coast-Bang Profiler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The fourth option to profile the rotation is a smoothed bang-coast-bang acceleration profile. This option is
selected by setting the module variables ``coastOptionBangDuration`` and ``smoothingDuration`` to nonzero values.
This profiler uses cubic splines to construct a continuous acceleration profiler across the entire rotation.
To profile this motion, the spinning body's hub-relative scalar states :math:`\theta`, :math:`\dot{\theta}`, and
:math:`\ddot{\theta}` are prescribed as a function of time and the rotational motion is split into seven different
segments.

The first segment smooths the acceleration from zero to the user-specified maximum acceleration value in the given
time ``smoothingDuration``. If the given reference angle is greater than the given initial angle, the
acceleration is smoothed positively to the given maximum acceleration value. If the given reference angle is less
than the given initial angle, the acceleration is smoothed from zero to the negative maximum acceleration value.
During this phase, the scalar hub-relative states are:

.. math::
    \ddot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} \left( \frac{3 (t - t_0)^2}{t_{\text{smooth}}^2} - \frac{2 (t - t_0)^3}{t_{\text{smooth}}^3} \right)

.. math::
    \dot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} \left( \frac{(t - t_0)^3}{t_{\text{smooth}}^2} - \frac{(t - t_0)^4}{2 t_{\text{smooth}}^3} \right)

.. math::
    \theta(t) = \pm \ddot{\theta}_{\text{max}} \left( \frac{(t - t_0)^4}{4 t_{\text{smooth}}^2} - \frac{(t - t_0)^5}{10 t_{\text{smooth}}^3} \right) + \theta_0

The second segment is the first bang segment where the maximum acceleration value is applied either positively or
negatively as discussed previously. The scalar hub-relative states during this phase are:

.. math::
    \ddot{\theta}(t) = \pm \ddot{\theta}_{\text{max}}

.. math::
    \dot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} (t - t_{s1}) + \dot{\theta}(t_{s1})

.. math::
    \theta(t) = \pm \frac{\ddot{\theta}_{\text{max}} (t - t_{s1})^2}{2} + \dot{\theta}(t_{s1})(t - t_{s1}) + \theta(t_{s1})

where :math:`t_{s1}` is the time at the end of the first smoothing segment.

The third segment prior to the coast phase smooths the acceleration from the current maximum acceleration value to zero.
The scalar hub-relative states during this phase are:

.. math::
    \ddot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} \left( 1 - \frac{3 (t - t_{b1})^2}{t_{\text{smooth}}^2} - \frac{2 (t - t_{b1})^3}{t_{\text{smooth}}^3} \right)

.. math::
    \dot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} \left( (t - t_{b1}) - \frac{(t - t_{b1})^3}{t_{\text{smooth}}^2} - \frac{(t - t_{b1})^4}{2 t_{\text{smooth}}^3} \right) + \dot{\theta}(t_{b1})

.. math::
    \theta(t) = \pm \ddot{\theta}_{\text{max}} \left( \frac{(t - t_{b1})^2}{2} - \frac{(t - t_{b1})^4}{4 t_{\text{smooth}}^2} - \frac{(t - t_{b1})^5}{10 t_{\text{smooth}}^3} \right) + \dot{\theta}(t_{b1})(t - t_{b1}) + \theta(t_{b1})

where :math:`t_{b1}` is the time at the end of the first bang segment.

The fourth segment is the coast segment where the rotational states are:

.. math::
    \ddot{\theta}(t) = 0

.. math::
    \dot{\theta}(t) = \dot{\theta}(t_{s2})

.. math::
    \theta(t) = \dot{\theta}(t_{s2}) (t - t_{s2}) + \theta(t_{s2})

where :math:`t_{s2}` is the time at the end of the second smoothing segment.

The fifth segment smooths the acceleration from zero to the maximum acceleration value prior to the second bang segment.
The rotational states during this phase are:

.. math::
    \ddot{\theta}(t) = \mp \ddot{\theta}_{\text{max}} \left( \frac{3 (t - t_c)^2}{t_{\text{smooth}}^2} - \frac{2 (t - t_c)^3}{t_{\text{smooth}}^3} \right)

.. math::
    \dot{\theta}(t) = \mp \ddot{\theta}_{\text{max}} \left( \frac{(t - t_c)^3}{t_{\text{smooth}}^2} - \frac{(t - t_c)^4}{2 t_{\text{smooth}}^3} \right) + \dot{\theta}(t_c)

.. math::
    \theta(t) = \mp \ddot{\theta}_{\text{max}} \left( \frac{(t - t_c)^4}{4 t_{\text{smooth}}^2} - \frac{(t - t_c)^5}{10 t_{\text{smooth}}^3} \right) + \dot{\theta}(t_c) (t - t_c) + \theta(t_c)

where :math:`t_c` is the time at the end of the coast segment.

The sixth segment is the second bang segment where the maximum acceleration value is applied either positively or
negatively as discussed previously. The scalar hub-relative states during this phase are:

.. math::
    \ddot{\theta}(t) = \mp \ddot{\theta}_{\text{max}}

.. math::
    \dot{\theta}(t) = \mp \ddot{\theta}_{\text{max}} (t - t_{s3}) + \dot{\theta}(t_{s3})

.. math::
    \theta(t) = \mp \frac{\ddot{\theta}_{\text{max}} (t - t_{s3})^2}{2} + \dot{\theta}(t_{s3})(t - t_{s3}) + \theta(t_{s3})

where :math:`t_{s3}` is the time at the end of the third smoothing segment.

The seventh segment is the fourth and final smoothing segment where the acceleration returns to zero. The scalar
hub-relative states during this phase are:

.. math::
    \ddot{\theta}(t) = \mp \ddot{\theta}_{\text{max}} \left (\frac{3(t_f - t)^2}{t_{\text{smooth}}^2} - \frac{2 (t_f - t)^3}{t_{\text{smooth}}^3} \right )

.. math::
    \dot{\theta}(t) = \pm \ddot{\theta}_{\text{max}} \left (\frac{(t_f - t)^3}{t_{\text{smooth}}^2} - \frac{(t_f - t)^4}{2 t_{\text{smooth}}^3} \right )

.. math::
    \theta(t) = \mp \ddot{\theta}_{\text{max}} \left (\frac{(t_f - t)^4}{4 t_{\text{smooth}}^2} - \frac{(t_f - t)^5}{10 t_{\text{smooth}}^3} \right ) + \theta_{\text{ref}}

where :math:`t_f` is the time at the end of the rotation:

Module Testing
^^^^^^^^^^^^^^
The unit test for this module ensures that the profiled 1 DOF rotation for a secondary rigid body relative to
the spacecraft hub is properly computed for several different simulation configurations. The unit test profiles
two successive rotations to ensure the module is correctly configured. The initial spinning body angle relative
to the spacecraft hub is varied, along with the two final reference angles and the maximum angular acceleration
for the rotation.

The unit test also tests four different methods of profiling the rotation. Two profilers prescribe a pure
bang-bang or bang-coast-bang angular acceleration profile for the rotation. The bang-bang option results in
the fastest possible rotation; while the bang-coast-bang option includes a coast period with zero acceleration
between the acceleration segments. The other two profilers apply smoothing to the bang-bang and bang-coast-bang
acceleration profiles so that the spinning body hub-relative rates start and end at zero.

To verify the module functionality, the final angle at the end of each rotation is checked to match the specified
reference angle. Additionally, for the smoothed profiler options, the numerical derivative of the profiled angles
and their rates is determined across the entire simulation. These numerical derivatives are checked with the module's
acceleration and rate profiles to ensure the profiled acceleration is correctly integrated in the module to obtain
the angles and their rates.

User Guide
----------
The general inputs to this module that must be set by the user are the spinning body rotation axis expressed as a
unit vector in mount frame components ``rotHat_M``, the initial spinning body angle relative to the hub-fixed
mount frame ``thetaInit``, the reference angle relative to the mount frame ``thetaRef``, and the maximum scalar angular
acceleration for the rotation ``thetaDDotMax``. The optional inputs ``coastOptionBangDuration`` and
``smoothingDuration`` can be set by the user to select the specific type of profiler that is desired. If these variables
are not set by the user, the module defaults to the non-smoothed bang-bang profiler. If only the variable
``coastOptionBangDuration`` is set to a nonzero value, the bang-coast-bang profiler is selected. If only the variable
``smoothingDuration`` is set to a nonzero value, the smoothed bang-bang profiler is selected. If both variables are
set to nonzero values, the smoothed bang-coast-bang profiler is selected.

This section is to outline the steps needed to set up the prescribed rotational 1 DOF module in python using Basilisk.

#. Import the prescribedRotation1DOF class::

    from Basilisk.simulation import prescribedRotation1DOF

#. Create an instantiation of the module::

    prescribedRot1DOF = prescribedRotation1DOF.PrescribedRotation1DOF()

#. Define all of the configuration data associated with the module. For example, to configure the smoothed bang-coast-bang option::

    prescribedRot1DOF.ModelTag = "prescribedRotation1DOF"
    prescribedRot1DOF.setRotHat_M(np.array([0.0, 1.0, 0.0]))
    prescribedRot1DOF.setThetaDDotMax(macros.D2R * 1.0)  # [rad/s^2]
    prescribedRot1DOF.setThetaInit(macros.D2R * 10.0)  # [rad]
    prescribedRot1DOF.setCoastOptionBangDuration(3.0)  # [s]
    prescribedRot1DOF.setSmoothingDuration(1.0)  # [s]

#. Connect a :ref:`HingedRigidBodyMsgPayload` message for the spinning body reference angle to the module. For example, the user can create a stand-alone message to specify the reference angle::

    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    hingedRigidBodyMessageData.theta = macros.D2R * 90.0  # [rad]
    hingedRigidBodyMessageData.thetaDot = 0.0  # [rad/s]
    hingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(hingedRigidBodyMessageData)

#. Subscribe the spinning body reference message to the prescribedRotation1DOF module input message::

    prescribedRot1DOF.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage)

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, prescribedRot1DOF)
