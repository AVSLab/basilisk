Executive Summary
-----------------
This module profiles linear translational motion for a rigid body connected to a rigid spacecraft hub. The body frame of
the translating body is designated by the frame :math:`\mathcal{P}`. The states of the translating body are profiled
relative to a hub-fixed frame :math:`\mathcal{M}`. The :ref:`PrescribedTranslationMsgPayload` message is used to output
the prescribed translational states from the module. The prescribed states profiled in this module are: ``r_PM_M``,
``rPrime_PM_M``, and ``rPrimePrime_PM_M``. This module has four options to profile the linear translation.
The first option is a bang-bang acceleration profile that minimizes the time required to complete the translation.
The second option is a bang-coast-bang acceleration profile that adds a coast period of zero acceleration between the
acceleration ramp segments. The third option is a smoothed bang-bang acceleration profile that uses cubic splines to
construct a continuous acceleration profile across the entire translation. The fourth option is a smoothed
bang-coast-bang acceleration profile.

The module defaults to the non-smoothed bang-bang option with no coast period. If the coast option is desired, the
user must set the module variable ``coastOptionBangDuration`` to a nonzero value. If smoothing is desired,
the module variable ``smoothingDuration`` must be set to a nonzero value.

.. important::
    Note that this module assumes the initial and final hub-relative translational rates of the translating body are zero.

The general inputs to this module that must be set by the user are the translational axis expressed as a
unit vector in mount frame components ``transHat_M``, the initial translational body position relative to the hub-fixed
mount frame ``transPosInit``, the reference position relative to the mount frame ``transPosRef``, and the maximum scalar
linear acceleration for the translation ``transAccelMax``. The optional inputs ``coastOptionBangDuration`` and
``smoothingDuration`` can be set by the user to select the specific type of profiler that is desired. If these variables
are not set by the user, the module defaults to the non-smoothed bang-bang profiler. If only the variable
``coastOptionBangDuration`` is set to a nonzero value, the bang-coast-bang profiler is selected. If only the variable
``smoothingDuration`` is set to a nonzero value, the smoothed bang-bang profiler is selected. If both variables are
set to nonzero values, the smoothed bang-coast-bang profiler is selected.

.. important::
    To use this module for prescribed motion, it must be connected to the :ref:`PrescribedMotionStateEffector`
    dynamics module. This ensures the translational body's states are correctly incorporated into the spacecraft
    dynamics.


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
    * - linearTranslationRigidBodyInMsg
      - :ref:`LinearTranslationRigidBodyMsgPayload`
      - input msg with the prescribed body reference translational states
    * - prescribedTranslationOutMsg
      - :ref:`PrescribedTranslationMsgPayload`
      - output message with the prescribed body translational states
    * - prescribedTranslationOutMsgC
      - :ref:`PrescribedTranslationMsgPayload`
      - C-wrapped output message with the prescribed body translational states

Detailed Module Description
---------------------------

Non-Smoothed Bang-Bang Profiler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The first option to profile the linear translation is a pure bang-bang acceleration profile. If the given reference
position is greater than the given initial position, the user-specified maximum acceleration value
is applied positively to the first half of the translation and negatively to the second half of the translation.
However, if the reference position is less than the initial position, the acceleration is instead applied
negatively during the first half of the translation and positively during the second half of the translation. As a
result of this acceleration profile, the translational body's hub-relative velocity changes linearly with time and
reaches a maximum in magnitude halfway through the translation. Note that the velocity is assumed to both start and
end at zero in this module. The resulting translational position profile is parabolic in time.

To profile this motion, the translational body's hub-relative scalar states :math:`\rho`, :math:`\dot{\rho}`, and
:math:`\ddot{\rho}` are prescribed as a function of time. During the first half of the translation the states are:

.. math::
    \ddot{\rho}(t) = \pm \ddot{\rho}_{\text{max}}

.. math::
    \dot{\rho}(t) = \ddot{\rho} (t - t_0) + \dot{\rho}_0

.. math::
    \rho(t) = a (t - t_0)^2 + \rho_0

where

.. math::
    a = \frac{ \rho_{\text{ref}} - \rho_0}{2 (t_{b1} - t_0)^2}

During the second half of the translation the states are:

.. math::
    \ddot{\rho}(t) = \mp \ddot{\rho}_{\text{max}}

.. math::
    \dot{\rho}(t) = \ddot{\rho} (t - t_f) + \dot{\rho}_0

.. math::
    \rho(t) = b (t - t_f)^2 + \rho_{\text{ref}}

where

.. math::
    b = - \frac{ \rho_{\text{ref}} - \rho_0}{2 (t_{b1} - t_f)^2}

The switch time :math:`t_{b1}` is the simulation time at the end of the first bang segment:

.. math::
    t_{b1} = t_0 + \frac{\Delta t_{\text{tot}}}{2}

The total time required to complete the translation :math:`\Delta t_{\text{tot}}` is:

.. math::
    \Delta t_{\text{tot}} = 2 \sqrt{ \frac{| \rho_{\text{ref}} - \rho_0 | }{\ddot{\rho}_{\text{max}}}} = t_f - t_0

Non-Smoothed Bang-Coast-Bang Profiler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The second option to profile the linear translation is a bang-coast-bang acceleration profile with an added coast
period between the acceleration segments where the acceleration is zero. Similar to the previous profiler, if the
reference position is greater than the given initial position, the maximum acceleration value is applied
positively for the specified ramp time ``coastOptionBangDuration`` to the first segment of the translation and negatively
to the third segment of the translation. The second segment of the translation is the coast period. However, if the
reference position is less than the initial position, the acceleration is instead applied negatively during the first
segment of the translation and positively during the third segment of the translation. As a result of this acceleration
profile, the translational body's hub-relative velocity changes linearly with time and reaches a maximum in magnitude
at the end of the first segment and is constant during the coast segment. The velocity returns to zero during the
third segment. The resulting position profiled is parabolic during the first and third segments and linear during the
coast segment.

To profile this linear motion, the scalar translating body's hub-relative states :math:`\rho`, :math:`\dot{\rho}`, and
:math:`\ddot{\rho}` are prescribed as a function of time. During the first segment of the translation the states are:

.. math::
    \ddot{\rho}(t) = \pm \ddot{\rho}_{\text{max}}

.. math::
    \dot{\rho}(t) = \ddot{\rho} (t - t_0) + \dot{\rho}_0

.. math::
    \rho(t) = a (t - t_0)^2 + \rho_0

where

.. math::
    a = \frac{ \rho(t_{b1}) - \rho_0}{2 (t_{b1} - t_0)^2}

and :math:`\rho(t_{b1})` is the hub-relative position at the end of the first bang segment:

.. math::
    \rho(t_{b1}) = \pm \frac{1}{2} \ddot{\rho}_{\text{max}} t_{\text{bang}}^2 + \dot{\rho}_0 t_{\text{bang}} + \rho_0

.. important::
    Note the distinction between :math:`t_{b1}` and :math:`t_{\text{bang}}`. :math:`t_{\text{bang}}` is the time
    duration of the acceleration segment and :math:`t_{b1}` is the simulation time at the end of the first acceleration
    segment. :math:`t_{b1} = t_0 + t_{\text{bang}}`

During the coast segment, the translational states are:

.. math::
    \ddot{\rho}(t) = 0

.. math::
    \dot{\rho}(t) = \dot{\rho}(t_{b1}) = \ddot{\rho}_{\text{max}} t_{\text{bang}} + \dot{\rho}_0

.. math::
    \rho(t) = \dot{\rho}(t_{b1}) (t - t_{b1}) + \rho(t_{b1})

During the third segment, the translational states are

.. math::
    \ddot{\rho}(t) = \mp \ddot{\rho}_{\text{max}}

.. math::
    \dot{\rho}(t) = \ddot{\rho} (t - t_f) + \dot{\rho}_0

.. math::
    \rho(t) = b (t - t_f)^2 + \rho_{\text{ref}}

where

.. math::
    b = - \frac{ \rho_{\text{ref}} - \rho(t_c) }{(t_c - t_f)^2}

Here :math:`\rho(t_c)` is the hub-relative position at the end of the coast segment:

.. math::
    \rho(t_c) = \rho(t_{b1}) + \Delta \rho_{\text{coast}}

and :math:`\Delta \rho_{\text{coast}}` is the distance traveled during the coast segment:

.. math::
    \Delta \rho_{\text{coast}} = (\rho_{\text{ref}} - \rho_0) - 2 (\rho(t_{b1}) - \rho_0)

:math:`t_c` is the simulation time at the end of the coast segment:

.. math::
    t_c = t_{b1} + \frac{\Delta \rho_{\text{coast}}}{\dot{\rho}(t_{b1})}

Using the given translation axis ``transHat_M``, the scalar states are then transformed to the prescribed translational
states ``r_PM_M``, ``rPrime_PM_M``, and ``rPrimePrime_PM_M``. The states are then written to the
:ref:`PrescribedTranslationMsgPayload` module output message.

Smoothed Bang-Bang Profiler
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The third option to profile the linear translation is a smoothed bang-bang acceleration profile. This option is selected
by setting the module variable ``smoothingDuration`` to a nonzero value. This profiler uses cubic splines to construct
a continuous acceleration profiler across the entire translation. Similar to the non-smoothed bang-bang profiler,
this option smooths the acceleration between the given maximum acceleration values.
To profile this motion, the translational body's hub-relative scalar states :math:`\rho`, :math:`\dot{\rho}`, and
:math:`\ddot{\rho}` are prescribed as a function of time and the translational motion is split into five different
segments.

The first segment smooths the acceleration from zero to the user-specified maximum acceleration value in the given
time ``smoothingDuration``. If the given reference position is greater than the given initial position, the
acceleration is smoothed positively to the given maximum acceleration value. If the given reference position is less
than the given initial position, the acceleration is smoothed from zero to the negative maximum acceleration value.
During this phase, the scalar hub-relative states are:

.. math::
    \ddot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} \left( \frac{3 (t - t_0)^2}{t_{\text{smooth}}^2} - \frac{2 (t - t_0)^3}{t_{\text{smooth}}^3} \right)

.. math::
    \dot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} \left( \frac{(t - t_0)^3}{t_{\text{smooth}}^2} - \frac{(t - t_0)^4}{2 t_{\text{smooth}}^3} \right)

.. math::
    \rho(t) = \pm \ddot{\rho}_{\text{max}} \left( \frac{(t - t_0)^4}{4 t_{\text{smooth}}^2} - \frac{(t - t_0)^5}{10 t_{\text{smooth}}^3} \right)

The second segment is the first bang segment where the maximum acceleration value is applied either positively or
negatively as discussed previously. The scalar hub-relative states during this phase are:

.. math::
    \ddot{\rho}(t) = \pm \ddot{\rho}_{\text{max}}

.. math::
    \dot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} (t - t_{s1}) + \dot{\rho}(t_{s1})

.. math::
    \rho(t) = \pm \frac{\ddot{\rho}_{\text{max}} (t - t_{s1})^2}{2} + \dot{\rho}(t_{s1})(t - t_{s1}) + \rho(t_{s1})

where :math:`t_{s1}` is the time at the end of the first smoothing segment:

.. math::
    t_{s1} = t_0 + t_{\text{smooth}}

The third segment smooths the acceleration from the current maximum acceleration value to the opposite magnitude maximum
acceleration value. The scalar hub-relative states during this phase are:

.. math::
    \ddot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} \left( 1 - \frac{3 (t - t_{b1})^2}{2 t_{\text{smooth}}^2} + \frac{(t - t_{b1})^3}{2 t_{\text{smooth}}^3} \right)

.. math::
    \dot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} \left( (t - t_{b1}) - \frac{(t - t_{b1})^3}{2 t_{\text{smooth}}^2} + \frac{(t - t_{b1})^4}{8 t_{\text{smooth}}^3} \right) + \dot{\rho}(t_{b1})

.. math::
    \rho(t) = \pm \ddot{\rho}_{\text{max}} \left( \frac{(t - t_{b1})^2}{2} - \frac{(t - t_{b1})^4}{8 t_{\text{smooth}}^2} + \frac{(t - t_{b1})^5}{40 t_{\text{smooth}}^3} \right) + \dot{\rho}(t_{b1})(t - t_{b1}) + \rho(t_{b1})

where :math:`t_{b1}` is the time at the end of the first bang segment:

.. math::
    t_{b1} = t_{s1} + t_{\text{bang}}

The fourth segment is the second bang segment where the maximum acceleration value is applied either positively or
negatively as discussed previously. The scalar hub-relative states during this phase are:

.. math::
    \ddot{\rho}(t) = \mp \ddot{\rho}_{\text{max}}

.. math::
    \dot{\rho}(t) = \mp \ddot{\rho}_{\text{max}} (t - t_{s2}) + \dot{\rho}(t_{s2})

.. math::
    \rho(t) = \mp \frac{\ddot{\rho}_{\text{max}} (t - t_{s2})^2}{2} + \dot{\rho}(t_{s2})(t - t_{s2}) + \rho(t_{s2})

where :math:`t_{s2}` is the time at the end of the second smoothing segment:

.. math::
    t_{s2} = t_{b1} + t_{\text{smooth}}

The fifth segment is the third and final smoothing segment where the acceleration returns to zero. The scalar
hub-relative states during this phase are:

.. math::
    \ddot{\rho}(t) = \mp \ddot{\rho}_{\text{max}} \left ( -1 + \frac{3(t - t_{b2})^2}{t_{\text{smooth}}^2} - \frac{2 (t - t_{b2})^3}{t_{\text{smooth}}^3} \right )

.. math::
    \dot{\rho}(t) = \mp \ddot{\rho}_{\text{max}} \left ( -(t - t_{b2}) + \frac{(t - t_{b2})^3}{t_{\text{smooth}}^2} - \frac{(t - t_{b2})^4}{2 t_{\text{smooth}}^3} \right ) + \dot{\rho}(t_{b2})

.. math::
    \rho(t) = \mp \ddot{\rho}_{\text{max}} \left ( \frac{(t - t_{b2})^2}{2} + \frac{(t - t_{b2})^4}{4 t_{\text{smooth}}^2} - \frac{(t - t_{b2})^5}{10 t_{\text{smooth}}^3} \right ) + \dot{\rho}(t_{b2})(t - t_{b2}) + \rho(t_{b2})

where :math:`t_{b2}` is the time at the end of the second bang segment:

.. math::
    t_{b2} = t_{s2} + t_{\text{bang}}

Smoothed Bang-Coast-Bang Profiler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The fourth option to profile the linear translation is a smoothed bang-coast-bang acceleration profile. This option is
selected by setting the module variables ``coastOptionBangDuration`` and ``smoothingDuration`` to nonzero values.
This profiler uses cubic splines to construct a continuous acceleration profiler across the entire translation.
To profile this motion, the translational body's hub-relative scalar states :math:`\rho`, :math:`\dot{\rho}`, and
:math:`\ddot{\rho}` are prescribed as a function of time and the translational motion is split into seven different
segments.

The first segment smooths the acceleration from zero to the user-specified maximum acceleration value in the given
time ``smoothingDuration``. If the given reference position is greater than the given initial position, the
acceleration is smoothed positively to the given maximum acceleration value. If the given reference position is less
than the given initial position, the acceleration is smoothed from zero to the negative maximum acceleration value.
During this phase, the scalar hub-relative states are:

.. math::
    \ddot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} \left( \frac{3 (t - t_0)^2}{t_{\text{smooth}}^2} - \frac{2 (t - t_0)^3}{t_{\text{smooth}}^3} \right)

.. math::
    \dot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} \left( \frac{(t - t_0)^3}{t_{\text{smooth}}^2} - \frac{(t - t_0)^4}{2 t_{\text{smooth}}^3} \right)

.. math::
    \rho(t) = \pm \ddot{\rho}_{\text{max}} \left( \frac{(t - t_0)^4}{4 t_{\text{smooth}}^2} - \frac{(t - t_0)^5}{10 t_{\text{smooth}}^3} \right)

The second segment is the first bang segment where the maximum acceleration value is applied either positively or
negatively as discussed previously. The scalar hub-relative states during this phase are:

.. math::
    \ddot{\rho}(t) = \pm \ddot{\rho}_{\text{max}}

.. math::
    \dot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} (t - t_{s1}) + \dot{\rho}(t_{s1})

.. math::
    \rho(t) = \pm \frac{\ddot{\rho}_{\text{max}} (t - t_{s1})^2}{2} + \dot{\rho}(t_{s1})(t - t_{s1}) + \rho(t_{s1})

where :math:`t_{s1}` is the time at the end of the first smoothing segment.

The third segment prior to the coast phase smooths the acceleration from the current maximum acceleration value to zero.
The scalar hub-relative states during this phase are:

.. math::
    \ddot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} \left( 1 - \frac{3 (t - t_{b1})^2}{t_{\text{smooth}}^2} - \frac{2 (t - t_{b1})^3}{t_{\text{smooth}}^3} \right)

.. math::
    \dot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} \left( (t - t_{b1}) - \frac{(t - t_{b1})^3}{t_{\text{smooth}}^2} - \frac{(t - t_{b1})^4}{2 t_{\text{smooth}}^3} \right) + \dot{\rho}(t_{b1})

.. math::
    \rho(t) = \pm \ddot{\rho}_{\text{max}} \left( \frac{(t - t_{b1})^2}{2} - \frac{(t - t_{b1})^4}{4 t_{\text{smooth}}^2} - \frac{(t - t_{b1})^5}{10 t_{\text{smooth}}^3} \right) + \dot{\rho}(t_{b1})(t - t_{b1}) + \rho(t_{b1})

where :math:`t_{b1}` is the time at the end of the first bang segment.

The fourth segment is the coast segment where the translational states are:

.. math::
    \ddot{\rho}(t) = 0

.. math::
    \dot{\rho}(t) = \dot{\rho}(t_{s2})

.. math::
    \rho(t) = \dot{\rho}(t_{s2}) (t - t_{s2}) + \rho(t_{s2})

where :math:`t_{s2}` is the time at the end of the second smoothing segment.

The fifth segment smooths the acceleration from zero to the maximum acceleration value prior to the second bang segment.
The translational states during this phase are:

.. math::
    \ddot{\rho}(t) = \mp \ddot{\rho}_{\text{max}} \left( \frac{3 (t - t_c)^2}{t_{\text{smooth}}^2} - \frac{2 (t - t_c)^3}{t_{\text{smooth}}^3} \right)

.. math::
    \dot{\rho}(t) = \mp \ddot{\rho}_{\text{max}} \left( \frac{(t - t_c)^3}{t_{\text{smooth}}^2} - \frac{(t - t_c)^4}{2 t_{\text{smooth}}^3} \right) + \dot{\rho}(t_c)

.. math::
    \rho(t) = \mp \ddot{\rho}_{\text{max}} \left( \frac{(t - t_c)^4}{4 t_{\text{smooth}}^2} - \frac{(t - t_c)^5}{10 t_{\text{smooth}}^3} \right) + \dot{\rho}(t_c) (t - t_c) + \rho(t_c)

where :math:`t_c` is the time at the end of the coast segment.

The sixth segment is the second bang segment where the maximum acceleration value is applied either positively or
negatively as discussed previously. The scalar hub-relative states during this phase are:

.. math::
    \ddot{\rho}(t) = \mp \ddot{\rho}_{\text{max}}

.. math::
    \dot{\rho}(t) = \mp \ddot{\rho}_{\text{max}} (t - t_{s3}) + \dot{\rho}(t_{s3})

.. math::
    \rho(t) = \mp \frac{\ddot{\rho}_{\text{max}} (t - t_{s3})^2}{2} + \dot{\rho}(t_{s3})(t - t_{s3}) + \rho(t_{s3})

where :math:`t_{s3}` is the time at the end of the third smoothing segment.

The seventh segment is the fourth and final smoothing segment where the acceleration returns to zero. The scalar
hub-relative states during this phase are:

.. math::
    \ddot{\rho}(t) = \mp \ddot{\rho}_{\text{max}} \left (\frac{3(t_f - t)^2}{t_{\text{smooth}}^2} - \frac{2 (t_f - t)^3}{t_{\text{smooth}}^3} \right )

.. math::
    \dot{\rho}(t) = \pm \ddot{\rho}_{\text{max}} \left (\frac{(t_f - t)^3}{t_{\text{smooth}}^2} - \frac{(t_f - t)^4}{2 t_{\text{smooth}}^3} \right )

.. math::
    \rho(t) = \mp \ddot{\rho}_{\text{max}} \left (\frac{(t_f - t)^4}{4 t_{\text{smooth}}^2} - \frac{(t_f - t)^5}{10 t_{\text{smooth}}^3} \right ) + \rho_{\text{ref}}

where :math:`t_f` is the time at the end of the translation:

Module Testing
^^^^^^^^^^^^^^
The unit test for this module ensures that the profiled linear translation for a secondary rigid body relative to
the spacecraft hub is properly computed for several different simulation configurations. The unit test profiles
two successive translations to ensure the module is correctly configured. The secondary body's initial scalar
translational position relative to the spacecraft hub is varied, along with the two final reference positions
and the maximum translational acceleration.

The unit test also tests four different methods of profiling the translation. Two profilers prescribe a pure
bang-bang or bang-coast-bang linear acceleration profile for the translation. The bang-bang option results in
the fastest possible translation; while the bang-coast-bang option includes a coast period with zero acceleration
between the acceleration segments. The other two profilers apply smoothing to the bang-bang and bang-coast-bang
acceleration profiles so that the secondary body hub-relative rates start and end at zero.

To verify the module functionality, the final position at the end of each translation segment is checked to match
the specified reference positions. Additionally, for the smoothed profiler options, the numerical derivative of the
profiled displacements and velocities is determined across the entire simulation. These numerical derivatives are
checked with the module's acceleration and velocity profiles to ensure the profiled acceleration is correctly
integrated in the module to obtain the displacements and velocities.

User Guide
----------
The general inputs to this module that must be set by the user are the translational axis expressed as a
unit vector in mount frame components ``transHat_M``, the initial translational body position relative to the hub-fixed
mount frame ``transPosInit``, the reference position relative to the mount frame ``transPosRef``, and the maximum scalar
linear acceleration for the translation ``transAccelMax``. The optional inputs ``coastOptionBangDuration`` and
``smoothingDuration`` can be set by the user to select the specific type of profiler that is desired. If these variables
are not set by the user, the module defaults to the non-smoothed bang-bang profiler. If only the variable
``coastOptionBangDuration`` is set to a nonzero value, the bang-coast-bang profiler is selected. If only the variable
``smoothingDuration`` is set to a nonzero value, the smoothed bang-bang profiler is selected. If both variables are
set to nonzero values, the smoothed bang-coast-bang profiler is selected.

This section is to outline the steps needed to setup the prescribed linear translational module in python using Basilisk.

#. Import the prescribedLinearTranslation class::

    from Basilisk.simulation import prescribedLinearTranslation

#. Create an instantiation of the module::

    prescribedLinearTrans = prescribedLinearTranslation.PrescribedLinearTranslation()

#. Define all of the configuration data associated with the module. For example, to configure the smoothed bang-coast-bang option::

    prescribedLinearTrans.ModelTag = "prescribedLinearTranslation"
    prescribedLinearTrans.setTransHat_M(np.array([0.5, 0.0, 0.5 * np.sqrt(3)]))
    prescribedLinearTrans.setTransAccelMax(0.01)  # [m/s^2]
    prescribedLinearTrans.setTransPosInit(0.5)  # [m]
    prescribedLinearTrans.setCoastRampDuration(1.0)  # [s]
    prescribedLinearTrans.setSmoothingDuration(1.0)  # [s]

#. Connect a :ref:`LinearTranslationRigidBodyMsgPayload` message for the translating body reference position to the module. For example, the user can create a stand-alone message to specify the reference position::

    linearTranslationRigidBodyMessageData = messaging.LinearTranslationRigidBodyMsgPayload()
    linearTranslationRigidBodyMessageData.rho = 1.0  # [m]
    linearTranslationRigidBodyMessageData.rhoDot = 0.0  # [m/s]
    linearTranslationRigidBodyMessage = messaging.LinearTranslationRigidBodyMsg().write(linearTranslationRigidBodyMessageData)

#. Subscribe the reference message to the prescribedTranslation module input message::

    prescribedLinearTrans.linearTranslationRigidBodyInMsg.subscribeTo(linearTranslationRigidBodyMessage)

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, prescribedLinearTrans)
