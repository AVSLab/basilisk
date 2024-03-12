Executive Summary
-----------------
This module profiles linear translational motion for a rigid body connected to a rigid spacecraft hub. The body frame of
the translating body is designated by the frame :math:`\mathcal{F}`. The states of the translating body are profiled
relative to a hub-fixed frame :math:`\mathcal{M}`. The :ref:`PrescribedTranslationMsgPayload` message is used to output
the prescribed translational states from the module. The prescribed states profiled in this module are: ``r_FM_M``,
``rPrime_FM_M``, and ``rPrimePrime_FM_M``. This module has two options to profile the linear translation.
The first option is a bang-bang acceleration profile that minimizes the time required to complete the translation.
The second option is a bang-off-bang acceleration profile that adds a coast period of zero acceleration between the
acceleration ramp segments. The module defaults to the bang-bang option with no coast period. If the coast option is
desired, the user must set the ramp time module variable ``coastOptionRampDuration`` to a nonzero value.

.. important::
    Note that this module assumes the initial and final hub-relative translational rates of the translating body are zero.

The general inputs to this module that must be set by the user are the translational axis expressed as a
unit vector in Mount frame components ``transHat_M``, the initial translational body position relative to the Mount
frame ``transPosInit``, the reference position relative to the Mount frame ``transPosRef``, the maximum scalar linear
acceleration for the translation ``transAccelMax``, and the ramp time variable ``coastOptionRampDuration`` for specifying
the time the acceleration ramp segments are applied if the coast option is desired. This variable is defaulted to zero,
meaning that the module defaults to the bang-bang acceleration profile with no coast period.

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

Profiler With No Coast Period
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
    a = \frac{ \rho_{\text{ref}} - \rho_0}{2 (t_s - t_0)^2}

During the second half of the translation the states are:

.. math::
    \ddot{\rho}(t) = \mp \ddot{\rho}_{\text{max}}

.. math::
    \dot{\rho}(t) = \ddot{\rho} (t - t_f) + \dot{\rho}_0

.. math::
    \rho(t) = b (t - t_f)^2 + \rho_{\text{ref}}

where

.. math::
    b = - \frac{ \rho_{\text{ref}} - \rho_0}{2 (t_s - t_f)^2}

The switch time :math:`t_s` is the simulation time halfway through the translation:

.. math::
    t_s = t_0 + \frac{\Delta t_{\text{tot}}}{2}

The total time required to complete the translation :math:`\Delta t_{\text{tot}}` is:

.. math::
    \Delta t_{\text{tot}} = 2 \sqrt{ \frac{| \rho_{\text{ref}} - \rho_0 | }{\ddot{\rho}_{\text{max}}}} = t_f - t_0

Profiler With Coast Period
^^^^^^^^^^^^^^^^^^^^^^^^^^

The second option to profile the linear translation is a bang-coast-bang acceleration profile with an added coast
period between the acceleration segments where the acceleration is zero. Similar to the previous profiler, if the
reference position is greater than the given initial position, the maximum acceleration value is applied
positively for the specified ramp time ``coastOptionRampDuration`` to the first segment of the translation and negatively
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
    a = \frac{ \rho(t_r) - \rho_0}{2 (t_r - t_0)^2}

and :math:`\rho(t_r)` is the hub-relative position at the end of the first segment:

.. math::
    \rho(t_r) = \pm \frac{1}{2} \ddot{\rho}_{\text{max}} t_{\text{ramp}}^2 + \dot{\rho}_0 t_{\text{ramp}} + \rho_0

.. important::
    Note the distinction between :math:`t_r` and :math:`t_{\text{ramp}}`. :math:`t_{\text{ramp}}` is the time duration of the acceleration segment
    and :math:`t_r` is the simulation time at the end of the first acceleration segment. :math:`t_r = t_0 + t_{\text{ramp}}`

During the coast segment, the translational states are:

.. math::
    \ddot{\rho}(t) = 0

.. math::
    \dot{\rho}(t) = \dot{\rho}(t_r) = \ddot{\rho}_{\text{max}} t_{\text{ramp}} + \dot{\rho}_0

.. math::
    \rho(t) = \dot{\rho}(t_r) (t - t_r) + \rho(t_r)

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
    \rho(t_c) = \rho(t_r) + \Delta \rho_{\text{coast}}

and :math:`\Delta \rho_{\text{coast}}` is the distance traveled during the coast segment:

.. math::
    \Delta \rho_{\text{coast}} = (\rho_{\text{ref}} - \rho_0) - 2 (\rho(t_r) - \rho_0)

:math:`t_c` is the simulation time at the end of the coast segment:

.. math::
    t_c = t_r + \frac{\Delta \rho_{\text{coast}}}{\dot{\rho}(t_r)}

Using the given translation axis ``transHat_M``, the scalar states are then transformed to the prescribed translational
states ``r_FM_M``, ``rPrime_FM_M``, and ``rPrimePrime_FM_M``. The states are then written to the
:ref:`PrescribedTranslationMsgPayload` module output message.

Module Testing
^^^^^^^^^^^^^^
The unit test for this module ensures that a profiled linear translation for a secondary rigid body connected to a
spacecraft hub is properly computed for several different simulation configurations. The unit test profiles two
successive translations to ensure the module is correctly configured. The body's initial scalar translational position
relative to the spacecraft hub is varied, along with the two final reference positions and the maximum translational
acceleration. The unit test also tests both methods of profiling the translation, where either a pure bang-bang
acceleration profile can be selected for the translation, or a coast option can be selected where the accelerations
are only applied for a specified ramp time and a coast segment with zero acceleration is applied between the two
acceleration periods. To validate the module, the final hub-relative position at the end of each translation is
checked to match the specified reference position.

User Guide
----------
The general inputs to this module that must be set by the user are the translational axis expressed as a
unit vector in Mount frame components ``transHat_M``, the initial translational body position relative to the Mount
frame ``transPosInit``, the reference position relative to the Mount frame ``transPosRef``, the maximum scalar linear
acceleration for the translation ``transAccelMax``, and the ramp time variable ``coastOptionRampDuration`` for specifying
the time the acceleration ramp segments are applied if the coast option is desired. This variable is defaulted to zero,
meaning that the module defaults to the bang-bang acceleration profile with no coast period.

This section is to outline the steps needed to setup the prescribed linear translational module in python using Basilisk.

#. Import the prescribedTranslation class::

    from Basilisk.simulation import prescribedTranslation

#. Create an instantiation of the module::

    prescribedLinearTrans = prescribedLinearTranslation.PrescribedLinearTranslation()

#. Define all of the configuration data associated with the module. For example, to configure the coast option::

    prescribedLinearTrans.ModelTag = "prescribedLinearTranslation"
    prescribedLinearTrans.setTransHat_M(np.array([0.5, 0.0, 0.5 * np.sqrt(3)]))
    prescribedLinearTrans.setTransAccelMax(0.01)  # [m/s^2]
    prescribedLinearTrans.setTransPosInit(0.5)  # [m]
    prescribedLinearTrans.setCoastRampDuration(1.0)  # [s]

#. Connect a :ref:`LinearTranslationRigidBodyMsgPayload` message for the translating body reference position to the module. For example, the user can create a stand-alone message to specify the reference position::

    linearTranslationRigidBodyMessageData = messaging.LinearTranslationRigidBodyMsgPayload()
    linearTranslationRigidBodyMessageData.rho = 1.0  # [m]
    linearTranslationRigidBodyMessageData.rhoDot = 0.0  # [m/s]
    linearTranslationRigidBodyMessage = messaging.LinearTranslationRigidBodyMsg().write(linearTranslationRigidBodyMessageData)

#. Subscribe the reference message to the prescribedTranslation module input message::

    prescribedLinearTrans.linearTranslationRigidBodyInMsg.subscribeTo(linearTranslationRigidBodyMessage)

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, prescribedLinearTrans)

