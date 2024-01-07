Executive Summary
-----------------
This module profiles a 1 DOF translation for a rigid body connected to a rigid spacecraft hub. The body frame
of the secondary body is designated by the frame :math:`\mathcal{F}`. The secondary body states are profiled
relative to a hub-fixed frame :math:`\mathcal{M}`. The :ref:`PrescribedMotionMsgPayload` message
is used to output the prescribed states from the module. The prescribed states are: ``r_FM_M``, ``rPrime_FM_M``,
``rPrimePrime_FM_M``, ``omega_FM_F``, ``omegaPrime_FM_F``, and ``sigma_FM``. Because this module only profiles
a translation, the secondary body rotational states ``omega_FM_F``, ``omegaPrime_FM_F``, and ``sigma_FM`` are fixed
in this module.

.. important::
    Note that this module assumes the initial and final secondary body translational rates are zero.

The general inputs to this module that must be set by the user are the secondary body rotational states ``omega_FM_F``,
``omegaPrime_FM_F``, and ``sigma_FM``, the secondary body translational axis expressed in Mount frame components
``transAxis_M``, the initial scalar position of the secondary body relative to the Mount frame ``transPosInit``,
the reference position relative to the Mount frame ``transPosRef``, the maximum translational acceleration for the
translation ``transAccelMax``, and a boolean toggle variable ``coastOption`` for selecting which of two profiling
options is desired. The first profiling option applies a bang-bang acceleration profile to the secondary body that
results in the fastest possible translation from a rest to rest state. This option is selected when ``coastOption`` is
set to ``False``. The second profiling option includes a coast segment between the two constant acceleration profiles
and is toggled with ``coastOption`` set to ``True``. To use the coast option, the user is required to specify the
additional module variable ``tRamp``. Defaulted as zero for the option with no coast period, this variable
specifies how long each acceleration segment is applied before and after the coast segment. If the user does not set
this variable, the module reverts to the profiler with no coast period.

.. important::
    To use this module for prescribed motion, it must be connected to the :ref:`PrescribedMotionStateEffector`
    dynamics module. This ensures the secondary body states are correctly incorporated into the spacecraft hub dynamics.

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
    * - prescribedTransInMsg
      - :ref:`PrescribedTransMsgPayload`
      - input msg with the secondary body reference states
    * - prescribedMotionOutMsg
      - :ref:`PrescribedMotionMsgPayload`
      - output message with the prescribed secondary body states

Detailed Module Description
---------------------------

Profiler With No Coast Period
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The first option to profile the secondary body translation is a pure bang-bang acceleration profile. If the secondary
body reference position is greater than the given initial position, the user-specified maximum acceleration value
is applied positively to the first half of the translation and negatively to the second half of the translation.
However, if the reference position is less than the initial secondary body position, the acceleration is instead applied
negatively during the first half of the translation and positively during the second half of the translation. As a 
result of this acceleration profile, the secondary body's velocity changes linearly with time and reaches a maximum
in magnitude halfway through the translation. Note that the velocity is assumed to both start and end at zero
in this module. The resulting secondary body position is parabolic in time.

To profile this motion, the scalar secondary body states :math:`l`, :math:`\dot{l}`, and
:math:`\ddot{l}` are prescribed as a function of time. During the first half of the translation the states are:

.. math::
    \ddot{l}(t) = \pm \ddot{l}_{\text{max}}

.. math::
    \dot{l}(t) = \ddot{l} (t - t_0) + \dot{l}_0

.. math::
    l(t) = a (t - t_0)^2 + l_0

where

.. math::
    a = \frac{ l_{\text{ref}} - l_0}{2 (t_s - t_0)^2}

During the second half of the translation the states are:

.. math::
    \ddot{l}(t) = \mp \ddot{l}_{\text{max}}

.. math::
    \dot{l}(t) = \ddot{l} (t - t_f) + \dot{l}_0

.. math::
    l(t) = b (t - t_f)^2 + l_{\text{ref}}

where

.. math::
    b = - \frac{ l_{\text{ref}} - l_0}{2 (t_s - t_f)^2}

The switch time :math:`t_s` is the simulation time halfway through the translation:

.. math::
    t_s = t_0 + \frac{\Delta t_{\text{tot}}}{2}

The total time required to complete the translation :math:`\Delta t_{\text{tot}}` is:

.. math::
    \Delta t_{\text{tot}} = 2 \sqrt{ \frac{| l_{\text{ref}} - l_0 | }{\ddot{l}_{\text{max}}}} = t_f - t_0

Profiler With Coast Period
^^^^^^^^^^^^^^^^^^^^^^^^^^

The second option to profile the secondary body translation is a bang-bang acceleration profile with an added coast
period between the acceleration segments where the acceleration is zero. Similarly to the previous profiler, if the
secondary body reference position is greater than the given initial position, the maximum translational acceleration
value is applied positively for the specified ramp time ``tRamp`` to the first segment of the translation and
negatively to the third segment of the translation. The second segment of the translation is the coast period. However,
if the reference position is less than the initial secondary body position, the acceleration is instead applied
negatively during the first segment of the translation and positively during the third segment of the translation.
As a result of this acceleration profile, the secondary body's velocity changes linearly with time and reaches a
maximum in magnitude at the end of the first segment and is constant during the coast segment. The velocity decreases
back to zero during the third segment. The resulting secondary body position is parabolic during the first and third
segments and linear during the coast segment.

To profile this motion, the scalar secondary body states :math:`l`, :math:`\dot{l}`, and
:math:`\ddot{l}` are prescribed as a function of time. During the first segment of the translation the states are:

.. math::
    \ddot{l}(t) = \pm \ddot{l}_{\text{max}}

.. math::
    \dot{l}(t) = \ddot{l} (t - t_0) + \dot{l}_0

.. math::
    l(t) = a (t - t_0)^2 + l_0

where

.. math::
    a = \frac{ l(t_r) - l_0}{2 (t_r - t_0)^2}

and :math:`l(t_r)` is the secondary body position at the end of the first segment:

.. math::
    l(t_r) = \pm \frac{1}{2} \ddot{l}_{\text{max}} t_{\text{ramp}}^2
                                       + \dot{l}_0 t_{\text{ramp}} + l_0

.. important::
    Note the distinction between :math:`t_r` and :math:`t_{\text{ramp}}`. :math:`t_{\text{ramp}}` is the time duration of the acceleration segment
    and :math:`t_r` is the simulation time at the end of the first acceleration segment.
    :math:`t_r = t_0 + t_{\text{ramp}}`

During the coast segment, the translation states are:

.. math::
    \ddot{l}(t) = 0

.. math::
    \dot{l}(t) = \dot{l}(t_r) = \ddot{l}_{\text{max}} t_{\text{ramp}} + \dot{l}_0

.. math::
    l(t) = \dot{l}(t_r) (t - t_r) + l(t_r)

During the third segment, the translational states are

.. math::
    \ddot{l}(t) = \mp \ddot{l}_{\text{max}}

.. math::
    \dot{l}(t) = \ddot{l} (t - t_f) + \dot{l}_0

.. math::
    l(t) = b (t - t_f)^2 + l_{\text{ref}}

where

.. math::
    b = - \frac{ l_{\text{ref}} - l(t_c) }{(t_c - t_f)^2}

Here :math:`l(t_c)` is the secondary body position at the end of the coast segment:

.. math::
    l(t_c) = l(t_r) + \Delta l_{\text{coast}}

and :math:`\Delta l_{\text{coast}}` is the distance traveled during the coast segment:

.. math::
    \Delta l_{\text{coast}} = (l_{\text{ref}} - l_0) - 2 (l(t_r) - l_0)

:math:`t_c` is the simulation time at the end of the coast segment:

.. math::
    t_c = t_r + \frac{\Delta l_{\text{coast}}}{\dot{l}(t_r)}

Using the given translation axis ``transAxis_M``, the scalar states are then transformed to the secondary body
translational states ``r_FM_M``, ``rPrime_FM_M``, and ``rPrimePrime_FM_M``. The states are then written to the
:ref:`PrescribedMotionMsgPayload` module output message.

Module Testing
^^^^^^^^^^^^^^
The unit test for this module ensures that a profiled 1 DOF translation for a secondary rigid body connected to a
spacecraft hub is properly computed for several different simulation configurations. The unit test profiles two
successive translations to ensure the module is correctly configured. The body's initial scalar translational position
relative to the spacecraft hub is varied, along with the two final reference positions and the maximum translational
acceleration. The unit test also tests both methods of profiling the translation, where either a pure bang-bang
acceleration profile can be selected for the translation, or a coast option can be selected where the accelerations
are only applied for a specified ramp time and a coast segment with zero acceleration is applied between the two
acceleration periods. To validate the module, the final position at the end of each translation is checked to match
the specified reference position.

User Guide
----------
The general inputs to this module that must be set by the user are the secondary body rotational states ``omega_FM_F``,
``omegaPrime_FM_F``, and ``sigma_FM``, the secondary body translational axis expressed in Mount frame components
``transAxis_M``, the initial scalar position of the secondary body relative to the Mount frame ``transPosInit``,
the reference position relative to the Mount frame ``transPosRef``, the maximum translational acceleration for the
translation ``transAccelMax``, and the boolean toggle variable ``coastOption`` for selecting which profiling options is
desired. To use the coast option, the user sets ``coastOption`` to True and must specify the variable ``tRamp``.
This variable specifies how long each acceleration segment is applied before and after the coast segment.
If the user does not set this variable, the module reverts to the profiler with no coast period.

This section is to outline the steps needed to setup a prescribed 1 DOF translational module in python using Basilisk.

#. Import the prescribedTrans class::

    from Basilisk.fswAlgorithms import prescribedTrans

#. Create an instantiation of the module::

    PrescribedTrans = prescribedTrans.prescribedTrans()

#. Define all of the configuration data associated with the module. For example, to configure the coast option::

    PrescribedTrans.ModelTag = "PrescribedTrans"
    PrescribedTrans.coastOption = True
    PrescribedTrans.tRamp = 3.0  # [s]
    PrescribedTrans.transAxis_M = np.array([0.5, 0.0, 0.5 * np.sqrt(3)])
    PrescribedTrans.transAccelMax = 0.01  # [m/s^2]
    PrescribedTrans.omega_FM_F = np.array([0.0, 0.0, 0.0])  # [rad/s]
    PrescribedTrans.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])  # [rad/s^2]
    PrescribedTrans.sigma_FM = np.array([0.0, 0.0, 0.0])
    PrescribedTrans.transPosInit = 0.5  # [m]

#. Connect a :ref:`PrescribedTransMsgPayload` message for the secondary body reference position to the module. For example, the user can create a stand-alone message to specify the reference position::

    PrescribedTransMessageData = messaging.PrescribedTransMsgPayload()
    PrescribedTransMessageData.scalarPos = 1.0  # [m]
    PrescribedTransMessageData.scalarVel = 0.0  # [m/s]
    PrescribedTransMessage = messaging.PrescribedTransMsg().write(PrescribedTransMessageData)

#. Subscribe the secondary body reference message to the prescribedRot1DOF module input message::

    PrescribedTrans.prescribedTransInMsg.subscribeTo(PrescribedTransMessage)

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, PrescribedTrans)

