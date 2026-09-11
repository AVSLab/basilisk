
Executive Summary
-----------------

This state effector class implements a variable speed control moment gyroscope or VSCMG device.

The module
:download:`PDF Description </../../src/simulation/dynamics/VSCMGs/_Documentation/Basilisk-VSCMGSTATEEFFECTOR-20180718.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: vscmgStateEffector
    :caption: Module I/O Messages

    input cmdsInMsg VSCMGArrayTorqueMsgPayload
        Motor torque command input message; must be linked and written when the effector is scheduled.
    output speedOutMsg VSCMGSpeedMsgPayload
        VSCMG speed output message.
    output vscmgOutMsgs VSCMGConfigMsgPayload
        vector of VSCMG output messages.

Initialization and Reset
------------------------

Attach the effector with ``spacecraft.addStateEffector()``. Spacecraft initialization validates the device
configuration and derives the mass totals, mass fractions, reference axes, combined inertias, and model-dependent
imbalance parameters before registering the integrated states. This works even when the effector is not added
to a simulation task. See :ref:`effectorInitialization` for the general initialization contract.

Configure at most `MAX_EFF_CNT
<https://github.com/AVSLab/basilisk/blob/develop/src/architecture/utilities/macroDefinitions.h>`__ devices before
initializing the simulation. Larger arrays raise ``BasiliskError`` during configuration validation because the
command and speed messages have fixed capacity.

The initial axes ``gsHat0_B``, ``gtHat0_B``, and ``ggHat_B`` must be finite and nonzero. Each is normalized,
then the resulting frame is checked for orthogonality and right-handedness. Parallel axes or a left-handed frame
raise ``BasiliskError``. The wheel and gimbal masses must be finite and nonnegative. Fully coupled jitter also
requires ``massW > 0`` because the center-of-mass offset is derived as ``U_s / massW``. Balanced and simple
jitter models continue to support zero wheel and gimbal mass; when their total mass is zero, the unused mass
fractions are set to zero.

The model selector, initial angles and rates, diagonal inertias, and location must be valid and finite. Diagonal
inertias must be nonnegative. Balanced and simple jitter require positive ``IW1`` and ``IW3 + IG3``, which
appear as divisors in their equations. Jitter imbalance parameters must be finite; the fully coupled model also
checks its off-diagonal gimbal inertias and additional geometry for finite values. Its spin inertia divisor
``eOmega = IW1 + massW*d*d`` is checked during configuration initialization. The coupled equations also check
``egamma``, ``eOmega``, and ``1 - cOmega*cgamma`` immediately before division on every dynamics evaluation,
including spacecraft initialization. Zero or non-finite divisors, and divisors whose reciprocal overflows,
raise ``BasiliskError``. These checks do not certify physical validity of the full inertia tensors or guarantee
numerical accuracy near a singular configuration.

Configured motor torques, friction magnitudes, torque limits, speed limits, and friction smoothing ratios must
be finite. Negative limits and smoothing ratios retain their documented disabling behavior. When linear friction
is enabled, the corresponding speed limit multiplied by its smoothing ratio must be finite and positive;
the scheduled motor processing checks this before division. Wheel and gimbal friction use their respective
smoothing ratios. Non-finite incoming torque commands raise ``BasiliskError`` before motor processing;
non-finite computed motor torques are rejected before they are applied to the dynamics.

Add the effector to a task when command processing and the speed/configuration output messages are needed.
Its scheduled ``UpdateState()`` requires a linked and written ``cmdsInMsg``. An attached-only effector uses the
configured ``u_s_current`` and ``u_g_current`` torques without reading new commands or applying the scheduled
friction and saturation logic; use zero initial torques for an uncommanded device.

``Reset()`` repeats configuration validation and derivation, clears the pending command and speed output buffers,
and leaves integrated states and currently applied motor torques unchanged. The next scheduled ``UpdateState()``
reads commands and computes the applied torques. State registration initializes the wheel/gimbal rates and gimbal
angle from their configured values and initializes integrated wheel jitter angles to zero. The shared configuration
helper is safe to call before or after spacecraft initialization and does not accumulate changes when both lifecycle
paths run.

User Guide
-----------
This section is to outline the steps needed to setup a VSCMG State Effector in Python using Basilisk.

#. Import the vscmgStateEffector class::

    from Basilisk.simulation import vscmgStateEffector

#. create a default VSCMG function::

    def defaultVSCMG():
      VSCMG = messaging.VSCMGConfigMsgPayload()
      VSCMG.rGB_B = [[0.],[0.],[0.]]
      VSCMG.gsHat0_B = [[1.],[0.],[0.]]  # [-]
      VSCMG.gtHat0_B = [[0.],[1.],[0.]]  # [-]
      VSCMG.ggHat_B = [[0.],[0.],[1.]]  # [-]
      VSCMG.u_s_max = -1
      VSCMG.u_s_min = -1
      VSCMG.u_s_f = 0.
      VSCMG.wheelLinearFrictionRatio = -1
      VSCMG.u_g_current = 0.
      VSCMG.u_g_max = -1
      VSCMG.u_g_min = -1
      VSCMG.u_g_f = 0.
      VSCMG.gimbalLinearFrictionRatio = -1
      VSCMG.Omega = 0.
      VSCMG.gamma = 0.
      VSCMG.gammaDot = 0.
      VSCMG.Omega_max = 6000. * macros.RPM
      VSCMG.gammaDot_max = -1
      VSCMG.IW1 = 100./VSCMG.Omega_max
      VSCMG.IW2 = 0.5*VSCMG.IW1
      VSCMG.IW3 = 0.5*VSCMG.IW1
      VSCMG.IG1 = 0.1
      VSCMG.IG2 = 0.2
      VSCMG.IG3 = 0.3
      VSCMG.U_s = 4.8e-06 * 1e4
      VSCMG.U_d = 1.54e-06 * 1e4
      VSCMG.l = 0.01
      VSCMG.L = 0.1
      VSCMG.rGcG_G = [[0.0001],[-0.02],[0.1]]
      VSCMG.massW = 6.
      VSCMG.massG = 6.
      VSCMG.VSCMGModel = 0
      return VSCMG

#. Create a list to store the VSCMGs::

    VSCMGs = []

#. Create a VSCMG and append it to the list::

    VSCMGs.append(defaultVSCMG())

#. (Optional) Adjust the VSCMG parameters::

    VSCMGs[0].gsHat0_B = [[1.0], [0.0], [0.0]]
    VSCMGs[0].gtHat0_B = [[0.0], [1.0], [0.0]]
    VSCMGs[0].ggHat_B = [[0.0], [0.0], [1.0]]
    VSCMGs[0].Omega = 2000 * macros.RPM
    VSCMGs[0].gamma = 0.
    VSCMGs[0].gammaDot = 0.06
    VSCMGs[0].rGB_B = [[0.1], [0.002], [-0.02]]

#. (Optional) Create additional VSCMGs and append them to the list::

    ang = 54.75 * np.pi/180
    VSCMGs.append(defaultVSCMG())
    VSCMGs[1].gsHat0_B = [[0.0], [1.0], [0.0]]
    VSCMGs[1].ggHat_B = [[math.cos(ang)], [0.0], [math.sin(ang)]]
    VSCMGs[1].gtHat0_B = np.cross(np.array([math.cos(ang), 0.0, math.sin(ang)]),np.array([0.0, 1.0, 0.0]))
    VSCMGs[1].Omega =  350 * macros.RPM
    VSCMGs[1].gamma = 0.
    VSCMGs[1].gammaDot = 0.011
    VSCMGs[1].rGB_B = [[0.0], [-0.05], [0.0]]

    VSCMGs.append(defaultVSCMG())
    VSCMGs[2].gsHat0_B = [[0.0], [-1.0], [0.0]]
    VSCMGs[2].ggHat_B = [[-math.cos(ang)], [0.0], [math.sin(ang)]]
    VSCMGs[2].gtHat0_B = np.cross(np.array([-math.cos(ang), 0.0, math.sin(ang)]),np.array([0.0, -1.0, 0.0]))
    VSCMGs[2].Omega = -900 * macros.RPM
    VSCMGs[2].gamma = 0.
    VSCMGs[2].gammaDot = -0.003
    VSCMGs[2].rGB_B = [[-0.1], [0.05], [0.05]]

#. Create an instantiation of the VSCMGs::
    vscmgStateEffector = vscmgStateEffector.VSCMGStateEffector()

#. Add the VSCMGs to the vscmgStateEffector::

    for item in VSCMGs:
      vscmgStateEffector.AddVSCMG(item)

#. The VSCMG angular states output message is ``speedOutMsg``.

#. The configuration of the VSCMG is created using an output vector of messages ``vscmgOutMsgs``.

#. Add the effector to your spacecraft::

    scObject.addStateEffector(vscmgStateEffector)

    See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Create the VSCMG torque command input message::

    cmdArray = messaging.VSCMGArrayTorqueMsgPayload()

#. Set the torque command for each VSCMG in the array::

    cmdArray.wheelTorque = [0.0, 0.0, 0.0]  # [Nm]
    cmdArray.gimbalTorque = [0.0, 0.0, 0.0]  # [Nm]

#. Write the command message::

    cmdMsg = messaging.VSCMGArrayTorqueMsg().write(cmdArray)

#. Subscribe the vscmgStateEffector to the command message::

    vscmgStateEffector.cmdsInMsg.subscribeTo(cmdMsg)

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, vscmgStateEffector)
