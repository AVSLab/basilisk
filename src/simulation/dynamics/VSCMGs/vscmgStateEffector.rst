
Executive Summary
-----------------

This dynamic effector class implements a variable speed control moment gyroscope or VSCMG device.

The module
:download:`PDF Description </../../src/simulation/dynamics/VSCMGs/_Documentation/Basilisk-VSCMGSTATEEFFECTOR-20180718.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


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
    * - cmdsInMsg
      - :ref:`VSCMGArrayTorqueMsgPayload`
      - motor torque command input message
    * - speedOutMsg
      - :ref:`VSCMGSpeedMsgPayload`
      - VSCMG speed output message
    * - vscmgOutMsgs
      - :ref:`VSCMGConfigMsgPayload`
      - vector of VSCMG output messages

User Guide
-----------
This section is to outline the steps needed to setup a VSCMG State Effector in Python using Basilisk.

#. Import the vscmgStateEffector class::

    from basilisk.simulation import vscmgStateEffector

#. create a default VSCMG function::

    def defaultVSCMG():
      VSCMG = messaging.VSCMGConfigMsgPayload()
      VSCMG.rGB_B = [[0.],[0.],[0.]]
      VSCMG.gsHat0_B = [[0.],[0.],[0.]]
      VSCMG.gtHat0_B = [[0.],[0.],[0.]]
      VSCMG.ggHat_B = [[0.],[0.],[0.]]
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
