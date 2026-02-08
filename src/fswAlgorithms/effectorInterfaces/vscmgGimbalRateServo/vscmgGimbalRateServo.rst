Executive Summary
-----------------
The ``vscmgGimbalRateServo`` module maps a desired wheel acceleration and gimbal rate for each VSCMG to a corresponding motor torque command for both the reaction wheel and gimbal.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg connection is set by the user from python. The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - vscmgParamsInMsg
      - :ref:`VSCMGArrayConfigMsgPayload`
      - VSCMG array configuration input message
    * - vscmgRefStatesInMsg
      - :ref:`VSCMGRefStatesMsgPayload`
      - reference VSCMG states input message
    * - attInMsg
      - :ref:`AttRefMsgPayload`
      - attitude navigation input message
    * - speedsInMsg
      - :ref:`VSCMGSpeedMsgPayload`
      - VSCMG speeds input message
    * - cmdsOutMsg
      - :ref:`VSCMGArrayTorqueMsgPayload`
      - VSCMG motor torque output message


Detailed Module Description
---------------------------
General Function
^^^^^^^^^^^^^^^^
The `vscmgGimbalRateServo` module creates the the VSCMG wheel motor torque :math:`{\bf u}_{s}` and gimbal motor torque :math:`{\bf u}_g` developed in chapter 8 of `Analytical Mechanics of Space Systems <http://doi.org/10.2514/4.105210>`__.

Algorithm
^^^^^^^^^
This module employs the Acceleration-Based VSCMG Steering Law of Section (8.8.3) of `Analytical Mechanics of Space Systems <http://doi.org/10.2514/4.105210>`__.  Taking the gimbal motor torque equation

.. math:: u_{g_{i}} = J_{g_{i}}(\hat{\boldsymbol{ g}}_{g_{i}}^{T}\dot{\boldsymbol{ \omega}} + \ddot{\gamma}_{i}) -(J_{s_{i}} - J_{t_{i}})\omega_{s_{i}}\omega_{t_{i}} - I_{w_{s_{i}}}\Omega\omega_{t_{i}}
    :label: eq:gimbalTorque

where

.. math:: {\boldsymbol{ \omega}} = \omega_{s_{i}}\hat{\boldsymbol{ g}}_{s_{i}} + \omega_{t_{i}}\hat{\boldsymbol{ g}}_{t_{i}} + \omega_{g_{i}}\hat{\boldsymbol{ g}}_{g_{i}}
    :label: eq:omegaDot

and :math:`J_{s_{i}}`, :math:`J_{t_{i}}`, :math:`J_{g_{i}}` are the inertias for the ith gimbal wheel system for the first, second and third axis respectively, :math:`I_{w_{s_{i}}}` is the ith RW spin axis inertia, :math:`\gamma` is the gimbal angle, and :math:`\Omega` is the wheel speed. Considering the relative magnitude of :math:`\dot{\bf \omega}` and :math:`\ddot{\gamma}_{i}`, the gimbal torque can be approximated as

.. math:: u_{g_{i}} \approx J_{g_{i}}\ddot{\gamma}_{i} -(J_{s_{i}} - J_{t_{i}})\omega_{s_{i}}\omega_{t_{i}} - I_{w_{s_{i}}}\Omega\omega_{t_{i}}
    :label: eq:gimbalTorqueApprox

Using a desired gimbal rate :math:`\dot{\gamma}_{d}` the servo tracking error for the gimbal rate is defined as

.. math:: \Delta\dot{\gamma} = \dot{\gamma} - \dot{\gamma}_{d}
    :label: eq:gimbalRateError

Picking the Lyapunov Function

.. math:: V = \frac{1}{2}\Delta\dot{\gamma}^{2}
    :label: eq:gimbalLyapunov

and taking the time derivative and forcing it to be negative definite leads to

.. math:: \dot{V} = \Delta\dot{\gamma}\Delta\ddot{\gamma} \equiv -K_{\dot{\gamma}}\Delta\dot{\gamma}^{2}
    :label: eq:gimbalLyapunovDerivative

with gain :math:`K_{\dot{\gamma}} > 0`.  Solving for the gimbal acceleration leads to

.. math:: \ddot{\gamma} = \ddot{\gamma}_{d} - K_{\dot{\gamma}}\Delta\dot{\gamma}
    :label: eq:gimbalAcceleration

dropping the feedforward term and substituing the result in to Eq. :eq:`eq:gimbalTorqueApprox` leads to the gimbal torque command

.. math:: u_{g_{i}} = -J_{g_{i}} K_{\dot{\gamma}}\Delta\dot{\gamma} - (J_{s_{i}} - J_{t_{i}})\omega_{s_{i}}\omega_{t_{i}} - I_{w_{s_{i}}}\Omega\omega_{t_{i}}
    :label: eq:gimbalTorqueCommand

The wheel motor torque equation is given by

.. math:: u_{s_{i}} = I_{w_{s_{i}}}(\dot{\Omega}_{i} + \hat{\boldsymbol{ g}}_{s_{i}}^{T}\dot{\boldsymbol{ \omega}} + \dot{\gamma}_{i}\omega_{t_{i}})
    :label: eq:wheelTorque

comparing the magnitude of :math:`\dot{\Omega}_{i}` and :math:`\dot{\boldsymbol{ \omega}}` leads to the approximation

.. math:: u_{s_{i}} \approx I_{w_{s_{i}}}(\dot{\Omega}_{i} + \dot{\gamma}_{i}\omega_{t_{i}})
    :label: eq:wheelTorqueApprox

Setting :math:`\dot{\Omega} = \dot{\Omega}_{d}` produces the open loop control

.. math:: u_{s_{i}} = I_{w_{s_{i}}}(\dot{\Omega}_{d_{i}} + \dot{\gamma}\omega_{t_{i}})
    :label: eq:wheelTorqueCommand

This servo implementation does not track the wheel spin rate :math:`{\Omega}_{i}` and thus relies on the outer control loop of the attitude feedback control to adjust the desired wheel accelerations to yield the desired close-loop dynamics.

User Guide
----------
This section is to outline the steps needed to setup the VSCMG gimbal rate servo module in Python using Basilisk.

#. Import the vscmgGimbalRateServo class::

    from Basilisk.fswAlgorithms import vscmgGimbalRateServo

#. Create an instance of the vscmgGimbalRateServo::

    vscmgGimbalRateServo = vscmgGimbalRateServo.VscmgGimbalRateServo()

#. Set the gain for the gimbal servo::

    K_gammaDot = 1.0
    vscmgGimbalRateServo.setK_gammaDot(K_gammaDot)

#. The VSCMG torque output message is ``cmdsOutMsg``.

#. add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, vscmgGimbalRateServo)
