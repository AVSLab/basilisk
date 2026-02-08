Executive Summary
-----------------
The ``vscmgVelocitySteering`` module maps the required spacecraft control torque vector (:math:`\boldsymbol{L}_r`) to a corresponding reference wheel acceleration and gimbal rate for each VSCMG.

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
    * - vehControlInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - vehicle control (Lr) input message
    * - attNavInMsg
      - :ref:`NavAttMsgPayload`
      - attitude navigation input message
    * - attGuideInMsg
      - :ref:`AttGuidMsgPayload`
      - attitude guidance input message
    * - speedsInMsg
      - :ref:`VSCMGSpeedMsgPayload`
      - VSCMG speeds input message
    * - vscmgRefStatesOutMsg
      - :ref:`VSCMGRefStatesMsgPayload`
      - reference VSCMG states output message


Detailed Module Description
---------------------------
General Function
^^^^^^^^^^^^^^^^
the `vscmgVelocitySteering` module creates the VSCMG reference state vector :math:`\dot{\boldsymbol{\eta}}` developed in chapter 8 of `Analytical Mechanics of Space Systems <http://doi.org/10.2514/4.105210>`__.

Algorithm
^^^^^^^^^
This module employs the Velocity-Based VSCMG Steering Law of Section (8.8.2) of `Analytical Mechanics of Space Systems <http://doi.org/10.2514/4.105210>`__.  Use of the :math:`2N\times1` state vector :math:`{\boldsymbol{\eta}}`

.. math:: \boldsymbol{\eta} = \begin{bmatrix} \boldsymbol{\Omega} \\ \boldsymbol{\gamma} \end{bmatrix}
    :label: eq:vscmgEta

and :math:`3\times2N` matrix :math:`[Q]`

.. math:: [Q] = \begin{bmatrix} D_0 & D \end{bmatrix}
    :label: eq:vscmgQ

allow for the compact expression

.. math:: [Q]\dot{\boldsymbol{\eta}} = -\boldsymbol{L}_r
    :label: eq:vscmgControl

where :math:`\boldsymbol{L}_r` is the required spacecraft control torque vector, :math:`N` is the number of VSCMGs, :math:`\boldsymbol{\Omega}` is the vector of RW spin rates, and :math:`\boldsymbol{\gamma}` is the vector of gimbal angles. The matrices :math:`[D_0]` and :math:`[D]` are defined with all components in the spacecraft body frame :math:`\mathcal{B}` as

.. math:: [D_0] = \begin{bmatrix} \cdots \hat{\boldsymbol{g}}_{s_{i}}I_{W_{s_{i}}} \cdots \end{bmatrix}
    :label: eq:vscmgD0

.. math:: [D_1] = \begin{bmatrix} \cdots \left(I_{W_{s_{i}}}\Omega_{i} + \frac{J_{s_{i}}}{2}\omega_{s_{i}} \right)\hat{\boldsymbol{g}}_{t_{i}} + \frac{J_{s_{i}}}{2}\omega_{t_{i}}\hat{\boldsymbol{g}}_{s_{i}} \cdots \end{bmatrix}
    :label: eq:vscmgD1

.. math:: [D_2] = \begin{bmatrix} \cdots \frac{1}{2}J_{t_{i}}(\omega_{t_{i}}\hat{\boldsymbol{g}}_{s_{i}}+\omega_{s_{i}}\hat{\boldsymbol{g}}_{t_{i}}) \cdots \end{bmatrix}
    :label: eq:vscmgD2

.. math:: [D_3] = \begin{bmatrix} \cdots J_{g_{i}}(\omega_{t_{i}}\hat{\boldsymbol{g}}_{s_{i}}-\omega_{s_{i}}\hat{\boldsymbol{g}}_{t_{i}}) \cdots \end{bmatrix}
    :label: eq:vscmgD3

.. math:: [D_4] = \begin{bmatrix} \cdots \frac{1}{2}(J_{s_{i}}-J_{t_{i}})(\hat{\boldsymbol{g}}_{s_{i}}\hat{\boldsymbol{g}}_{t_{i}}^{T}\boldsymbol{\omega}_{r} + \hat{\boldsymbol{g}}_{t_{i}}\hat{\boldsymbol{g}}_{s_{i}}^{T}\boldsymbol{\omega}_{r}) \cdots \end{bmatrix}
    :label: eq:vscmgD4

.. math:: [D] = ([D_1] - [D_2] + [D_3] + [D_4])
    :label: eq:vscmgD

where

.. math:: {\boldsymbol{ \omega}} = \omega_{s_{i}}\hat{\boldsymbol{ g}}_{s_{i}} + \omega_{t_{i}}\hat{\boldsymbol{ g}}_{t_{i}} + \omega_{g_{i}}\hat{\boldsymbol{ g}}_{g_{i}}
    :label: eq:omegaComponents

and :math:`\hat{\boldsymbol{g}}_{s_{i}}`, :math:`\hat{\boldsymbol{g}}_{t_{i}}`, and :math:`\hat{\boldsymbol{g}}_{g_{i}}` are the unit vectors in the direction of the first, second and third axis of the ith gimbal wheel system respectively, :math:`J_{s_{i}}`, :math:`J_{t_{i}}`, :math:`J_{g_{i}}` are the inertias for the ith gimbal wheel system for the first, second and third axis, :math:`I_{w_{s_{i}}}` is the ith RW spin axis inertia, and :math:`\boldsymbol{\omega}_{r}` is the reference angular velocity for the spacecraft.

Defining the :math:`2N\times2N` diagonal weighting matrix :math:`[W]`

.. math:: [W] = \text{diag}\{W_{s_{1}},\cdots,W_{s_{N}},W_{g_{1}},\cdots,W_{g_{N}}\}
    :label: eq:vscmgWeights

where :math:`W_{s_{i}}` and :math:`W_{g_{i}}` are the weights associated with the ith VSCMG RW and CMG respectively. The desired :math:`\dot{\boldsymbol{\eta}}` can be found through

.. math:: \dot{\boldsymbol{\eta}} = [W][Q]^{T}([Q][W][Q]^{T})^{-1}(-\boldsymbol{L}_{r})
    :label: eq:vscmgEtaDot

The weights are defined to be dependant on proximity to a CMG singularity through the use of the non-dimensional scalar

.. math:: \delta = \text{det}\frac{1}{\overline{h}^{2}}([D_{1}][D_{1}]^{T})
    :label: eq:vscmgDelta

where :math:`\overline{h}` is the nominal RW angular momentum found through

.. math:: \overline{h} = \frac{1}{N}\sum_{i=1}^{N}I_{w_{s_{i}}}\Omega_{i_{0}}
    :label: eq:vscmgHbar

where :math:`\Omega_{i_{0}}` is the nominal spin rate of the ith RW. As the CMG approaches a singularity :math:`\delta` goes to zero. The weights :math:`W_{s_{i}}` can be defined as

.. math:: W_{s_{i}} = W_{s_{i}}^{0}e^{(-\mu\delta)}
    :label: eq:vscmgWs

with :math:`W_{s_{i}}^{0}` and :math:`\mu` being positive scalars. The weights :math:`W_{g_{i}}` are held constant. This allows for the weights on the RW mode to be small when far from the CMG singularity but still allow the control to produce :math:`\boldsymbol{L}_{r}` if a singularity is reached.

User Guide
----------
This section is to outline the steps needed to setup the VSCMG velocity steering module in Python using Basilisk.

#. Import the vscmgVelocitySteering class::

    from Basilisk.fswAlgorithms import vscmgVelocitySteering

#. Create an instance of the vscmgVelocitySteering class::

    vscmgSteering = vscmgVelocitySteering.VscmgVelocitySteering()

#. Set the module parameters::

    mu = 1e-9
    vscmgSteering.setMu(mu)
    W0_s = [200.0] * numVSCMGs
    W_g = [1.0] * numVSCMGs
    vscmgSteering.setW0_s(W0_s)
    vscmgSteering.setW_g(W_g)

#. The VSCMG reference states output message is ``vscmgRefStatesOutMsg``.

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, vscmgSteering)
