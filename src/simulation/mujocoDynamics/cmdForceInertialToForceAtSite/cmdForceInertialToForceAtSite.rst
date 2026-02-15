Executive Summary
-----------------
The ``CmdForceInertialToForceAtSite`` module converts a commanded force vector expressed in the inertial frame into
an equivalent force vector expressed in a site-fixed frame. The resulting site-frame force is published as a
``ForceAtSiteMsgPayload`` for use by downstream force application modules.

Module Description
------------------
The module reads a commanded inertial force :math:`\mathbf{F}_N` from ``cmdForceInertialInMsg`` and obtains site
attitude :math:`\sigma_{SN}` from one of two sources:

- ``siteAttInMsg`` (preferred when linked)
- ``siteFrameStateInMsg`` (fallback when ``siteAttInMsg`` is not linked)

From :math:`\sigma_{SN}`, the direction cosine matrix :math:`\mathbf{C}_{SN}` is formed and used to rotate the force:

.. math::
   \mathbf{F}_S = \mathbf{C}_{SN}\mathbf{F}_N

The output vector :math:`\mathbf{F}_S` is written to ``forceOutMsg.force_S``.

Message Interfaces
------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdForceInertialInMsg
      - :ref:`CmdForceInertialMsgPayload`
      - Input commanded force vector in inertial frame, read from ``forceRequestInertial``.
    * - siteAttInMsg
      - :ref:`NavAttMsgPayload`
      - Optional/primary input attitude source. If linked, ``sigma_BN`` from this message is used to build the
        inertial-to-site rotation.
    * - siteFrameStateInMsg
      - :ref:`SCStatesMsgPayload`
      - Optional/fallback input attitude source. Used only when ``siteAttInMsg`` is not linked.
    * - forceOutMsg
      - :ref:`ForceAtSiteMsgPayload`
      - Output site-frame force vector, written to ``force_S``.

Detailed Behavior
-----------------
At each update step, the module performs the following operations:

1. Reads the input inertial force from ``cmdForceInertialInMsg``.
2. Selects the attitude source:
   - If ``siteAttInMsg`` is linked, use its ``sigma_BN``.
   - Otherwise use ``siteFrameStateInMsg.sigma_BN``.
3. Builds :math:`\mathbf{C}_{SN}` from the selected MRP attitude.
4. Rotates the force from inertial to site frame to compute :math:`\mathbf{F}_S`.
5. Writes ``forceOutMsg.force_S``.

During ``Reset()``, the module validates connectivity and logs errors if:

- ``cmdForceInertialInMsg`` is not linked, or
- both ``siteAttInMsg`` and ``siteFrameStateInMsg`` are unlinked.

Verification and Testing
------------------------
The module is verified by automated unit tests in
``src/simulation/mujocoDynamics/cmdForceInertialToForceAtSite/_UnitTest/test_cmdForceInertialToForceAtSite.py``.

Two test cases validate both attitude-source paths:

- ``test_cmdForceInertialToForceAtSiteSCStates``:
  links ``siteFrameStateInMsg`` and verifies
  :math:`\mathbf{F}_S = \mathbf{C}_{SN}\mathbf{F}_N` against an independent MRP-to-DCM reference implementation.
- ``test_cmdForceInertialToForceAtSiteNavAtt``:
  links both attitude messages and verifies that ``siteAttInMsg`` takes precedence over
  ``siteFrameStateInMsg`` by matching output to the NavAtt-based reference rotation.

Both tests check agreement to tight numerical tolerance.
