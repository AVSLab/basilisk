Executive Summary
-----------------

Converts a commanded body-frame torque into a ``TorqueAtSiteMsgPayload``
expressed in a site frame, for driving a MuJoCo torque actuator at that site.


Message Connection Descriptions
-------------------------------
The following diagram and table list the module input and output messages.

.. bsk-module-io:: CmdTorqueBodyToTorqueAtSite
    :caption: Module I/O Messages

    input cmdTorqueInMsg CmdTorqueBodyMsgPayload
        Input commanded torque ``torqueRequestBody``, expressed in body frame B.

    output torqueOutMsg TorqueAtSiteMsgPayload
        Output torque ``torque_S``, expressed in site frame S.


Module Assumptions and Limitations
----------------------------------
The output is :math:`{}^{S}\boldsymbol{L} = [SB]\,{}^{B}\boldsymbol{L}`, where
``dcm_SB`` is a constant body-to-site rotation that defaults to identity
(aligned frames). The configured matrix is copied on assignment; subsequent
changes to the caller's array do not affect the module.
The torque is treated as a pure couple, so the site position
does not affect the output. ``Reset()`` reports an error if ``cmdTorqueInMsg``
is not linked.


User Guide
----------

.. code-block:: python

    from Basilisk.simulation import cmdTorqueBodyToTorqueAtSite

    torqueBridge = cmdTorqueBodyToTorqueAtSite.CmdTorqueBodyToTorqueAtSite()
    torqueBridge.ModelTag = "torqueBridge"
    torqueBridge.dcm_SB = dcm_SB  # optional, 3x3 proper rotation matrix
    torqueBridge.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    torqueActuator.torqueInMsg.subscribeTo(torqueBridge.torqueOutMsg)
