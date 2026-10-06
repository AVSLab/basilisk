#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import numpy as np

from Basilisk.architecture import messaging, sysModel


class CmdTorqueBodyToTorqueAtSite(sysModel.SysModel):
    """
    Convert a commanded body-frame torque into a torque expressed in a site frame.

    :ivar cmdTorqueInMsg: Commanded torque input message, expressed in body frame B.
    :ivar torqueOutMsg: Torque output message, expressed in site frame S.

    The output is ``torque_S = dcm_SB @ torqueRequestBody``. ``dcm_SB`` defaults
    to identity, which corresponds to a site frame aligned with the body frame.
    """

    def __init__(self):
        super().__init__()
        self.cmdTorqueInMsg = messaging.CmdTorqueBodyMsgReader()
        self.torqueOutMsg = messaging.TorqueAtSiteMsg()
        self.torqueOut = messaging.TorqueAtSiteMsgPayload()
        self._dcm_SB = np.eye(3)

    @property
    def dcm_SB(self):
        """Direction cosine matrix mapping body-frame vectors into the site frame"""
        return self._dcm_SB.copy()

    @dcm_SB.setter
    def dcm_SB(self, value):
        dcm = np.asarray(value, dtype = float)
        if dcm.shape != (3, 3):
            raise ValueError("CmdTorqueBodyToTorqueAtSite.dcm_SB must be a 3x3 matrix.")
        if not np.allclose(dcm @ dcm.T, np.eye(3), atol = 1e-10) or not np.isclose(np.linalg.det(dcm), 1.0, atol = 1e-10):
            raise ValueError("CmdTorqueBodyToTorqueAtSite.dcm_SB must be a proper rotation matrix.")
        self._dcm_SB = dcm

    def validateInputMessages(self):
        """Raise ``BasiliskError`` if a required input message is not linked."""
        if not self.cmdTorqueInMsg.isLinked():
            self.bskLogger.error("CmdTorqueBodyToTorqueAtSite.cmdTorqueInMsg was not linked.")

    def Reset(self, CurrentSimNanos):
        self.validateInputMessages()
        self.torqueOutMsg.write(messaging.TorqueAtSiteMsgPayload())

    def UpdateState(self, CurrentSimNanos):
        torque_B = np.array(self.cmdTorqueInMsg().torqueRequestBody)
        self.torqueOut.torque_S = list(self._dcm_SB @ torque_B)
        self.torqueOutMsg.write(self.torqueOut, CurrentSimNanos, self.moduleID)
