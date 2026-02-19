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


class ThrFiringRound(sysModel.SysModel):
    """
    Convert thruster force commands into thruster on-time commands.

    Mapping per thruster i:
        onTime_i = controlPeriodSec * force_i / thrForceMax_i

    Then rounded to nearest multiple of onTimeResolutionSec.
    """

    def __init__(self):
        super().__init__()

        # Input/output messages
        self.thrForceInMsg = messaging.THRArrayCmdForceMsgReader()
        self.thrOnTimeOutMsg = messaging.THRArrayOnTimeCmdMsg()
        self.thrOnTimePayload = messaging.THRArrayOnTimeCmdMsgPayload()

        # Configuration
        self.controlPeriodSec = 1.0
        self.onTimeResolutionSec = 0.01
        self.numThrusters = None
        self.thrForceMax = 1.0

    def setControlPeriodSec(self, controlPeriodSecIn: float):
        self.controlPeriodSec = float(controlPeriodSecIn)

    def setOnTimeResolutionSec(self, onTimeResolutionSecIn: float):
        self.onTimeResolutionSec = float(onTimeResolutionSecIn)

    def setNumThrusters(self, numThrustersIn: int):
        self.numThrusters = int(numThrustersIn)

    def setThrForceMax(self, thrForceMaxIn):
        """
        Set thrust normalization for force->on-time conversion.

        Accepted:
        - scalar: same max force for all thrusters
        - vector length nThr: per-thruster max force
        """
        thrForceMaxArr = np.asarray(thrForceMaxIn, dtype=float)
        if thrForceMaxArr.ndim == 0:
            self.thrForceMax = float(thrForceMaxArr)
            return
        if thrForceMaxArr.ndim != 1:
            raise ValueError("setThrForceMax expects scalar or 1D vector.")
        for thr in thrForceMaxArr:
            if thr <= 0.0:
                raise ValueError("setThrForceMax expects positive values.")
        self.thrForceMax = thrForceMaxArr.copy()

    def resolveNumThrusters(self, thrForceCmd: np.ndarray) -> int:

        thrForceMaxArr = np.asarray(self.thrForceMax, dtype=float)
        if thrForceMaxArr.ndim == 1:
            return int(thrForceMaxArr.size)

        return int(thrForceCmd.size)

    def resolveThrForceMax(self, nThr: int) -> np.ndarray:
        thrForceMaxArr = np.asarray(self.thrForceMax, dtype=float)
        if thrForceMaxArr.ndim == 0:
            return np.full(nThr, float(thrForceMaxArr), dtype=float)
        if thrForceMaxArr.ndim != 1:
            raise ValueError("thrForceMax must be scalar or 1D vector.")
        if thrForceMaxArr.size == 1:
            return np.full(nThr, float(thrForceMaxArr[0]), dtype=float)
        if thrForceMaxArr.size != nThr:
            raise ValueError(f"thrForceMax vector length {thrForceMaxArr.size} does not match nThr {nThr}.")
        return thrForceMaxArr.copy()

    def Reset(self, CurrentSimNanos):
        self.thrOnTimeOutMsg.write(messaging.THRArrayOnTimeCmdMsgPayload())

    def UpdateState(self, CurrentSimNanos):
        thrForceMsg = self.thrForceInMsg()
        thrForceCmd = np.asarray(thrForceMsg.thrForce, dtype=float)

        if self.numThrusters is None:
            nThr = self.resolveNumThrusters(thrForceCmd)
        else:
            nThr = self.numThrusters
        thrForceMax = self.resolveThrForceMax(nThr)

        # Force commands are one-sided for on-time logic.
        forceCmd = np.maximum(thrForceCmd[:nThr], 0.0)
        onTimeRequest = np.zeros(nThr)
        onTimeRequest = self.controlPeriodSec * forceCmd / thrForceMax
        onTimeRequest = np.clip(onTimeRequest, 0.0, self.controlPeriodSec)
        onTimeRequest = np.rint(onTimeRequest / self.onTimeResolutionSec) * self.onTimeResolutionSec
        onTimeRequest = np.clip(onTimeRequest, 0.0, self.controlPeriodSec)

        self.thrOnTimePayload.OnTimeRequest = onTimeRequest.tolist()
        self.thrOnTimeOutMsg.write(self.thrOnTimePayload, CurrentSimNanos, self.moduleID)
