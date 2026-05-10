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

from Basilisk.architecture import messaging, sysModel


class StateMerge(sysModel.SysModel):
    """
    Merge attitude and translation state sources into one spacecraft state.

    :ivar attStateInMsg: Attitude-centric spacecraft state input message.
    :ivar transStateInMsg: Translation-centric spacecraft state input message.
    :ivar stateOutMsg: Merged spacecraft state output message.

    The merged output is intended for navigation modules that expect both
    attitude and translation information in a single ``SCStatesMsgPayload``.
    """

    def __init__(self):
        super().__init__()
        self.attStateInMsg = messaging.SCStatesMsgReader()
        self.transStateInMsg = messaging.SCStatesMsgReader()
        self.stateOutMsg = messaging.SCStatesMsg()
        self.stateOut = messaging.SCStatesMsgPayload()

    def validateInputMessages(self):
        """Raise ``BasiliskError`` if a required input message is not linked."""
        requiredInputMessages = [
            ("attStateInMsg", self.attStateInMsg),
            ("transStateInMsg", self.transStateInMsg),
        ]
        for msgName, msgReader in requiredInputMessages:
            if not msgReader.isLinked():
                self.bskLogger.bskError(f"StateMerge.{msgName} was not linked.")

    def Reset(self, CurrentSimNanos):
        self.validateInputMessages()
        self.stateOutMsg.write(messaging.SCStatesMsgPayload())

    def UpdateState(self, CurrentSimNanos):
        attState = self.attStateInMsg()
        transState = self.transStateInMsg()

        # Copy attitude values from the attitude source.
        self.stateOut.sigma_BN = list(attState.sigma_BN)
        self.stateOut.omega_BN_B = list(attState.omega_BN_B)

        # Map translation source center-of-mass state into hub state fields for
        # downstream modules that read SCStatesMsgPayload.r_BN_N and v_BN_N.
        self.stateOut.r_BN_N = list(transState.r_CN_N)
        self.stateOut.v_BN_N = list(transState.v_CN_N)

        self.stateOutMsg.write(self.stateOut, CurrentSimNanos, self.moduleID)
