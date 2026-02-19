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
    Merge two SCStates messages into one SCStates output message for simpleNav use.

    Inputs:
    - attStateInMsg: attitude-centric state source
    - transStateInMsg: translation-centric state source

    Output:
    - stateOutMsg: merged state output for simpleNav use
    """

    def __init__(self):
        super().__init__()
        self.attStateInMsg = messaging.SCStatesMsgReader()
        self.transStateInMsg = messaging.SCStatesMsgReader()
        self.stateOutMsg = messaging.SCStatesMsg()
        self.stateOut = messaging.SCStatesMsgPayload()

    def Reset(self, CurrentSimNanos):
        self.stateOutMsg.write(messaging.SCStatesMsgPayload())

    def UpdateState(self, CurrentSimNanos):
        attState = self.attStateInMsg()
        transState = self.transStateInMsg()

        # Copy over attitude values from attitude source
        self.stateOut.sigma_BN = list(attState.sigma_BN)
        self.stateOut.omega_BN_B = list(attState.omega_BN_B)

        # Relocate the position/velocity from the translation source
        # this is relocation needed so due to later modules calling r_BN_N for things that relate to CoM
        self.stateOut.r_BN_N = list(transState.r_CN_N)
        self.stateOut.v_BN_N = list(transState.v_CN_N)

        self.stateOutMsg.write(self.stateOut, CurrentSimNanos, self.moduleID)
