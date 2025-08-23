# ISC License
#
# Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import numpy as np
from Basilisk.architecture import messaging

#
#   FSW Setup Utilities for VSCMG
#
vscmgList = []

def create(
        gsHat0_B,
        gtHat0_B,
        ggHat_B,
        IG1,
        IG2,
        IG3,
        IW1,
        IW2,
        IW3,
        Omega0,
        gamma0,
        gammaDot0
    ):
    """
    Create a FSW VSCMG object

    This function is called to setup a FSW VSCMG device in python, and adds it to the list
    of VSCMG devices in vscmgList[].  This list is accessible from the parent python script that
    imported this vscmg library script, and thus any particular value can be over-ridden
    by the user.
    """
    global vscmgList

    # create the blank VSCMG object
    VSCMG = messaging.VSCMGConfigElementMsgPayload()

    norm = np.linalg.norm(gsHat0_B)
    if norm > 1e-10:
        gsHat0_B = gsHat0_B / norm
    else:
        print('Error: VSCMG gsHat0 input must be non-zero 3x1 vector')
        exit(1)
    norm = np.linalg.norm(gtHat0_B)
    if norm > 1e-10:
        gtHat0_B = gtHat0_B / norm
    else:
        print('Error: VSCMG gtHat0 input must be non-zero 3x1 vector')
        exit(1)
    norm = np.linalg.norm(ggHat_B)
    if norm > 1e-10:
        ggHat_B = ggHat_B / norm
    else:
        print('Error: VSCMG ggHat input must be non-zero 3x1 vector')
        exit(1)

    VSCMG.gsHat0_B = np.squeeze(np.array(gsHat0_B)).tolist()
    VSCMG.gtHat0_B = np.squeeze(np.array(gtHat0_B)).tolist()
    VSCMG.ggHat_B = np.squeeze(np.array(ggHat_B)).tolist()
    VSCMG.Js = IG1 + IW1
    VSCMG.Jt = IG2 + IW2
    VSCMG.Jg = IG3 + IW3
    VSCMG.Iws = IW1
    VSCMG.Omega0 = Omega0
    VSCMG.gamma0 = gamma0
    VSCMG.gammaDot0 = gammaDot0

    # add VSCMG to the list of VSCMG devices
    vscmgList.append(VSCMG)

    return

def writeConfigMessage():
    """
    Write FSW VSCMG array msg

    This function should be called after all devices are created with create()
    It creates the C-class container for the array of VSCMG devices, and attaches
    this container to the spacecraft object
    """
    global vscmgList

    Gs0Matrix_B = []
    Gt0Matrix_B = []
    GgMatrix_B = []
    JsList = []
    JtList = []
    JgList = []
    IwsList = []
    Omega0List = []
    gamma0List = []
    gammaDot0List = []

    for vscmg in vscmgList:
        Gs0Matrix_B.extend(vscmg.gsHat0_B)
        Gt0Matrix_B.extend(vscmg.gtHat0_B)
        GgMatrix_B.extend(vscmg.ggHat_B)
        JsList.append(vscmg.Js)
        JtList.append(vscmg.Jt)
        JgList.append(vscmg.Jg)
        IwsList.append(vscmg.Iws)
        Omega0List.append(vscmg.Omega0)
        gamma0List.append(vscmg.gamma0)
        gammaDot0List.append(vscmg.gammaDot0)


    vscmgConfigParams = messaging.VSCMGArrayConfigMsgPayload()
    vscmgConfigParams.Gs0Matrix_B = Gs0Matrix_B
    vscmgConfigParams.Gt0Matrix_B = Gt0Matrix_B
    vscmgConfigParams.GgMatrix_B = GgMatrix_B
    vscmgConfigParams.JsList = JsList
    vscmgConfigParams.JtList = JtList
    vscmgConfigParams.JgList = JgList
    vscmgConfigParams.IwsList = IwsList
    vscmgConfigParams.Omega0List = Omega0List
    vscmgConfigParams.gamma0List = gamma0List
    vscmgConfigParams.gammaDot0List = gammaDot0List
    vscmgConfigParams.numVSCMG = len(vscmgList)
    vscmgConfigMsg = messaging.VSCMGArrayConfigMsg().write(vscmgConfigParams)

    return vscmgConfigMsg

def clearSetup():
    global vscmgList

    vscmgList = []

    return

def getNumOfDevices():
    return len(vscmgList)
