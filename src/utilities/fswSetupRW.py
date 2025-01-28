# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


import numpy
from Basilisk.architecture import messaging

#
#   FSW Setup Utilities for RW
#
rwList = []


def create(
        gsHat_B,
        Js,
        uMax = numpy.nan
    ):
    """
    Create a FSW RW object

    This function is called to setup a FSW RW device in python, and adds it to the of RW
    devices in rwList[].  This list is accessible from the parent python script that
    imported this rw library script, and thus any particular value can be over-ridden
    by the user.
    """
    global rwList

    # create the blank RW object
    RW = messaging.RWConfigElementMsgPayload()

    norm = numpy.linalg.norm(gsHat_B)
    if norm > 1e-10:
        gsHat_B = gsHat_B / norm
    else:
        print('Error: RW gsHat input must be non-zero 3x1 vector')
        exit(1)

    RW.gsHat_B = gsHat_B
    RW.uMax = uMax
    RW.Js = Js

    # add RW to the list of RW devices
    rwList.append(RW)

    return


def writeConfigMessage():
    """
    Write FSW RW array msg

    This function should be called after all devices are created with create()
    It creates the C-class container for the array of RW devices, and attaches
    this container to the spacecraft object

    """
    global rwList

    GsMatrix_B = []
    JsList = []
    uMaxList = []
    for rw in rwList:
        GsMatrix_B.extend(rw.gsHat_B)
        JsList.extend([rw.Js])
        uMaxList.extend([rw.uMax])

    rwConfigParams = messaging.RWArrayConfigMsgPayload()
    rwConfigParams.GsMatrix_B = GsMatrix_B
    rwConfigParams.JsList = JsList
    rwConfigParams.uMax = uMaxList
    rwConfigParams.numRW = len(rwList)
    rwConfigMsg = messaging.RWArrayConfigMsg().write(rwConfigParams)

    return rwConfigMsg


def clearSetup():
    global rwList

    rwList = []

    return


def getNumOfDevices():
    return len(rwList)
