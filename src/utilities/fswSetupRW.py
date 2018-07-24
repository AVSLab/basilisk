''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''


#
#   FSW Setup Utilities for RW
#

import sys, os, inspect

from Basilisk.fswAlgorithms import fswMessages
import numpy


rwList = []

#
#   This function is called to setup a FSW RW device in python, and adds it to the of RW
#   devices in rwList[].  This list is accessible from the parent python script that
#   imported this rw library script, and thus any particular value can be over-ridden
#   by the user.
#
def create(
        gsHat_B,
        Js,
        uMax = numpy.NaN
    ):
    global rwList

    # create the blank RW object
    RW = fswMessages.RWConfigElementFswMsg()

    norm = numpy.linalg.norm(gsHat_B)
    if norm > 1e-10:
        gsHat_B = gsHat_B / norm
    else:
        print 'Error: RW gsHat input must be non-zero 3x1 vector'
        exit(1)

    RW.gsHat_B = gsHat_B
    RW.uMax = uMax
    RW.Js = Js

    # add RW to the list of RW devices
    rwList.append(RW)

    return

#
#   This function should be called after all devices are created with create()
#   It creates the C-class container for the array of RW devices, and attaches
#   this container to the spacecraft object
#
def writeConfigMessage(rwConfigMsgName, simObject, processName):
    global rwList

    GsMatrix_B = []
    JsList = []
    uMaxList = []
    for rw in rwList:
        GsMatrix_B.extend(rw.gsHat_B)
        JsList.extend([rw.Js])
        uMaxList.extend([rw.uMax])

    rwConfigParams = fswMessages.RWArrayConfigFswMsg()
    rwConfigParams.GsMatrix_B = GsMatrix_B
    rwConfigParams.JsList = JsList
    rwConfigParams.uMax = uMaxList
    rwConfigParams.numRW = len(rwList)

    messageSize = rwConfigParams.getStructSize()

    simObject.CreateNewMessage(processName,
                               rwConfigMsgName,
                               messageSize,
                               2)
    simObject.WriteMessageData( rwConfigMsgName,
                                messageSize,
                                0,
                                rwConfigParams)

    return

def clearSetup():
    global rwList

    rwList = []

    return

def getNumOfDevices():
    return len(rwList)
