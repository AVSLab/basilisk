
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



import os  # Don't worry about this, standard stuff plus file discovery

import numpy
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import RigidBodyKinematics


def ckWrite(handle, time, MRPArray, avArray, startSeg, sc = -62, rf = "J2000"):
    """
    Purpose: Creates a CK kernel from a timeArray, MRPArray, and an avArray. Assumes that the SCLK is furnshed

    .. warning::

        time stamps for the timeArray, MRPArray, and avArray must line up exactly!!

    :param handle: What you would like the CK file to be named. Note, it must be in double quotes and end in .bc, ex: "moikernel.bc"
    :param time: numpy array of time stamps in nanoseconds
    :param MRPArray: array of modified Rodriguez parameters in column order x, y, z
    :param avArray: array of angular velocities about 3 axis in column order x, y, z
    :param startSeg: the SCLK time that the file begins at in UTC Gregorian ex: 'FEB 01,2021  12:00:55.9999 (UTC)'
    :param sc: spacecraft ID ex:-62
    :param rf: reference frame ex:"J2000"
    :return:
    """
    try:
        os.remove(handle)
    except OSError:
        pass
    fileHandle = pyswice.new_intArray(1)
    pyswice.ckopn_c(handle, "my-ckernel", 0, fileHandle)
    velLen = avArray.shape[0]
    velArray = pyswice.new_doubleArray(velLen * 3)
    z = MRPArray.shape[0]
    shapeMRP = numpy.shape(MRPArray)
    shapeavArray = numpy.shape(avArray)
    et = pyswice.new_doubleArray(1)
    pyswice.str2et_c(startSeg, et)
    starts = pyswice.new_doubleArray(1)
    pyswice.sce2c_c(sc, pyswice.doubleArray_getitem(et, 0), starts)
    zeroTime = 0  # pyswice.doubleArray_getitem(starts, 0)
    for w in range(velLen):
        for m in range(3):
            if shapeavArray[1] == 4:
                pyswice.doubleArray_setitem(velArray, (3 * w) + m, avArray[w, m + 1])
            else:
                pyswice.doubleArray_setitem(velArray, (3 * w) + m, avArray[w, m])
    quatArray = pyswice.new_doubleArray(z * 4)
    timeArray = pyswice.new_doubleArray(z)
    for i in range(z):
        for j in range(4):
            if shapeMRP[1] == 4:
                quat = RigidBodyKinematics.MRP2EP(MRPArray[i, 1:4])
                quat[1:4] = -quat[1:4]
            else:
                quat = RigidBodyKinematics.MRP2EP(MRPArray[i, 0:3])
                quat[1:4] = -quat[1:4]
            pyswice.doubleArray_setitem(quatArray, (4 * i) + j, quat[j])
        sclkdp = pyswice.new_doubleArray(1)
        pyswice.sce2c_c(-62, time[i] + zeroTime*1.0E-9, sclkdp)
        pyswice.doubleArray_setitem(timeArray, i, pyswice.doubleArray_getitem(sclkdp, 0))
    # Getting time into usable format
    encStartTime = pyswice.doubleArray_getitem(timeArray, 0) - 1.0e-3 #Pad the beginning for roundoff
    encEndTime = pyswice.doubleArray_getitem(timeArray, z-1) + 1.0e-3 #Pad the end for roundoff
    pyswice.ckw03_c(pyswice.intArray_getitem(fileHandle, 0), encStartTime, encEndTime, -62000, rf, 1,
                    "InertialData", z, timeArray, quatArray, velArray, 1, starts)
    pyswice.ckcls_c(pyswice.intArray_getitem(fileHandle, 0))
    return

def ckRead(time, SCID=-62, rf="J2000"):
    """
    Purpose: Read information out of a CK Kernel for a single instance and returns a quaternion array
    and an angular velocity array

    .. warning::

        Assumes that SCLK and CK kernels are already loaded using furnsh because pyswice gets mad when loading the same files over and over again.

    :param time: Should be in UTC Gregorian, and passed in as a string, ex: 'FEB 01,2021  14:00:55.9999 (UTC)'
    :param SCID: Spacecraft ID -- Default: -62
    :param rf: is a character string which specifies the, reference frame of the segment. Reference Frame, ex: "J2000"
    :return: None
    """
    #Getting time into usable format
    et = pyswice.new_doubleArray(1)
    pyswice.str2et_c(time, et)
    tick = pyswice.new_doubleArray(1)
    pyswice.sce2c_c(SCID, pyswice.doubleArray_getitem(et, 0), tick)
    cmat = pyswice.new_doubleArray(9)
    av = pyswice.new_doubleArray(3)
    clkout = pyswice.new_doubleArray(1)
    intArray = pyswice.new_intArray(1)
    cmatConversion = numpy.zeros((3, 3))
    kernalQuatArray = numpy.zeros((1, 4))
    kernMRPArray = numpy.zeros((1, 3))
    kernOmega = numpy.zeros((1, 3))
    pyswice.ckgpav_c(SCID, pyswice.doubleArray_getitem(tick, 0), 0, rf, cmat, av, clkout, intArray)
    for q in range(9):
        if q < 3:
            cmatConversion[0, q] = pyswice.doubleArray_getitem(cmat, q)
            kernOmega[0, q] = pyswice.doubleArray_getitem(av, q)
        elif q >= 6:
            cmatConversion[2, (q - 6)] = pyswice.doubleArray_getitem(cmat, q)
        else:
            cmatConversion[1, (q - 3)] = pyswice.doubleArray_getitem(cmat, q)
    kernalQuat = RigidBodyKinematics.C2EP(cmatConversion)
    kernMRP = RigidBodyKinematics.C2MRP(cmatConversion)
    for s in range(4):
        if s < 3:
            kernMRPArray[0, s] = -kernMRP[s]
        kernalQuatArray[0, s] = -kernalQuat[s]
        if s == 0:
            kernalQuatArray[0, s] = -kernalQuatArray[0, s]
    etout = pyswice.doubleArray_getitem(et, 0)
    return etout, kernalQuatArray, kernOmega

def ckInitialize(ck_file_in):
    pyswice.furnsh_c(ck_file_in)

def ckClose(ck_file_in):
    pyswice.unload_c(ck_file_in)

def spkRead(target, time, ref, observer):
    et = pyswice.new_doubleArray(1)
    pyswice.str2et_c(time, et)
    state = pyswice.new_doubleArray(6)
    lt = pyswice.new_doubleArray(1)

    pyswice.spkezr_c(target, pyswice.doubleArray_getitem(et, 0), ref, "NONE",
        observer, state, lt)
    stateArray = numpy.zeros(6)
    lightTime = pyswice.doubleArray_getitem(lt, 0)
    for i in range(6):
        stateArray[i] = pyswice.doubleArray_getitem(state, i)
    return stateArray
