
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
from Basilisk.utilities import RigidBodyKinematics, macros


def ckWrite(handle, time, mrp_array, av_array, start_seg, spacecraft_id=-62, reference_frame="J2000"):
    """
    Purpose: Creates a CK kernel from a time_array, mrp_array, and an av_array. Assumes that the SCLK is furnshed

    .. warning::

        time stamps for the time_array, mrp_array, and av_array must line up exactly!!

    :param handle: What you would like the CK file to be named. Note, it must be in double quotes and end in .bc, ex: "moikernel.bc"
    :param time: numpy array of time stamps in nanoseconds
    :param mrp_array: array of modified Rodriguez parameters in column order x, y, z
    :param av_array: array of angular velocities about 3 axis in column order x, y, z
    :param start_seg: the SCLK time that the file begins at in UTC Gregorian ex: 'FEB 01,2021  12:00:55.9999 (UTC)'
    :param spacecraft_id: spacecraft ID ex:-62
    :param reference_frame: reference frame ex:"J2000"
    :return:
    """
    try:
        os.remove(handle)
    except OSError:
        pass

    # Open the CK file
    file_handle = pyswice.new_spiceIntArray(1)
    pyswice.ckopn_c(handle, "my-ckernel", 0, file_handle)

    # Create empty containers for time, attitude and angular velocity
    num_data_points = len(time)
    time_array = pyswice.new_doubleArray(num_data_points)
    vel_array = pyswice.new_doubleArray(num_data_points * 3)
    quat_array = pyswice.new_doubleArray(num_data_points * 4)

    # Find the elapsed seconds between initial time and reference ephemeris
    ephemeris_time_container = pyswice.new_doubleArray(1)
    pyswice.str2et_c(start_seg, ephemeris_time_container)
    ephemeris_time = pyswice.doubleArray_getitem(ephemeris_time_container, 0)

    # Convert the initial time to number of spacecraft clock ticks
    start_ticks = pyswice.new_doubleArray(1)
    pyswice.sce2c_c(spacecraft_id, ephemeris_time, start_ticks)

    # Process data for each timestep
    for i in range(num_data_points):
        # Process the attitude
        quat = RigidBodyKinematics.MRP2EP(mrp_array[i, -3:])  # Grab the last 3 elements in case the first column is time
        quat[1:4] = - quat[1:4]  # Convert to JPL-style quaternions
        for j in range(4):
            pyswice.doubleArray_setitem(quat_array, (4 * i) + j, quat[j])

        # Process the angular velocity
        omega = av_array[i, -3:]  # Grab the last 3 elements in case the first column is time
        for j in range(3):
            pyswice.doubleArray_setitem(vel_array, (3 * i) + j, omega[j])

        # Process time
        current_time = ephemeris_time + time[i] * macros.NANO2SEC  # Compute the current time in elapsed seconds from ephemeris
        current_ticks = pyswice.new_doubleArray(1)
        pyswice.sce2c_c(spacecraft_id, current_time, current_ticks)  # Convert from ephemeris seconds to spacecraft clock ticks
        pyswice.doubleArray_setitem(time_array, i, pyswice.doubleArray_getitem(current_ticks, 0))

    # Get time into usable format
    encoded_start_time = pyswice.doubleArray_getitem(time_array, 0) - 1.0e-3  # Pad the beginning for roundoff
    encoded_end_time = pyswice.doubleArray_getitem(time_array, num_data_points - 1) + 1.0e-3  # Pad the end for roundoff

    # Save the date into a CK file
    pyswice.ckw03_c(pyswice.spiceIntArray_getitem(file_handle, 0), encoded_start_time, encoded_end_time, spacecraft_id,
                    reference_frame, 1, "InertialData", num_data_points, time_array, quat_array, vel_array, 1,
                    start_ticks)

    # Close the CK file
    pyswice.ckcls_c(pyswice.spiceIntArray_getitem(file_handle, 0))


def ckRead(time, spacecraft_id=-62, reference_frame="J2000"):
    """
    Purpose: Read information out of a CK Kernel for a single instance and returns a quaternion array
    and an angular velocity array

    .. warning::

        Assumes that SCLK and CK kernels are already loaded using furnsh because pyswice gets mad when loading the same files over and over again.

    :param time: Should be in UTC Gregorian, and passed in as a string, ex: 'FEB 01,2021  14:00:55.9999 (UTC)'
    :param spacecraft_id: Spacecraft ID -- Default: -62
    :param reference_frame: is a character string which specifies the, reference frame of the segment. Reference Frame, ex: "J2000"
    :return: None
    """
    # Find the elapsed seconds between initial time and reference ephemeris
    ephemeris_time_container = pyswice.new_doubleArray(1)
    pyswice.str2et_c(time, ephemeris_time_container)
    ephemeris_time = pyswice.doubleArray_getitem(ephemeris_time_container, 0)

    # Convert initial time to spacecraft clock tick
    tick = pyswice.new_doubleArray(1)
    pyswice.sce2c_c(spacecraft_id, ephemeris_time, tick)

    # Get attitude and angular velocity for a specified spacecraft clock time
    dcm_container = pyswice.new_doubleArray(9)
    av_container = pyswice.new_doubleArray(3)
    tick_container = pyswice.new_doubleArray(1)
    requested_pointing_flag = pyswice.new_spiceBoolArray(1)
    pyswice.ckgpav_c(spacecraft_id, pyswice.doubleArray_getitem(tick, 0), 0, reference_frame, dcm_container,
                     av_container, tick_container, requested_pointing_flag)

    # Grab angular velocity
    omega = numpy.zeros(3)
    for i in range(3):
        omega[i] = pyswice.doubleArray_getitem(av_container, i)

    # Grab attitude as a DCM
    dcm = numpy.zeros((3, 3))
    for i in range(9):
        dcm[i // 3, i % 3] = pyswice.doubleArray_getitem(dcm_container, i)  # Map 9D array into 3x3 matrix

    # Convert attitude to quaternions
    quat = RigidBodyKinematics.C2EP(dcm)
    quat[1:4] = - quat[1:4]  # Convert to JPL-style quaternions

    return ephemeris_time, quat, omega


def ckInitialize(ck_file_in):
    pyswice.furnsh_c(ck_file_in)


def ckClose(ck_file_in):
    pyswice.unload_c(ck_file_in)
