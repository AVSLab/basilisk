# ISC License
#
# Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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
import os
import pytest

from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, pyswice_ck_utilities, simIncludeGravBody, RigidBodyKinematics as rbk
from Basilisk.topLevelModules import pyswice


@pytest.mark.timeout(30)  # seconds
def test_ck_read_write(tmp_path, show_plots):
    simulation = SimulationBaseClass.SimBaseClass()

    process = simulation.CreateNewProcess("testProcess")
    taskName = "task"
    dynTaskRate = macros.sec2nano(1.0)
    process.addTask(simulation.CreateNewTask(taskName, dynTaskRate))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraft"
    simulation.AddModelToTask(taskName, scObject)

    scObject.hub.mHub = 750.0
    scObject.hub.IHubPntBc_B = np.array([[900., 0., 0.],
                                         [0., 800., 0.],
                                         [0., 0., 600.]])

    scObject.hub.sigma_BNInit = [[0.1], [-0.2], [0.3]]
    scObject.hub.omega_BN_BInit = [[0.01], [-0.01], [0.03]]

    # Load up the leap second and spacecraft SPICE kernels
    gravFactory = simIncludeGravBody.gravBodyFactory()
    timeInit = 'FEB 01, 2021 12:00:00 (UTC)'
    spiceObject = gravFactory.createSpiceInterface(time=timeInit)
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'naif0011.tls')  # leap second file
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'MVN_SCLKSCET.00000.tsc')  # spacecraft clock file

    scObjectLogger = scObject.scStateOutMsg.recorder(dynTaskRate)
    simulation.AddModelToTask(taskName, scObjectLogger)

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(59))  # run for 59 seconds for easy time logic
    simulation.ExecuteSimulation()

    # Write a CK file using the attitude data from the simulation
    timeWrite = scObjectLogger.times()
    sigmaWrite = scObjectLogger.sigma_BN
    omegaWrite = scObjectLogger.omega_BN_B
    file_name = tmp_path / "test.bc"
    fileNameStr = str(file_name)
    print(fileNameStr, flush=True)
    print("DEBUG: Before ckWrite", flush=True)
    pyswice_ck_utilities.ckWrite(fileNameStr, timeWrite, sigmaWrite, omegaWrite, timeInit, spacecraft_id=-202)

    # Read the same CK file to check if the values are identical
    print("DEBUG: Before ckInitialize", flush=True)
    pyswice_ck_utilities.ckInitialize(fileNameStr)
    sigmaRead = np.empty_like(sigmaWrite)
    omegaRead = np.empty_like(omegaWrite)
    for idx in range(len(timeWrite)):
        print(f"DEBUG: Entering loop iteration {idx}", flush=True)  # Add this to see if it hangs within the loop
        # Change the time string to account for increasing time
        timeString = timeInit[:19] + f"{int(timeWrite[idx] * macros.NANO2SEC):02}" + timeInit[21:]
        _, kernQuat, kernOmega = pyswice_ck_utilities.ckRead(timeString, spacecraft_id=-202)

        sigmaRead[idx, :] = - rbk.EP2MRP(kernQuat)  # Convert from JPL-style quaternion notation
        omegaRead[idx, :] = kernOmega
    print("DEBUG: Before ckClose", flush=True)
    pyswice_ck_utilities.ckClose(fileNameStr)

    # Compare the read and write data
    np.testing.assert_allclose(sigmaRead, sigmaWrite)
    np.testing.assert_allclose(omegaRead, omegaWrite)

    if os.path.exists(fileNameStr):
        # Delete the file
        print("DEBUG: Before os.remove", flush=True)
        os.remove(fileNameStr)
