# ISC License
#
# Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder
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

"""Test that the magnetic field base class evaluates a time dependent model at the step midpoint.

The WMM field changes with the decimal year of the evaluation time. The spacecraft is at rest, so the extrapolation
moves nothing and only the evaluation time of the model can change the field."""

import numpy as np

from Basilisk.architecture import messaging
from Basilisk.simulation import magneticFieldWMM
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

STEP = 10.0  # [s] module update period
EPOCH_SECONDS = 30.0  # [s] seconds of the minute of the epoch 2025-06-01 12:00


def _field(extrapolate, epochSeconds=EPOCH_SECONDS):
    """Return the WMM field at the third module update, at 20 s.

    Args:
        extrapolate (bool): True to enable the extrapolation of the spacecraft state to the step midpoint.
        epochSeconds (float): [s] seconds of the minute of the epoch 2025-06-01 12:00.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(STEP)))

    wmm = magneticFieldWMM.MagneticFieldWMM()
    wmm.ModelTag = "WMM"
    wmm.configureWMMFile(str(get_path(DataFile.MagneticFieldData.WMM)))
    wmm.setExtrapolateScStateToStepMidpoint(extrapolate)
    epochPayload = messaging.EpochMsgPayload()
    epochPayload.year = 2025
    epochPayload.month = 6
    epochPayload.day = 1
    epochPayload.hours = 12
    epochPayload.minutes = 0
    epochPayload.seconds = epochSeconds
    epochMsg = messaging.EpochMsg().write(epochPayload)
    wmm.epochInMsg.subscribeTo(epochMsg)

    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = [(orbitalMotion.REQ_EARTH + 400.0) * 1000.0, 0.0, 0.0]  # [m]
    scMsg = messaging.SCStatesMsg().write(payload, 0)
    wmm.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("t", wmm)
    recorder = wmm.envOutMsgs[0].recorder()
    scSim.AddModelToTask("t", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(STEP))
    scSim.ExecuteSimulation()
    # the spacecraft rewrites its state one step later, so that the module observes the spacecraft task period
    scMsg.write(payload, macros.sec2nano(STEP))
    scSim.ConfigureStopTime(macros.sec2nano(2 * STEP))
    scSim.ExecuteSimulation()
    return np.array(recorder.magField_N[-1])


def test_model_time_is_the_step_midpoint():
    """Verify the model is evaluated at the step midpoint when the extrapolation applies.

    With the extrapolation the third update, at 20 s, evaluates the model at 15 s. This must equal the field of a run
    without extrapolation whose epoch is 5 s earlier, and differ from the field of a run without extrapolation at the
    same epoch, which evaluates at 20 s."""
    fieldMidpoint = _field(True)
    fieldMidpointReference = _field(False, EPOCH_SECONDS - STEP / 2.0)
    fieldCurrentTime = _field(False)

    np.testing.assert_allclose(fieldMidpoint, fieldMidpointReference, rtol=1e-13, atol=0.0)
    relativeChange = np.linalg.norm(fieldMidpoint - fieldCurrentTime) / np.linalg.norm(fieldCurrentTime)
    assert relativeChange > 1e-12  # the secular variation over 5 s is of the order of 1e-10


if __name__ == "__main__":
    test_model_time_is_the_step_midpoint()
