#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems
#  Laboratory, University of Colorado Boulder
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
#

import pytest

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import orbElemOffset


@pytest.mark.parametrize("useMeanAnomalyOffset", [False, True])
def test_orbElemOffset(useMeanAnomalyOffset):
    """
    Functional test for OrbElemOffset:

    - When useMeanAnomalyOffset is False:
        All ClassicElements fields are summed element wise:
            out = main + offset

    - When useMeanAnomalyOffset is True:
        All fields except f are still summed element wise. The f field
        is computed by interpreting offset.f as a mean anomaly increment
        DeltaM (rad) and performing

            E_main   = f2E(main.f, main.e)
            M_main   = E2M(E_main, main.e)
            e_out    = main.e + offset.e
            M_out    = M_main + offset.f
            f_out    = E2f(M2E(M_out, e_out), e_out)
    """

    # 1. Create test simulation
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("TestProc")
    task = scSim.CreateNewTask("unitTask", macros.sec2nano(0.1))
    proc.addTask(task)

    # 2. Construct the module
    module = orbElemOffset.OrbElemOffset()
    module.ModelTag = "OrbElemOffset"
    module.useMeanAnomalyOffset = useMeanAnomalyOffset
    scSim.AddModelToTask("unitTask", module)

    # 3. Create two ClassicElements messages
    main = messaging.ClassicElementsMsgPayload()
    offset = messaging.ClassicElementsMsgPayload()

    # Main elements (choose a moderately eccentric orbit)
    main.a = 1.0
    main.e = 0.1
    main.i = 0.2
    main.Omega = 0.3
    main.omega = 0.4
    main.f = 0.5

    # Offsets: small increments
    offset.a = 2.0
    offset.e = 0.02
    offset.i = 0.03
    offset.Omega = 0.04
    offset.omega = 0.05
    # For the angle offset, this will be interpreted either as df or DeltaM
    offset.f = 0.06

    msgMain = messaging.ClassicElementsMsg().write(main)
    msgOffset = messaging.ClassicElementsMsg().write(offset)

    # 4. Subscribe module input messages
    module.mainElementsInMsg.subscribeTo(msgMain)
    module.offsetElementsInMsg.subscribeTo(msgOffset)

    # 5. Record output
    recorder = module.elementsOutMsg.recorder()
    scSim.AddModelToTask("unitTask", recorder)

    # Initialize and run one step
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(0.1))
    scSim.ExecuteSimulation()

    # 6. Extract result - only one write should have occurred
    assert recorder.a.shape[0] > 0
    idx = 0

    # Common expected fields: always simple sums
    expected_a = main.a + offset.a
    expected_e = main.e + offset.e
    expected_i = main.i + offset.i
    expected_Omega = main.Omega + offset.Omega
    expected_omega = main.omega + offset.omega

    # Check non angle fields
    assert recorder.a[idx] == pytest.approx(expected_a)
    assert recorder.e[idx] == pytest.approx(expected_e)
    assert recorder.i[idx] == pytest.approx(expected_i)
    assert recorder.Omega[idx] == pytest.approx(expected_Omega)
    assert recorder.omega[idx] == pytest.approx(expected_omega)

    # 7. Check the anomaly handling
    if not useMeanAnomalyOffset:
        # Plain true anomaly increment
        expected_f = main.f + offset.f
        assert recorder.f[idx] == pytest.approx(expected_f)
    else:
        recorderM = orbitalMotion.E2M(orbitalMotion.f2E(recorder.f[idx], recorder.e[idx]), recorder.e[idx])
        mainM = orbitalMotion.E2M(orbitalMotion.f2E(main.f, main.e), main.e)
        assert (recorderM - mainM) == pytest.approx(offset.f, rel=1e-12, abs=1e-12)


if __name__ == "__main__":
    # Run the test standalone for quick checking
    test_orbElemOffset(useMeanAnomalyOffset=False)
    test_orbElemOffset(useMeanAnomalyOffset=True)
