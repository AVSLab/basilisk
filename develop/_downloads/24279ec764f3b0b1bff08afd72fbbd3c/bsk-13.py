# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass, macros


def run(num_threads=2):
    """Propagate two independent spacecraft on the requested number of workers.

    :param num_threads: Positive number of simulation worker threads.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.resetThreads(num_threads)
    time_step = macros.sec2nano(0.1)  # [ns]
    duration = 1.0  # [s]
    speeds = [1.0, 2.0]  # [m/s]
    recorders = []

    for index, speed in enumerate(speeds):
        process = scSim.CreateNewProcess(f"spacecraftProcess{index}")
        task_name = f"spacecraftTask{index}"
        process.addTask(scSim.CreateNewTask(task_name, time_step))

        body = spacecraft.Spacecraft()
        body.ModelTag = f"spacecraft{index}"
        body.hub.mHub = 1.0  # [kg]
        body.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
        body.hub.r_CN_NInit = [0.0, 0.0, 0.0]  # [m]
        body.hub.v_CN_NInit = [speed, 0.0, 0.0]  # [m/s]
        scSim.AddModelToTask(task_name, body, 1)

        recorder = body.scStateOutMsg.recorder()
        scSim.AddModelToTask(task_name, recorder, 0)
        recorders.append(recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(duration))  # [ns]
    scSim.ExecuteSimulation()

    position_tolerance = 1e-12  # [m]
    for index, (recorder, speed) in enumerate(zip(recorders, speeds)):
        expected_position = [speed * duration, 0.0, 0.0]  # [m]
        position = recorder.r_BN_N[-1]  # [m]
        np.testing.assert_allclose(position, expected_position, rtol=0, atol=position_tolerance)
        print(f"spacecraft{index}: x = {position[0]:.3f} m")


if __name__ == "__main__":
    run()
