#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import tempfile
import subprocess
import os
import platform
from typing import List, Union

import numpy as np

from Basilisk import __path__

bskPath = __path__[0]


def visualize(
    time: np.ndarray,
    qpos: np.ndarray,
    modelFileOrScene: Union[str, "MJScene"],
    speedUp: float = 1,
    track: str = "",
    files: List[str] = [],
):
    """Calls a tool to visualize the movement of multi-body systems
    simulated through ``MJScene``.

    To be able to use this function, one must have built Basilisk with
    the flag: '--mujocoReplay True'.

    The arguments ``time`` and ``qpos`` are typically the product of an
    ``MJScene`` state recorder::

        scene: MJScene = ...
        stateRecorder = scene.stateOutMsg.recorder()
        scSim.AddModelToTask("myTask", stateRecorder)

        # Run simulation

        mujoco.visualize(
            stateRecorder.times(),
            np.squeeze(stateRecorder.qpos),
            scene,
            ...
        )

    Args:
        time (np.ndarray): A one-dimensional array with the times
            at which the scene positions are recorded (in seconds)
        qpos (np.ndarray): A two-dimensional array with the position
            of the multi-body in general coordinates. The length of
            the first dimension must match the length of the ``time``
            argument.
        modelFileOrScene (Union[str, MJScene]): Path to a file
            describing the MuJoCo model (can be XML or compiled ``.mjb``).
            Alternatively, this can be an ``MJScene`` object, from
            which a model file will generated.
        speedUp (float, optional): Factor with which to speed up
            simulation replay. =1 is real time, =2 is double speed,
            =0.2 is five times slower, etc.. Defaults to 1.
        track (str, optional): Name of the body to track during visualization,
            by default, the first free body in the simulation. If 'none',
            camera is moved freely by the user. Defaults to "".
        files (List[str], optional): Paths to extra files to expose to
            MuJoCo, for example to load meshes. Defaults to [].
    """

    tqpos = np.column_stack([time, qpos])

    with tempfile.NamedTemporaryFile("w", delete=False) as fqpos:
        np.savetxt(fqpos, tqpos)

    if isinstance(modelFileOrScene, str):
        modelFile = modelFileOrScene
        fmodel = None
    else:
        with tempfile.NamedTemporaryFile("w", delete=False) as fmodel:
            modelFileOrScene.saveToFile(fmodel.name)
        modelFile = fmodel.name

    if platform.system() == "Windows":
        script_fn = "replay.exe"
    else:
        script_fn = "replay"
    script = os.path.join(bskPath, rf"utilities/mujocoUtils/bin/{script_fn}")

    if not os.path.exists(script):
        raise RuntimeError(f"Couldn't find the visualization tool at '{script}'."
                            " Did you build Basilisk with the flag "
                            "'--mujocoReplay True'? If so, did this tool build correctly?")

    args = [script, "--model", modelFile, "--state", fqpos.name]

    args.extend(["--speed", str(speedUp)])

    if track:
        args.extend(["--track", track])

    for file in files:
        args.extend(["--file", file])

    try:
        subprocess.check_output(args)
    except subprocess.CalledProcessError as e:
        print(e)

    os.unlink(fqpos.name)
    if fmodel is not None:
        os.unlink(fmodel.name)
