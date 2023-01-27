#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
from Basilisk.architecture import messaging
from Basilisk.simulation import planetHeading
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import orbitalMotion as om


def test_planetHeading(show_plots=False, relTol=1e-8):
    """
    **Test Description**

    Test that a planet heading is properly calculated from a spacecraft and planet position and spacecraft attitude.
    To test this, the earth is placed at the inertial origin. A spacecraft with inertial attitude is placed
    at 1AU in the z-direction.  The heading is checked to be [0, 0, -1].
    These values were chosen arbitrarily. They are checked to be accurate to within a relative tolerance of the
    input ``relTol``, 1e-8 by default.

    Args:
        relTol (float): positive, the relative tolerance to which the result is checked.

    **Variables Being Tested**

    This test checks that ``headingOut`` stores the pulled log of the module ``bodyHeadingOutMsg``.

"""
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)

    earthPositionMessage = messaging.SpicePlanetStateMsgPayload()
    earthPositionMessage.PositionVector = [0., 0., 0.]
    plMsg = messaging.SpicePlanetStateMsg().write(earthPositionMessage)

    scPositionMessage = messaging.SCStatesMsgPayload()
    scPositionMessage.r_BN_N = [0., 0., om.AU*1000]
    scMsg = messaging.SCStatesMsg().write(scPositionMessage)

    ph = planetHeading.PlanetHeading()
    ph.ModelTag = "planetHeading"
    sim.AddModelToTask(task.Name, ph)

    ph.planetPositionInMsg.subscribeTo(plMsg)
    ph.spacecraftStateInMsg.subscribeTo(scMsg)

    dataLog = ph.planetHeadingOutMsg.recorder()
    sim.AddModelToTask(task.Name, dataLog)

    sim.InitializeSimulation()
    sim.TotalSim.SingleStepProcesses()
    headingOut = dataLog.rHat_XB_B[-1]

    assert headingOut == pytest.approx([0., 0., -1.], rel=relTol)


if __name__ == "__main__":
    test_planetHeading()
