#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

"""Regression tests for two MJBody mass-property bugs.

1. ``_com`` site position: the auto-created ``<body>_com`` site must report the
   body's true center of mass, not the body frame origin.  MuJoCo latches a
   site's ``sameframe`` at compile time; because the site is created at the body
   origin it latches to ``mjSAMEFRAME_BODY`` and ``mj_local2Global`` then ignores
   the later attempt to move it to the CoM, pinning ``site_xpos`` to the body
   frame origin.  The orientation of the ``_com`` site must remain the body
   (origin) frame, not the principal-inertia frame.

2. Inertia update on mass change: ``MJBody::updateMujocoModelFromMassProps``
   rescales the inertia tensor with the ratio ``newMass / body_mass`` but
   overwrites ``body_mass`` with ``newMass`` first, making the ratio identically
   1.0 -- so the inertia never changes when the mass does.
"""

import os

import pytest

try:
    from Basilisk.simulation import mujoco

    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

from Basilisk.architecture import messaging
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

import numpy as np


# A single free body whose geom (and therefore center of mass) is offset from
# the body frame origin by COM_OFFSET along the body z-axis.
COM_OFFSET = 2.0  # [m]
_BODY_WITH_OFFSET_COM_XML = f"""
<mujoco>
    <option gravity="0 0 0" integrator="RK4" timestep="0.01"/>
    <worldbody>
        <body name="sat">
            <freejoint/>
            <geom type="box" size="1 1 0.5" pos="0 0 {COM_OFFSET}" density="100"/>
        </body>
    </worldbody>
</mujoco>
"""

# A free body used to probe the rotational inertia through its angular response
# to a constant applied torque.  Box half-extents (1, 1, 2) -> full sides
# (2, 2, 4); Ixx = m/12 * (2^2 + 4^2).
_ROTOR_XML = """
<mujoco>
    <option gravity="0 0 0" integrator="RK4" timestep="0.01"/>
    <worldbody>
        <body name="rotor">
            <freejoint/>
            <geom type="box" size="1 1 2" density="100"/>
        </body>
    </worldbody>
</mujoco>
"""


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_com_site_reports_true_center_of_mass():
    """The ``_com`` site must sit at the true CoM (offset from the body origin)
    and keep the body-origin frame orientation, not the principal-inertia frame.
    """
    dt = 0.01  # s

    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene(_BODY_WITH_OFFSET_COM_XML)
    scSim.AddModelToTask("test", scene)

    body = scene.getBody("sat")
    comRec = body.getCenterOfMass().stateOutMsg.recorder()
    originRec = body.getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", comRec)
    scSim.AddModelToTask("test", originRec)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(dt))
    scSim.ExecuteSimulation()

    r_com = np.array(comRec.r_BN_N)[-1]
    r_origin = np.array(originRec.r_BN_N)[-1]
    sigma_com = np.array(comRec.sigma_BN)[-1]
    sigma_origin = np.array(originRec.sigma_BN)[-1]

    # The body has not moved (no forces), so the origin sits at the world origin
    # and the CoM must be COM_OFFSET away from it.
    offset = np.linalg.norm(r_com - r_origin)
    assert offset == pytest.approx(COM_OFFSET, abs=1e-9), (
        f"_com site is {offset} m from the body origin; expected {COM_OFFSET} m "
        "(the geom CoM offset). A value near 0 means the site is pinned to the "
        "body frame origin (sameframe latched to BODY)."
    )

    # The _com site orientation must match the body-origin frame, NOT be rotated
    # into the principal-inertia frame.
    np.testing.assert_allclose(sigma_com, sigma_origin, atol=1e-12)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_inertia_rescales_with_mass():
    """When the body mass changes, the rotational inertia must scale with it.

    A constant torque produces angular acceleration ``alpha = tau / I``.  As the
    mass (and hence inertia) grows, ``alpha`` must shrink proportionally.  With
    the inertia-update bug the inertia never changes, so ``alpha`` stays constant
    and the early/late angular-acceleration ratio is ~1 instead of ~0.5.
    """
    dt = 0.01  # s
    tf = 2.0  # s
    torque = 5.0  # N*m about the body x-axis
    massDot = 800.0  # kg/s -> mass doubles over tf (initial mass is 1600 kg)

    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene(_ROTOR_XML)
    scSim.AddModelToTask("test", scene)

    scene.addTorqueActuator("trq", "rotor_origin")
    body = scene.getBody("rotor")

    torquePayload = messaging.TorqueAtSiteMsgPayload()
    torquePayload.torque_S = [torque, 0.0, 0.0]
    torqueMsg = messaging.TorqueAtSiteMsg()
    torqueMsg.write(torquePayload)
    scene.getTorqueActuator("trq").torqueInMsg.subscribeTo(torqueMsg)

    massDotPayload = messaging.SCMassPropsMsgPayload()
    massDotPayload.massSC = massDot
    massDotMsg = messaging.SCMassPropsMsg()
    massDotMsg.write(massDotPayload)
    body.derivativeMassPropertiesInMsg.subscribeTo(massDotMsg)

    omegaRec = body.getOrigin().stateOutMsg.recorder()
    massRec = body.massPropertiesOutMsg.recorder()
    scSim.AddModelToTask("test", omegaRec)
    scSim.AddModelToTask("test", massRec)

    scSim.InitializeSimulation()
    massInitial = body.getMass()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    times = np.array(omegaRec.times()) * macros.NANO2SEC
    omega_x = np.array(omegaRec.omega_BN_B)[:, 0]
    mass = np.array(massRec.massSC)

    # Sanity: the mass really did roughly double over the run.
    assert mass[-1] == pytest.approx(2.0 * massInitial, rel=1e-3)

    def slope(i, j):
        return (omega_x[j] - omega_x[i]) / (times[j] - times[i])

    n = len(times)
    alphaEarly = slope(1, 5)
    alphaLate = slope(n - 5, n - 1)

    # Inertia ~ mass, so when mass doubles the angular acceleration halves.
    assert alphaLate / alphaEarly == pytest.approx(0.5, rel=0.1), (
        f"alpha(late)/alpha(early) = {alphaLate / alphaEarly:.3f}; expected ~0.5. "
        "A ratio near 1.0 means the inertia did not rescale with the mass."
    )


if __name__ == "__main__":
    test_com_site_reports_true_center_of_mass()
    test_inertia_rescales_with_mass()
