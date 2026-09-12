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

"""Check fixed-step integrator convergence against an analytic circular orbit."""

import sys

import numpy as np
import pytest

from Basilisk.simulation import gravityEffector, spacecraft, svIntegrators
from Basilisk.utilities import SimulationBaseClass, macros


def _final_position(integrator_name, step_count):
    """Propagate a unit-radius circular orbit for one second with equal steps."""
    duration = 1.0  # [s]
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("dynamics")
    process.addTask(sim.CreateNewTask("orbit", macros.sec2nano(duration/step_count)))

    central_body = gravityEffector.GravBodyData()
    central_body.planetName = "central_body"
    central_body.mu = 1.0  # [m^3/s^2]
    central_body.isCentralBody = True
    body = spacecraft.Spacecraft()
    body.hub.r_CN_NInit = [1.0, 0.0, 0.0]  # [m]
    body.hub.v_CN_NInit = [0.0, 1.0, 0.0]  # [m/s]
    body.gravField.gravBodies = spacecraft.GravBodyVector([central_body])

    if integrator_name == "rk3":
        integrator = svIntegrators.svIntegratorRungeKutta(
            body,
            a_coefficients=[[0, 0, 0], [1/2, 0, 0], [-1, 2, 0]],
            b_coefficients=[1/6, 2/3, 1/6],
            c_coefficients=[0, 1/2, 1],
        )
    else:
        integrator_class = {
            "euler": svIntegrators.svIntegratorEuler,
            "rk2": svIntegrators.svIntegratorRK2,
            "rk4": svIntegrators.svIntegratorRK4,
        }[integrator_name]
        integrator = integrator_class(body)
    body.setIntegrator(integrator)

    sim.AddModelToTask("orbit", body)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    assert sim.TotalSim.CurrentNanos == macros.sec2nano(duration)
    return np.array(body.scStateOutMsg.read().r_BN_N)


@pytest.mark.parametrize("integrator_name, expected_order", [
    ("euler", 1), ("rk2", 2), ("rk3", 3), ("rk4", 4),
])
def test_integrator_convergence(integrator_name, expected_order):
    r"""Verify the global position error scales as the expected power of step size.

    With unit gravitational parameter, radius, and tangential speed, the exact
    position is :math:`\mathbf{r}(t) = [\cos(t), \sin(t), 0]` in SI units.
    Compare the position at one second for four successively halved steps.
    Each error ratio should approach :math:`2^p` for a method of order :math:`p`.

    :param integrator_name: Fixed-step integrator to exercise.
    :param expected_order: Expected global convergence order.
    """
    final_angle = 1.0  # [rad], one second at 1 rad/s
    reference = np.array([np.cos(final_angle), np.sin(final_angle), 0.0])  # [m]
    # Binary subdivisions land exactly on the final time in integer nanoseconds
    # and keep even RK4's finest-step error well above floating-point roundoff.
    errors = np.array([
        np.linalg.norm(_final_position(integrator_name, count)-reference)
        for count in (8, 16, 32, 64)
    ])  # [m]

    assert np.all(np.isfinite(errors)) and np.all(errors > 0), errors
    assert np.all(np.diff(errors) < 0), errors
    observed_orders = np.log2(errors[:-1]/errors[1:])
    # Allow finite-step corrections while still distinguishing adjacent orders.
    np.testing.assert_allclose(
        observed_orders, expected_order, rtol=0, atol=0.25,
        err_msg=f"{integrator_name}: position errors {errors}",
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, *sys.argv[1:]]))
