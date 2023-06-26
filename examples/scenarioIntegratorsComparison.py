#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

r"""
Overview
--------
This scenario illustrates how different integrators compare in terms of
accuracy and computational cost.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioIntegratorsComparison.py

For information on how to setup different integrators, see :ref:`scenarioIntegrators` and :ref:`scenarioVariableTimeStepIntegrators`. 

Currently, Basilisk only supports explicit Runge-Kutta integrators,
both of the regular and adaptive variations. Non-adaptive Runge-Kutta
integrators can be controlled solely by the step size: larger step
sizes means that faster computation, but less accurate results. 

In contrast, adaptive Runge-Kutta methods are affected both by the step
size and absolute and relative tolerance. These integrators will try
to use the user-given step size, but if the error grows too large, a
smaller time step is used internally for greater accuracy.

When using an adaptive integrator, the Basilisk dynamics task time step
can be increased without risk of increasing the integration error. However, 
this also means that other modules in the task are updated less often,
which might be undesirable. Additionally, spacecraft state messages will
also be updated less frequently. 

Finally, each integrator is associated with an order. Greater
order integrators are more accurate, but more computationally
expensive. The order of a method cannot be altered by users.

Comparison of integrators
-------------------------
Five integrators are compared in this section, two of them adaptive.
These are the Euler method (order 1), Heun's method (order 2),
the Runge-Kutta 4 (order 4), the Runge-Kutta-Fehlberg 4(5) (adaptive
with order 5), and the Runge-Kutta-Fehlberg 7(8) (adaptive with order 8).
The adaptive integrators are used with two different absolute 
tolerances: 0.1 and 10.

Each integrator is used to propagate a two-body orbit around the
Earth. The final position of each propagation is compared to 
the analytical solution, and a final position error is obtained.
Moreover, the time that it takes to propagate each orbit is recorded.

.. image:: /_images/Scenarios/scenarioIntegratorsComparison.svg
   :align: center

The figure above shows the results for the analysis previously described.
Note that the axis are expressed in logarithmic scale.

For the fixed step integrators, one can see a clear linear trend for
the error with respect to the step size. Moreover, the slope of each
of these lines depends on the order of the method. Thus, reducing the
size of the time steps produces exponential gains in accuracy, with this
exponential improvement being stronger for methods of higher order.

Unfortunately, the computational costs also grow in an exponential
fashion with the time step. However, it is interesting to note that
the slope of these lines is not dependent on the method order.

Analyzing the adaptive Runge-Kutta methods is more challenging.
One can distinguish two distinct behaviours. For small time steps,
the methods behave similarly to fixed-step RK methods. This is
because the user-provided step size is small enough to achieve
the desired accuracy, and thus no "adaption" is needed. For larger
time steps, however, the integrator has to take smaller internal
steps to adjust to the desired accuracy, and thus we see that the
position error does not depend on the user-provided time step,
but instead it depends on the tolerance used.

In terms of computational cost, adaptive RK methods behave
similarly to fixed-step RK methods. The main difference occurs
for the larger time steps in which time adaption takes place.
Here, a tighter tolerance would translate into higher computational
costs. However, this is hard to see in the plot given the inherent
noisiness of performance measuring. 

Fixed-timestep integrators are helpful when you want your simulation runtime 
to be consistent as you vary simulation parameters. Since there is no adaptation, 
runtime will be similar even if the parameters change the stiffness of the system's 
dynamic equations. Of course, this comes at the cost of accuracy, but can it be 
very useful for hardware-in-the-loop simulations.

One should note that adaptive RK methods are inherently slower than
their fixed-step counterparts. This is because the former methods
have a greater computational overhead. Adaptive methods are preferable
when the simulation step size can be made large enough, or when the
stiffness of the dynamic changes significantly during propagation
(i.e. for very elliptical orbits). Fixed-step methods, on the other
hand, should be preferred when the simulation has to run at small
enough time steps (because other simulation models need updating),
or when the dynamics remain similar during propagation.

To sum up, choosing an integrator has a significant impact
on the accuracy and speed of your simulation. However, there is no
single, perfect intergator for every problem. The most optimal
alternative may only be found through testing.
"""

import time
from dataclasses import dataclass
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import svIntegrators

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport

# The following code may not be very interesting to learn how to
# set up different integrators. To learn this, refer to the other
# two scenarios: scenarioIntegrators and scenarioAdaptiveIntegrators


@dataclass
class IntegratorData:
    label: str
    type: type
    color: int
    linestyle: str = "-"
    absTol: Optional[float] = None


INTEGRATORS = {
    "euler": IntegratorData("Euler", svIntegrators.svIntegratorEuler, 0),
    "rk2": IntegratorData("Heun", svIntegrators.svIntegratorRK2, 1),
    "rk4": IntegratorData("RK 4", svIntegrators.svIntegratorRK4, 2),
    "rkf45 10": IntegratorData(
        r"RKF4(5) - $\epsilon = 10$", svIntegrators.svIntegratorRKF45, 3, "-", 10
    ),
    "rkf45 0.1": IntegratorData(
        r"RKF4(5) - $\epsilon = 0.1$", svIntegrators.svIntegratorRKF45, 3, "--", 0.1
    ),
    "rkf78 10": IntegratorData(
        r"RKF7(8) - $\epsilon = 10$", svIntegrators.svIntegratorRKF78, 4, "-", 10
    ),
    "rkf78 0.1": IntegratorData(
        r"RKF7(8) - $\epsilon = 0.1$", svIntegrators.svIntegratorRKF78, 4, "--", 0.1
    ),
}


def run(show_plots=True):
    time_steps_to_test = 2.0 ** np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])

    reference = get_reference()

    errors = {}
    times = {}

    for integrator in INTEGRATORS:
        errors[integrator] = []
        times[integrator] = []
        for time_step in time_steps_to_test:
            tic = time.time()
            _, pos = get_solution(integrator, time_step)
            toc = time.time()
            errors[integrator].append(np.linalg.norm(reference - pos[-1, :]))
            times[integrator].append(toc - tic)

    fig, axes = plt.subplots(nrows=2, figsize=(12, 7))
    ax = axes[0]
    for integrator in INTEGRATORS:
        ax.loglog(
            time_steps_to_test,
            errors[integrator],
            color=unitTestSupport.getLineColor(INTEGRATORS[integrator].color, 5),
            linestyle=INTEGRATORS[integrator].linestyle,
            label=INTEGRATORS[integrator].label,
        )

    ax.legend()
    ax.set_ylabel("Position error [m]")

    ax = axes[1]
    for integrator in INTEGRATORS:
        ax.loglog(
            time_steps_to_test,
            times[integrator],
            color=unitTestSupport.getLineColor(INTEGRATORS[integrator].color, 5),
            linestyle=INTEGRATORS[integrator].linestyle,
            label=INTEGRATORS[integrator].label,
        )

    ax.legend()
    ax.set_xlabel("Time step [s]")
    ax.set_ylabel("Integration time [s]")

    if show_plots:
        plt.show()

    return {"scenarioIntegratorsComparison": fig}


def get_initial_conditions():
    mu = simIncludeGravBody.gravBodyFactory().createEarth().mu

    oe = orbitalMotion.ClassicElements()
    rLEO = 7000.0 * 1000  # meters
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)
    return rN, vN, oe


def get_simulation_time():
    mu = simIncludeGravBody.gravBodyFactory().createEarth().mu
    _, _, oe = get_initial_conditions()

    # Ensure all time steps that are a power of 2 lower or equal
    # 11 can finish the propagation exactly.
    desired_divisor = 2**11
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2 * np.pi / n
    propagation_time = (P // desired_divisor + 1) * desired_divisor
    return propagation_time


def get_reference():
    mu = simIncludeGravBody.gravBodyFactory().createEarth().mu
    _, _, oe = get_initial_conditions()
    propagation_time = get_simulation_time()

    E0 = orbitalMotion.f2E(oe.f, oe.e)
    M0 = orbitalMotion.E2M(E0, oe.e)
    Mf = M0 + np.sqrt(mu / oe.a / oe.a / oe.a) * propagation_time
    Ef = orbitalMotion.M2E(Mf, oe.e)
    ff = orbitalMotion.E2f(Ef, oe.e)

    oe.f = ff

    rN, _ = orbitalMotion.elem2rv(mu, oe)
    return rN


def get_solution(integrator, time_step):
    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(time_step)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Set up integrator depending on the data
    integratorData = INTEGRATORS[integrator]
    integratorObject = integratorData.type(scObject)
    if integratorData.absTol is not None:
        integratorObject.relTol = 0
        integratorObject.absTol = integratorData.absTol
    scObject.setIntegrator(integratorObject)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # Create gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(
        list(gravFactory.gravBodies.values())
    )

    rN, vN, _ = get_initial_conditions()

    # Initialize Spacecraft States with in the initialization variables
    scObject.hub.r_CN_NInit = rN  # m
    scObject.hub.v_CN_NInit = vN  # m/s

    # Setup data logging
    dataLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, dataLog)

    # Initialize Simulation
    scSim.InitializeSimulation()

    # configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(get_simulation_time()))
    scSim.ExecuteSimulation()

    # retrieve the logged data
    times = dataLog.times()
    posData = dataLog.r_BN_N

    return times, posData


if __name__ == "__main__":
    run(True)
