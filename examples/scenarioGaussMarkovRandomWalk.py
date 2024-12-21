#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
---------

This script demonstrates the difference between bounded Gauss-Markov random walk and pure Gaussian noise by simulating
two IMU sensors with different configurations. The simulation shows how the Gauss-Markov process can be configured
either to maintain bounded random walk behavior or to generate pure Gaussian noise. The script sets up a spacecraft
in an elliptical Earth orbit and attaches two IMU sensors with different noise configurations.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioGaussMarkovRandomWalk.py

When the simulation completes a plot is shown for the IMU sensor measurements over time.

Simulation Scenario Setup Details
----------------------------------
The simulation layout is shown in the following illustration. A single simulation process is created which
contains both the spacecraft simulation module and two IMU sensor modules. Each IMU sensor has different
configurations for the process noise (P Matrix), state propagation (A Matrix), and walk bounds.

The dynamics simulation is setup using a :ref:`Spacecraft` module in an elliptical Earth orbit. The IMU sensors
are configured with different parameters:

- IMU 1 demonstrates bounded random walk behavior:
    - Uses non-zero A Matrix for state propagation (-0.1 on diagonal)
    - Has positive walk bounds (±3.0 rad/s by default)
    - Shows mean-reverting behavior characteristic of Gauss-Markov process
    - Uses process noise level of 0.5 by default

- IMU 2 demonstrates pure Gaussian noise:
    - Uses zero A Matrix (set after initialization)
    - Has negative walk bounds to disable random walk
    - Shows independent noise samples with no temporal correlation
    - Uses the same process noise level as IMU 1

Both IMUs use the same process noise level (P Matrix) to ensure comparable noise magnitudes.

Note that any sensors using the ``GaussMarkov`` noise model should be configured with
user-defined configuration parameters such as ``walkBounds`` and ``AMatrix``. While this
scenario intentionally configures noise to demonstrate different behaviors, in normal usage
these parameters should start disabled by default and only be enabled when explicitly needed.

Illustration of Simulation Results
-----------------------------------

The following plot shows the angular velocity measurements from both IMU sensors over a 10-minute period:

.. image:: /_images/Scenarios/scenarioGaussMarkovRandomWalk.svg
   :align: center

The plot demonstrates how:
- IMU 1 (blue) exhibits bounded random walk behavior within ±3.0 rad/s bounds
- IMU 2 (orange) shows pure Gaussian noise without temporal correlation
- The bounded random walk maintains memory of previous states and shows smoother transitions
- The pure Gaussian noise shows more rapid, independent variations

The simulation provides insight into how different Gauss-Markov process configurations affect sensor
measurement behavior and demonstrates the flexibility of the model to generate both bounded random
walk and pure Gaussian noise patterns.
"""

import numpy as np
import matplotlib.pyplot as plt
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.simulation import imuSensor
from Basilisk.simulation import spacecraft
from Basilisk.architecture import messaging
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import svIntegrators

def run(show_plots, processNoiseLevel=0.5, walkBounds=3.0):
    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Create simulation
    scSim = SimulationBaseClass.SimBaseClass()

    # Create the simulation process and task
    simulationTimeStep = macros.sec2nano(0.1)  # update process rate update time
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Create spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraft"

    # Set the integrator to RKF45
    integratorObject = svIntegrators.svIntegratorRKF45(scObject)
    scObject.setIntegrator(integratorObject)

    # Set spacecraft mass and inertia properties
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = [
        [900.0, 0.0, 0.0],
        [0.0, 800.0, 0.0],
        [0.0, 0.0, 600.0]
    ]

    # Setup gravity model
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([planet])

    # Set initial spacecraft state
    oe = orbitalMotion.ClassicElements()
    oe.a = planet.radEquator + 7500e3  # meters
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(planet.mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    # Add spacecraft to simulation
    scSim.AddModelToTask(simTaskName, scObject)

    # Create IMU sensor modules
    imuSensor1 = imuSensor.ImuSensor()
    imuSensor1.ModelTag = "imuSensor1"

    # Configure IMU1 with bounded random walk
    imuSensor1.PMatrixGyro = [
        [processNoiseLevel, 0.0, 0.0],
        [0.0, processNoiseLevel, 0.0],
        [0.0, 0.0, processNoiseLevel]
    ]
    imuSensor1.AMatrixGyro = [
        [-0.1, 0.0, 0.0],
        [0.0, -0.1, 0.0],
        [0.0, 0.0, -0.1]
    ]
    imuSensor1.setWalkBoundsGyro(np.array([walkBounds, walkBounds, walkBounds], dtype=np.float64))
    imuSensor1.applySensorErrors = True
    imuSensor1.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # Create second IMU with pure Gaussian noise
    imuSensor2 = imuSensor.ImuSensor()
    imuSensor2.ModelTag = "imuSensor2"

    # Configure IMU2 matrices first
    imuSensor2.PMatrixGyro = [
        [processNoiseLevel, 0.0, 0.0],
        [0.0, processNoiseLevel, 0.0],
        [0.0, 0.0, processNoiseLevel]
    ]
    imuSensor2.walkBoundsGyro = [-1.0, -1.0, -1.0]
    imuSensor2.senRotBias = [0.0, 0.0, 0.0]
    imuSensor2.applySensorErrors = True
    imuSensor2.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # Add both IMUs to simulation
    scSim.AddModelToTask(simTaskName, imuSensor1)
    scSim.AddModelToTask(simTaskName, imuSensor2)

    # Set up messages for both IMU's
    dataLog1 = imuSensor1.sensorOutMsg.recorder()
    dataLog2 = imuSensor2.sensorOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, dataLog1)
    scSim.AddModelToTask(simTaskName, dataLog2)

    scSim.InitializeSimulation()

    # Set IMU2's A Matrix to zero to demonstrate different error propagation behavior.
    imuSensor2.AMatrixGyro = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ]

    simulationTime = macros.min2nano(10)
    scSim.ConfigureStopTime(simulationTime)

    scSim.ExecuteSimulation()

    # Get time vector
    timeData = dataLog1.times() * macros.NANO2SEC

    # Get gyro data from the sensor messages
    data1 = []
    data2 = []

    # Get the recorded messages by calling the record() method
    messages1 = dataLog1.record()()
    messages2 = dataLog2.record()()

    # Process each message
    for i in range(len(messages1)):
        msg1 = messages1[i]
        msg2 = messages2[i]

        # Try to access the gyro data
        try:
            data1.append(msg1.AngVelPlatform[0])  # Use AngVelPlatform instead of sensedValues
            data2.append(msg2.AngVelPlatform[0])
        except Exception as e:
            print(f"\nError accessing gyro data: {str(e)}")
            return None

    data1 = np.array(data1)
    data2 = np.array(data2)

    # Calculate statistics
    std1 = np.std(data1)
    std2 = np.std(data2)

    print(f"\nStandard deviations:")
    print(f"IMU1: {std1:.3f} rad/s")
    print(f"IMU2: {std2:.3f} rad/s")

    # Create figure dictionary to store plots
    figureList = {}

    # Create the plot
    plt.figure(1, figsize=(12, 8))
    plt.plot(timeData, data1, label='IMU 1 (Bounded Random Walk)', alpha=0.7)
    plt.plot(timeData, data2, label='IMU 2 (Pure Gaussian)', alpha=0.7)
    # Plot bounds for IMU1 only
    plt.axhline(y=walkBounds, color='r', linestyle='--', alpha=0.3, label='IMU1 Bounds')
    plt.axhline(y=-walkBounds, color='r', linestyle='--', alpha=0.3)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('IMU Gyro Measurements: Random Walk vs Pure Gaussian')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # Store the figure in the figure dictionary
    pltName = 'scenarioGaussMarkovRandomWalk'
    figureList[pltName] = plt.figure(1)

    if show_plots:
        plt.show()
    plt.close('all')  # Close plots to free memory

    return figureList

if __name__ == "__main__":
    run(True)
