#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
#   Unit Test Script
#   Module Name:        motorThermal
#   Author:             João Vaz Carneiro
#   Creation Date:      March 4, 2021
#

import numpy as np
import pytest

from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import motorThermal
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import spacecraft
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW


# Fixed module configuration shared by the scenarios below.  Each value is held
# constant so the expected temperature can be derived analytically from the
# module's heat-balance equations rather than from a stored regression vector.
DT = 0.1                 # [s] task update period
N_STEPS = 10             # number of update steps simulated per scenario
THERMAL_RESISTANCE = 10.0  # [Celsius/W] ambient thermal resistance
HEAT_CAPACITY = 10.0       # [J/Celsius] motor heat capacity


def referenceTemperatures(scenario, times):
    r"""Reproduce the module heat balance step-by-step for the given log times.

    This mirrors ``MotorThermal::computeTemperature``:

    - ``wheelPower    = Omega * u_current``
    - ``frictionHeat  = Omega * frictionTorque``
    - ``heatGeneration = dt * (|wheelPower| / eff * (1 - eff) + |frictionHeat|)``
    - ``heatDissipation = dt * (T - T_ambient) / R``
    - ``T <- T + (heatGeneration - heatDissipation) / C``

    ``dt`` is taken from the spacing of the recorder time stamps so the reference
    uses exactly the same time step the module saw (the module computes
    ``dt = CurrentSimNanos - prevTime`` and starts with ``prevTime`` at the
    simulation start time).
    """
    temperature = scenario["currentTemperature"]
    previousNanos = 0
    expected = []
    for currentNanos in times:
        dt = (currentNanos - previousNanos) * 1.0e-9
        wheelPower = scenario["Omega"] * scenario["u_current"]
        frictionHeat = scenario["Omega"] * scenario["frictionTorque"]
        heatGeneration = dt * (
            abs(wheelPower) / scenario["efficiency"] * (1.0 - scenario["efficiency"])
            + abs(frictionHeat)
        )
        heatDissipation = dt * (temperature - scenario["ambientTemperature"]) / THERMAL_RESISTANCE
        temperature = temperature + (heatGeneration - heatDissipation) / HEAT_CAPACITY
        expected.append(temperature)
        previousNanos = currentNanos
    return np.array(expected)


def runScenario(scenario):
    """Drive the motorThermal module with a standalone reaction wheel state
    message holding constant, fully-controlled inputs.  Returns the recorded
    log times (in nanoseconds) and the temperature history."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    process = unitTestSim.CreateNewProcess("TestProcess")
    process.addTask(unitTestSim.CreateNewTask("unitTask", macros.sec2nano(DT)))

    thermalModel = motorThermal.MotorThermal()
    thermalModel.ModelTag = "rwThermals"
    thermalModel.currentTemperature = scenario["currentTemperature"]
    thermalModel.ambientTemperature = scenario["ambientTemperature"]
    thermalModel.efficiency = scenario["efficiency"]
    thermalModel.ambientThermalResistance = THERMAL_RESISTANCE
    thermalModel.motorHeatCapacity = HEAT_CAPACITY

    # Feed the module a stand-alone reaction wheel state message so the wheel
    # speed, motor torque and friction torque are known exactly.  This isolates
    # the thermal model from the reaction wheel dynamics.
    rwStatePayload = messaging.RWConfigLogMsgPayload()
    rwStatePayload.Omega = scenario["Omega"]                    # [rad/s]
    rwStatePayload.u_current = scenario["u_current"]            # [N*m]
    rwStatePayload.frictionTorque = scenario["frictionTorque"]  # [N*m]
    rwStateMsg = messaging.RWConfigLogMsg().write(rwStatePayload)
    thermalModel.rwStateInMsg.subscribeTo(rwStateMsg)

    unitTestSim.AddModelToTask("unitTask", thermalModel)
    tempLog = thermalModel.temperatureOutMsg.recorder()
    unitTestSim.AddModelToTask("unitTask", tempLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(DT * N_STEPS))
    unitTestSim.ExecuteSimulation()

    return np.array(tempLog.times()), np.array(tempLog.temperature)


# Each scenario isolates a single physical mechanism of the heat balance so that
# a failure points at the responsible term rather than at an opaque truth value.
# Units per scenario field: Omega [rad/s], u_current [N*m], frictionTorque [N*m],
# efficiency [-] (dimensionless), currentTemperature [Celsius],
# ambientTemperature [Celsius].
SCENARIOS = [
    # Pure dissipation, motor hotter than ambient: no power and no friction means
    # no heat is generated, so the temperature must decay monotonically toward
    # the (lower) ambient temperature.
    dict(name="dissipation_cooling",
         Omega=0.0, u_current=0.0, frictionTorque=0.0,     # [rad/s], [N*m], [N*m]
         efficiency=0.5,                                   # [-]
         currentTemperature=20.0, ambientTemperature=0.0,  # [Celsius], [Celsius]
         mechanism="dissipation", direction="cooling"),
    # Pure dissipation, motor colder than ambient: temperature must rise
    # monotonically toward the (higher) ambient temperature.
    dict(name="dissipation_warming",
         Omega=0.0, u_current=0.0, frictionTorque=0.0,     # [rad/s], [N*m], [N*m]
         efficiency=0.5,                                   # [-]
         currentTemperature=0.0, ambientTemperature=20.0,  # [Celsius], [Celsius]
         mechanism="dissipation", direction="warming"),
    # Pure inefficiency heating: friction is off and the motor starts at ambient
    # temperature, so on the first step there is no dissipation and all heating
    # comes from the motor power inefficiency term.
    dict(name="inefficiency_only",
         Omega=100.0, u_current=0.2, frictionTorque=0.0,   # [rad/s], [N*m], [N*m]
         efficiency=0.5,                                   # [-]
         currentTemperature=20.0, ambientTemperature=20.0, # [Celsius], [Celsius]
         mechanism="inefficiency", direction="heating"),
    # Pure friction heating: the motor torque is zero (so the inefficiency term
    # vanishes regardless of efficiency) and the motor starts at ambient
    # temperature, so the first-step heating comes solely from friction.
    dict(name="friction_only",
         Omega=100.0, u_current=0.0, frictionTorque=0.01,  # [rad/s], [N*m], [N*m]
         efficiency=0.5,                                   # [-]
         currentTemperature=20.0, ambientTemperature=20.0, # [Celsius], [Celsius]
         mechanism="friction", direction="heating"),
    # All three mechanisms active at once with the motor hotter than ambient.
    dict(name="combined",
         Omega=100.0, u_current=0.2, frictionTorque=0.01,  # [rad/s], [N*m], [N*m]
         efficiency=0.5,                                   # [-]
         currentTemperature=20.0, ambientTemperature=0.0,  # [Celsius], [Celsius]
         mechanism="combined", direction="heating"),
]


@pytest.mark.parametrize("scenario", SCENARIOS, ids=[s["name"] for s in SCENARIOS])
@pytest.mark.parametrize("accuracy", [1e-8])
def test_motorThermal(show_plots, scenario, accuracy):
    r"""
    **Validation Test Description**

    This unit test validates the :ref:`motorThermal` heat-balance model by
    feeding it a stand-alone reaction wheel state message with fully controlled
    wheel speed, motor torque and friction torque.  Because the inputs are known
    exactly, the expected temperature history is derived analytically from the
    module equations instead of being compared against a stored regression
    vector of "magic" truth numbers.

    Five scenarios each isolate one part of the model:

    - ``dissipation_cooling`` / ``dissipation_warming``: with no wheel power and
      no friction the only active term is heat dissipation, so the temperature
      must move monotonically toward the ambient temperature (down when the motor
      is hotter, up when it is colder).
    - ``inefficiency_only``: with friction disabled and the motor starting at the
      ambient temperature, the first-step temperature rise must equal the motor
      power inefficiency contribution ``dt * |Omega * u_current| / eff * (1 - eff) / C``.
    - ``friction_only``: with the motor torque set to zero (which removes the
      inefficiency term for any efficiency) and the motor starting at ambient,
      the first-step temperature rise must equal the friction contribution
      ``dt * |Omega * frictionTorque| / C``.
    - ``combined``: all three terms active at once, checked against the full
      analytic heat-balance recurrence.

    Every scenario also checks the complete temperature history against the
    step-by-step analytic recurrence to the supplied accuracy.

    **Test Parameters**

    Args:
        accuracy (float): absolute accuracy used to compare the module output to
            the analytic reference temperatures.

    **Description of Variables Being Tested**

    The recorded ``temperature`` history is compared element-by-element against
    ``referenceTemperatures``, and the first-step temperature change is compared
    against the closed-form contribution of the mechanism isolated by each
    scenario.
    """
    times, temperatures = runScenario(scenario)
    expected = referenceTemperatures(scenario, times)

    # The recorded history must match the analytic heat-balance recurrence.
    # rtol=0.0 so the supplied accuracy is enforced as a pure absolute tolerance
    # (NumPy otherwise applies a default rtol=1e-7).
    np.testing.assert_allclose(
        temperatures, expected, atol=accuracy, rtol=0.0,
        err_msg=f"motorThermal temperature history mismatch for scenario '{scenario['name']}'",
    )

    # There must be at least one full update step (the sample at t = 0 uses
    # dt = 0 and leaves the temperature unchanged at its initial value).
    assert len(temperatures) > 1
    assert temperatures[0] == pytest.approx(scenario["currentTemperature"], abs=accuracy)
    firstStep = temperatures[1] - scenario["currentTemperature"]

    # Independent, closed-form check of the mechanism isolated by each scenario.
    if scenario["mechanism"] == "dissipation":
        # No heat is generated, so the temperature relaxes toward ambient.
        expectedFirstStep = -DT * (scenario["currentTemperature"] - scenario["ambientTemperature"]) / THERMAL_RESISTANCE / HEAT_CAPACITY
        assert firstStep == pytest.approx(expectedFirstStep, abs=accuracy)
        differences = np.diff(temperatures[1:])
        if scenario["direction"] == "cooling":
            assert np.all(differences < 0.0)   # monotonically cooling
            assert np.all(temperatures[1:] >= scenario["ambientTemperature"])
        else:
            assert np.all(differences > 0.0)   # monotonically warming
            assert np.all(temperatures[1:] <= scenario["ambientTemperature"])

    elif scenario["mechanism"] == "inefficiency":
        # First step starts at ambient, so dissipation is zero and the rise is
        # entirely from the power inefficiency term.
        wheelPower = scenario["Omega"] * scenario["u_current"]
        expectedFirstStep = DT * abs(wheelPower) / scenario["efficiency"] * (1.0 - scenario["efficiency"]) / HEAT_CAPACITY
        assert firstStep == pytest.approx(expectedFirstStep, abs=accuracy)
        assert firstStep > 0.0

    elif scenario["mechanism"] == "friction":
        # Motor torque is zero, so the inefficiency term vanishes and the rise is
        # entirely from friction.
        frictionHeat = scenario["Omega"] * scenario["frictionTorque"]
        expectedFirstStep = DT * abs(frictionHeat) / HEAT_CAPACITY
        assert firstStep == pytest.approx(expectedFirstStep, abs=accuracy)
        assert firstStep > 0.0


def test_motorThermal_integration(show_plots):
    r"""
    **Validation Test Description**

    Integration check that :ref:`motorThermal` works when wired to a live
    :ref:`ReactionWheelStateEffector` (the connection used in real scenarios such
    as :ref:`scenarioTempMeasurementAttitude`), rather than to a stand-alone
    message. A single reaction wheel is spun under a constant motor torque with
    friction enabled and the motor starts at the ambient temperature.

    Because the wheel speed, motor torque and friction torque evolve with the
    reaction wheel dynamics, this scenario does not assert exact temperatures
    (those are covered analytically in ``test_motorThermal``). It verifies the
    end-to-end message path instead: the temperatures must stay finite (no
    ``NaN`` from an unconnected or mis-typed message) and the motor must heat up,
    since a spinning, loaded wheel generates heat and there is no dissipation at
    the starting ambient temperature.
    """
    unitTestSim = SimulationBaseClass.SimBaseClass()
    process = unitTestSim.CreateNewProcess("TestProcess")
    process.addTask(unitTestSim.CreateNewTask("unitTask", macros.sec2nano(DT)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    rwFactory = simIncludeRW.rwFactory()
    rwFactory.create("Honeywell_HR16", [1, 0, 0], Omega=300.,  # [RPM]
                     maxMomentum=50.,  # [N*m*s]
                     useRWfriction=True)
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "ReactionWheel"
    rwFactory.addToSpacecraft(rwStateEffector.ModelTag, rwStateEffector, scObject)

    cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    cmdArray.motorTorque = [0.2, 0., 0.]  # [N*m]
    cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(cmdMsg)

    thermalModel = motorThermal.MotorThermal()
    thermalModel.ModelTag = "rwThermals"
    thermalModel.currentTemperature = 20.  # [Celsius]
    thermalModel.ambientTemperature = 20.  # [Celsius]
    thermalModel.efficiency = 0.5
    thermalModel.ambientThermalResistance = THERMAL_RESISTANCE
    thermalModel.motorHeatCapacity = HEAT_CAPACITY
    thermalModel.rwStateInMsg.subscribeTo(rwStateEffector.rwOutMsgs[0])

    # Order matters: the reaction wheel effector must update (and publish its
    # state message) before motorThermal reads it within the same task step.
    unitTestSim.AddModelToTask("unitTask", rwStateEffector)
    unitTestSim.AddModelToTask("unitTask", scObject)
    unitTestSim.AddModelToTask("unitTask", thermalModel)
    tempLog = thermalModel.temperatureOutMsg.recorder()
    unitTestSim.AddModelToTask("unitTask", tempLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(DT * N_STEPS))
    unitTestSim.ExecuteSimulation()

    temperatures = np.array(tempLog.temperature)
    assert np.all(np.isfinite(temperatures)), "motorThermal produced non-finite temperatures"
    assert temperatures[0] == pytest.approx(20., abs=1e-8)
    assert temperatures[-1] > temperatures[0], "a spinning, loaded wheel should heat the motor"


if __name__ == "__main__":
    for s in SCENARIOS:
        test_motorThermal(False, s, 1e-8)
    test_motorThermal_integration(False)
    print("PASSED")
