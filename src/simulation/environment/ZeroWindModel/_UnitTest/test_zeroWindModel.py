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

#
#   Unit Test Script
#   Module Name:        zeroWindModel
#   Author:             Carlo Cena
#   Creation Date:      2026
#

import numpy as np
import pytest

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.simulation import zeroWindModel
from Basilisk.architecture import messaging
from Basilisk.utilities import orbitalMotion


def test_zeroWindModel_basic_functionality():
    """Test that ZeroWindModel correctly outputs co-rotating atmosphere velocity."""

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    # Create ZeroWindModel
    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "zeroWind"

    # Set custom planet angular velocity
    omega_planet = np.array([0.0, 0.0, 1.0e-4])  # [rad/s]
    windModel.setPlanetOmega_N(omega_planet)
    windModel.setUseSpiceOmegaFlag(False)  # Use manual value

    # Setup planet position (non-zero to test relative position calculation)
    planetPos = np.array([1.0e6, 2.0e6, -5.0e5])  # [m]
    planetMsg = messaging.SpicePlanetStateMsgPayload()
    planetMsg.PositionVector = planetPos
    plMsg = messaging.SpicePlanetStateMsg().write(planetMsg)
    windModel.planetPosInMsg.subscribeTo(plMsg)

    # Setup spacecraft position
    r_BN_N = np.array([6_578_000.0, 0.0, 0.0]) + planetPos  # [m]
    scMsg = messaging.SCStatesMsgPayload()
    scMsg.r_BN_N = r_BN_N
    scStateMsg = messaging.SCStatesMsg().write(scMsg)
    windModel.addSpacecraftToModel(scStateMsg)

    # Setup data logging
    dataLog = windModel.envOutMsgs[0].recorder()
    sim.AddModelToTask(unitTaskName, windModel)
    sim.AddModelToTask(unitTaskName, dataLog)

    # Run simulation
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify results
    windData = dataLog.v_air_N
    windPerturbation = dataLog.v_wind_N

    # Calculate expected values
    r_BP_N = r_BN_N - planetPos  # Relative position
    expected_v_air_N = np.cross(omega_planet, r_BP_N)
    expected_v_wind_N = np.array([0.0, 0.0, 0.0])  # [m/s] Zero wind perturbation

    # Verify that v_air_N equals co-rotating velocity
    assert np.allclose(windData[-1], expected_v_air_N, atol=1.0e-8), \
        f"v_air_N mismatch: got {windData[-1]}, expected {expected_v_air_N}"

    # Verify that v_wind_N is zero (no additional wind)
    assert np.allclose(windPerturbation[-1], expected_v_wind_N, atol=1.0e-8), \
        f"v_wind_N should be zero: got {windPerturbation[-1]}"

    # Verify relationship: v_air_N = v_corotating + v_wind_N
    assert np.allclose(windData[-1], windPerturbation[-1] + expected_v_air_N, atol=1.0e-8), \
        "v_air_N should equal v_corotating + v_wind_N"


def test_windBase_spice_integration():
    """Test that WindBase correctly handles SPICE-derived planet angular velocity."""

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    # Create ZeroWindModel (which inherits from WindBase)
    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "zeroWind"
    windModel.setUseSpiceOmegaFlag(True)  # Enable SPICE mode

    # Setup SPICE with custom angular velocity
    spice_omega = np.array([0.0, 0.0, 1.5e-4])  # [rad/s]
    planetMsg = messaging.SpicePlanetStateMsgPayload()
    planetMsg.PositionVector = np.array([0.0, 0.0, 0.0])  # [m]
    planetMsg.J20002Pfix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    planetMsg.J20002Pfix_dot = [
        [0.0,              spice_omega[2], 0.0],
        [-spice_omega[2], 0.0,            0.0],
        [0.0,              0.0,            0.0],
    ]
    plMsg = messaging.SpicePlanetStateMsg().write(planetMsg)
    windModel.planetPosInMsg.subscribeTo(plMsg)

    # Setup spacecraft
    r_BN_N = np.array([6_578_000.0, 0.0, 0.0])  # [m]
    scMsg = messaging.SCStatesMsgPayload()
    scMsg.r_BN_N = r_BN_N
    scStateMsg = messaging.SCStatesMsg().write(scMsg)
    windModel.addSpacecraftToModel(scStateMsg)

    # Setup data logging
    dataLog = windModel.envOutMsgs[0].recorder()
    sim.AddModelToTask(unitTaskName, windModel)
    sim.AddModelToTask(unitTaskName, dataLog)

    # Run simulation
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify that SPICE-derived omega was used
    windData = dataLog.v_air_N
    expected_v_air_N = np.cross(spice_omega, r_BN_N)

    assert np.allclose(windData[-1], expected_v_air_N, atol=1.0e-8), \
        f"Should use SPICE-derived omega: got {windData[-1]}, expected {expected_v_air_N}"


def test_windBase_spice_zero_rotation():
    """Test that WindBase correctly outputs zero omega when SPICE C_dot is zero (non-rotating planet)."""

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    # Create ZeroWindModel
    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "zeroWind"
    windModel.setUseSpiceOmegaFlag(True)  # Enable SPICE mode

    # Setup SPICE with zero derivatives (non-rotating planet)
    planetMsg = messaging.SpicePlanetStateMsgPayload()
    planetMsg.PositionVector = np.array([0.0, 0.0, 0.0])  # [m]
    planetMsg.J20002Pfix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    planetMsg.J20002Pfix_dot = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]
    plMsg = messaging.SpicePlanetStateMsg().write(planetMsg)
    windModel.planetPosInMsg.subscribeTo(plMsg)

    # Setup spacecraft
    r_BN_N = np.array([6_578_000.0, 0.0, 0.0])  # [m]
    scMsg = messaging.SCStatesMsgPayload()
    scMsg.r_BN_N = r_BN_N
    scStateMsg = messaging.SCStatesMsg().write(scMsg)
    windModel.addSpacecraftToModel(scStateMsg)

    # Setup data logging
    dataLog = windModel.envOutMsgs[0].recorder()
    sim.AddModelToTask(unitTaskName, windModel)
    sim.AddModelToTask(unitTaskName, dataLog)

    # Run simulation
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Zero C_dot must produce zero SPICE omega — no stale or fallback value
    windData = dataLog.v_air_N
    assert np.allclose(windData[-1], np.zeros(3), atol=1.0e-8), \
        f"Zero C_dot should yield zero wind: got {windData[-1]}"


def test_zeroWindModel_default_earth_rotation():
    """Test that ZeroWindModel uses default Earth rotation rate when SPICE mode is disabled."""

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    # Create ZeroWindModel with default settings
    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "zeroWind"
    windModel.setUseSpiceOmegaFlag(False)  # Use manual default (Earth rotation)

    # Setup planet (no SPICE derivatives, so should use default)
    planetMsg = messaging.SpicePlanetStateMsgPayload()
    planetMsg.PositionVector = np.array([0.0, 0.0, 0.0])  # [m]
    planetMsg.J20002Pfix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    planetMsg.J20002Pfix_dot = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]
    plMsg = messaging.SpicePlanetStateMsg().write(planetMsg)
    windModel.planetPosInMsg.subscribeTo(plMsg)

    # Setup spacecraft
    r_BN_N = np.array([6_578_000.0, 0.0, 0.0])  # [m]
    scMsg = messaging.SCStatesMsgPayload()
    scMsg.r_BN_N = r_BN_N
    scStateMsg = messaging.SCStatesMsg().write(scMsg)
    windModel.addSpacecraftToModel(scStateMsg)

    # Setup data logging
    dataLog = windModel.envOutMsgs[0].recorder()
    sim.AddModelToTask(unitTaskName, windModel)
    sim.AddModelToTask(unitTaskName, dataLog)

    # Run simulation
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify that default Earth rotation was used
    windData = dataLog.v_air_N
    expected_omega = np.array([0.0, 0.0, orbitalMotion.OMEGA_EARTH])
    expected_v_air_N = np.cross(expected_omega, r_BN_N)

    assert np.allclose(windData[-1], expected_v_air_N, atol=1.0e-8), \
        f"Should use default Earth rotation: got {windData[-1]}, expected {expected_v_air_N}"


def test_zeroWindModel_multiple_spacecraft():
    """Test that ZeroWindModel correctly handles multiple spacecraft."""

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    # Create ZeroWindModel
    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "zeroWind"

    # Set planet angular velocity
    omega_planet = np.array([0.0, 0.0, 1.0e-4])  # [rad/s]
    windModel.setPlanetOmega_N(omega_planet)
    windModel.setUseSpiceOmegaFlag(False)

    # Setup planet
    planetPos = np.array([1.0e6, 0.0, 0.0])  # [m]
    planetMsg = messaging.SpicePlanetStateMsgPayload()
    planetMsg.PositionVector = planetPos
    plMsg = messaging.SpicePlanetStateMsg().write(planetMsg)
    windModel.planetPosInMsg.subscribeTo(plMsg)

    # Setup multiple spacecraft at different positions
    spacecraft_positions = [
        np.array([6_578_000.0, 0.0, 0.0]) + planetPos,  # [m]
        np.array([6_578_000.0, 100_000.0, 50_000.0]) + planetPos,  # [m]
        np.array([6_578_000.0, -50_000.0, -25_000.0]) + planetPos,  # [m]
    ]

    # Create spacecraft messages and model subscriptions in separate steps
    # This avoids the race condition that occurs when creating and adding in one step
    scStateMsgs = []
    for r_BN_N in spacecraft_positions:
        scMsg = messaging.SCStatesMsgPayload()
        scMsg.r_BN_N = r_BN_N
        scStateMsg = messaging.SCStatesMsg().write(scMsg)
        scStateMsgs.append(scStateMsg)

    # Now add each spacecraft individually to the model using addSpacecraftToModel
    for scStateMsg in scStateMsgs:
        windModel.addSpacecraftToModel(scStateMsg)

    # Add wind model to task first to ensure messages are written before logging
    sim.AddModelToTask(unitTaskName, windModel)

    # Then add data loggers
    dataLogs = []
    for i in range(len(scStateMsgs)):
        dataLog = windModel.envOutMsgs[i].recorder()
        dataLogs.append(dataLog)
        sim.AddModelToTask(unitTaskName, dataLog)

    # Run simulation
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify results for each spacecraft
    for i, (dataLog, r_BN_N) in enumerate(zip(dataLogs, spacecraft_positions)):
        windData = dataLog.v_air_N
        windPerturbation = dataLog.v_wind_N

        # Calculate expected values
        r_BP_N = r_BN_N - planetPos
        expected_v_air_N = np.cross(omega_planet, r_BP_N)
        expected_v_wind_N = np.array([0.0, 0.0, 0.0])  # [m/s]

        assert np.allclose(windData[-1], expected_v_air_N, atol=1.0e-8), \
            f"Spacecraft {i}: v_air_N mismatch: got {windData[-1]}, expected {expected_v_air_N}"

        assert np.allclose(windPerturbation[-1], expected_v_wind_N, atol=1.0e-8), \
            f"Spacecraft {i}: v_wind_N should be zero: got {windPerturbation[-1]}"


def test_windBase_error_handling():
    """Test that WindBase properly handles error conditions."""

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    # Create ZeroWindModel
    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "zeroWind"

    # Setup spacecraft but don't connect planet message
    r_BN_N = np.array([6_578_000.0, 0.0, 0.0])
    scMsg = messaging.SCStatesMsgPayload()
    scMsg.r_BN_N = r_BN_N
    scStateMsg = messaging.SCStatesMsg().write(scMsg)
    windModel.addSpacecraftToModel(scStateMsg)

    # Don't connect planetPosInMsg - this should cause an error during Reset

    sim.AddModelToTask(unitTaskName, windModel)

    # Run simulation - should fail during initialization
    with pytest.raises(Exception):
        sim.InitializeSimulation()


def test_windBase_planet_at_origin():
    """Test that WindBase correctly handles planet at origin (0,0,0)."""

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    # Create ZeroWindModel
    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "zeroWind"

    # Set planet angular velocity
    omega_planet = np.array([0.0, 0.0, 1.0e-4])  # [rad/s]
    windModel.setPlanetOmega_N(omega_planet)
    windModel.setUseSpiceOmegaFlag(False)

    # Setup planet at origin
    planetPos = np.array([0.0, 0.0, 0.0])  # [m]
    planetMsg = messaging.SpicePlanetStateMsgPayload()
    planetMsg.PositionVector = planetPos
    plMsg = messaging.SpicePlanetStateMsg().write(planetMsg)
    windModel.planetPosInMsg.subscribeTo(plMsg)

    # Setup spacecraft
    r_BN_N = np.array([6_578_000.0, 0.0, 0.0])  # [m] Spacecraft at (6578000, 0, 0), planet at (0,0,0)
    scMsg = messaging.SCStatesMsgPayload()
    scMsg.r_BN_N = r_BN_N
    scStateMsg = messaging.SCStatesMsg().write(scMsg)
    windModel.addSpacecraftToModel(scStateMsg)

    # Setup data logging
    dataLog = windModel.envOutMsgs[0].recorder()
    sim.AddModelToTask(unitTaskName, windModel)
    sim.AddModelToTask(unitTaskName, dataLog)

    # Run simulation
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify results
    windData = dataLog.v_air_N

    # Calculate expected values: r_BP_N = r_BN_N - planetPos = r_BN_N - [0,0,0] = r_BN_N
    r_BP_N = r_BN_N - planetPos
    expected_v_air_N = np.cross(omega_planet, r_BP_N)

    assert np.allclose(windData[-1], expected_v_air_N, atol=1.0e-8), \
        f"Planet at origin: v_air_N mismatch: got {windData[-1]}, expected {expected_v_air_N}"


def test_windBase_zero_angular_velocity():
    """Test that WindBase correctly handles zero angular velocity."""

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    # Create ZeroWindModel
    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "zeroWind"

    # Set zero angular velocity
    omega_zero = np.array([0.0, 0.0, 0.0])  # [rad/s]
    windModel.setPlanetOmega_N(omega_zero)
    windModel.setUseSpiceOmegaFlag(False)

    # Setup planet
    planetPos = np.array([1.0e6, 0.0, 0.0])  # [m]
    planetMsg = messaging.SpicePlanetStateMsgPayload()
    planetMsg.PositionVector = planetPos
    plMsg = messaging.SpicePlanetStateMsg().write(planetMsg)
    windModel.planetPosInMsg.subscribeTo(plMsg)

    # Setup spacecraft
    r_BN_N = np.array([6_578_000.0, 0.0, 0.0]) + planetPos  # [m]
    scMsg = messaging.SCStatesMsgPayload()
    scMsg.r_BN_N = r_BN_N
    scStateMsg = messaging.SCStatesMsg().write(scMsg)
    windModel.addSpacecraftToModel(scStateMsg)

    # Setup data logging
    dataLog = windModel.envOutMsgs[0].recorder()
    sim.AddModelToTask(unitTaskName, windModel)
    sim.AddModelToTask(unitTaskName, dataLog)

    # Run simulation
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify results
    windData = dataLog.v_air_N
    windPerturbation = dataLog.v_wind_N

    # With zero angular velocity, both should be zero
    expected_v_air_N = np.array([0.0, 0.0, 0.0])
    expected_v_wind_N = np.array([0.0, 0.0, 0.0])

    assert np.allclose(windData[-1], expected_v_air_N, atol=1.0e-8), \
        f"Zero angular velocity: v_air_N should be zero: got {windData[-1]}"

    assert np.allclose(windPerturbation[-1], expected_v_wind_N, atol=1.0e-8), \
        f"Zero angular velocity: v_wind_N should be zero: got {windPerturbation[-1]}"


def test_windBase_set_get_omega():
    """Test that setPlanetOmega_N and getPlanetOmega_N work correctly."""

    windModel = zeroWindModel.ZeroWindModel()

    # Test default value
    default_omega = np.array(windModel.getPlanetOmega_N()).flatten()
    expected_default = np.array([0.0, 0.0, orbitalMotion.OMEGA_EARTH])
    assert np.allclose(default_omega, expected_default, atol=1.0e-8), \
        f"Default omega mismatch: got {default_omega}, expected {expected_default}"

    # Test custom value round-trip
    custom_omega = np.array([1.0e-5, 2.0e-5, 3.0e-5])  # [rad/s]
    windModel.setPlanetOmega_N(custom_omega)
    retrieved_omega = np.array(windModel.getPlanetOmega_N()).flatten()
    assert np.allclose(retrieved_omega, custom_omega, atol=1.0e-8), \
        f"Round-trip omega mismatch: set {custom_omega}, got {retrieved_omega}"


def test_windBase_spice_mode_switching():
    """Test switching between SPICE and manual omega modes."""

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    # Create ZeroWindModel
    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "zeroWind"

    # Setup SPICE with custom angular velocity
    spice_omega = np.array([0.0, 0.0, 1.0e-4])
    planetMsg = messaging.SpicePlanetStateMsgPayload()
    planetMsg.PositionVector = np.array([0.0, 0.0, 0.0])
    planetMsg.J20002Pfix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    planetMsg.J20002Pfix_dot = [
        [0.0,              spice_omega[2], 0.0],
        [-spice_omega[2], 0.0,            0.0],
        [0.0,              0.0,            0.0],
    ]
    plMsg = messaging.SpicePlanetStateMsg().write(planetMsg)
    windModel.planetPosInMsg.subscribeTo(plMsg)

    # Setup spacecraft
    r_BN_N = np.array([6_578_000.0, 0.0, 0.0])
    scMsg = messaging.SCStatesMsgPayload()
    scMsg.r_BN_N = r_BN_N
    scStateMsg = messaging.SCStatesMsg().write(scMsg)
    windModel.addSpacecraftToModel(scStateMsg)

    # Test 1: Manual mode (should ignore SPICE)
    manual_omega = np.array([0.0, 0.0, 2.0e-4])
    windModel.setPlanetOmega_N(manual_omega)
    windModel.setUseSpiceOmegaFlag(False)  # Force manual mode

    dataLog1 = windModel.envOutMsgs[0].recorder()
    sim.AddModelToTask(unitTaskName, windModel)
    sim.AddModelToTask(unitTaskName, dataLog1)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    wind_manual = dataLog1.v_air_N[-1]
    expected_manual = np.cross(manual_omega, r_BN_N)
    assert np.allclose(wind_manual, expected_manual, atol=1.0e-8), \
        f"Manual mode: got {wind_manual}, expected {expected_manual}"

    # Test 2: SPICE mode (should use SPICE value)
    sim2 = SimulationBaseClass.SimBaseClass()
    testProc2 = sim2.CreateNewProcess(unitProcessName)
    testProc2.addTask(sim2.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    windModel2 = zeroWindModel.ZeroWindModel()
    windModel2.ModelTag = "zeroWind2"

    # Set a different manual omega (should be ignored in SPICE mode)
    fallback_omega = np.array([0.0, 0.0, 3.0e-4])  # [rad/s]
    windModel2.setPlanetOmega_N(fallback_omega)
    windModel2.setUseSpiceOmegaFlag(True)  # Enable SPICE mode

    # Connect same planet message (with SPICE derivatives)
    windModel2.planetPosInMsg.subscribeTo(plMsg)

    # Connect same spacecraft message
    windModel2.addSpacecraftToModel(scStateMsg)

    dataLog2 = windModel2.envOutMsgs[0].recorder()
    sim2.AddModelToTask(unitTaskName, windModel2)
    sim2.AddModelToTask(unitTaskName, dataLog2)

    sim2.InitializeSimulation()
    sim2.ConfigureStopTime(macros.sec2nano(1.0))
    sim2.ExecuteSimulation()

    wind_spice = dataLog2.v_air_N[-1]
    expected_spice = np.cross(spice_omega, r_BN_N)
    assert np.allclose(wind_spice, expected_spice, atol=1.0e-8), \
        f"SPICE mode: got {wind_spice}, expected {expected_spice}"


def test_windBase_unwritten_messages():
    """Test that wind output is zero when input messages are not written."""

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "zeroWind"

    # Link messages but don't write to them
    windModel.planetPosInMsg.subscribeTo(messaging.SpicePlanetStateMsg())
    windModel.addSpacecraftToModel(messaging.SCStatesMsg())

    dataLog = windModel.envOutMsgs[0].recorder()
    sim.AddModelToTask(unitTaskName, windModel)
    sim.AddModelToTask(unitTaskName, dataLog)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    wind = dataLog.v_air_N[-1]
    expected = np.zeros(3)
    assert np.allclose(wind, expected, atol=1.0e-12), \
        f"Expected zero wind when messages unwritten, got {wind}"


def test_windBase_input_validation():
    """Test that WindBase validates input for NaN and infinite values."""

    import numpy as np

    # NaN input should trigger BSK_ERROR during simulation initialization
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess(unitProcessName)
    testProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    windModel_nan = zeroWindModel.ZeroWindModel()
    windModel_nan.ModelTag = "testWindNaN"

    # Setup basic connections
    from Basilisk.architecture import messaging
    planetMsg = messaging.SpicePlanetStateMsgPayload()
    planetMsg.PositionVector = np.array([0.0, 0.0, 0.0])
    plMsg = messaging.SpicePlanetStateMsg().write(planetMsg)
    windModel_nan.planetPosInMsg.subscribeTo(plMsg)

    scMsg = messaging.SCStatesMsgPayload()
    scMsg.r_BN_N = np.array([6_578_000.0, 0.0, 0.0])
    scStateMsg = messaging.SCStatesMsg().write(scMsg)
    windModel_nan.addSpacecraftToModel(scStateMsg)

    # Set NaN omega - this should trigger BSK_ERROR
    nan_omega = np.array([0.0, np.nan, 0.0])
    with pytest.raises(Exception):
        windModel_nan.setPlanetOmega_N(nan_omega)

    # Inf input should also trigger BSK_ERROR
    sim2 = SimulationBaseClass.SimBaseClass()
    testProc2 = sim2.CreateNewProcess(unitProcessName)
    testProc2.addTask(sim2.CreateNewTask(unitTaskName, macros.sec2nano(1.0)))

    windModel_inf = zeroWindModel.ZeroWindModel()
    windModel_inf.ModelTag = "testWindInf"

    # Setup connections
    windModel_inf.planetPosInMsg.subscribeTo(plMsg)
    windModel_inf.addSpacecraftToModel(scStateMsg)

    # Set inf omega - this should trigger BSK_ERROR during Reset
    inf_omega = np.array([0.0, np.inf, 0.0])
    with pytest.raises(Exception):
        windModel_inf.setPlanetOmega_N(inf_omega)


if __name__ == "__main__":
    test_windBase_spice_integration()
    test_windBase_spice_zero_rotation()
    test_windBase_error_handling()
    test_windBase_planet_at_origin()
    test_windBase_zero_angular_velocity()
    test_windBase_set_get_omega()
    test_windBase_spice_mode_switching()
    test_windBase_unwritten_messages()
    test_windBase_input_validation()
    test_zeroWindModel_basic_functionality()
    test_zeroWindModel_default_earth_rotation()
    test_zeroWindModel_multiple_spacecraft()
    print("All tests passed!")
