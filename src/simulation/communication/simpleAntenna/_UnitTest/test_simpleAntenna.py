#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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
import numpy as np
import pytest

from Basilisk import __path__
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.utilities import macros, orbitalMotion
from Basilisk.simulation import simpleAntenna
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

bskPath = __path__[0]

# Constants
K_BOLTZMANN = 1.38064852e-23
ACCURACY = 1e-6

def earth_horizon_sigma():
    theta = np.arcsin(
        (orbitalMotion.REQ_EARTH * 1e3)
        / ((800 + orbitalMotion.REQ_EARTH) * 1e3)
    )
    return rbk.C2MRP(
        np.array([
            [0, 0, 1],
            [np.sin(theta), np.cos(theta), 0],
            [-np.cos(theta), np.sin(theta), 0],
        ])
    )

@pytest.mark.parametrize("ACCURACY", [1e-6])
@pytest.mark.parametrize(
    "antennaState, env_type, T_AmbientSet, useHaslamMap, T_sky_true, eclipseValue, sigma_AB, scPosUnit",
    [
        (simpleAntenna.ANTENNA_OFF, 0, 150, False, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RX,  0, 150, False, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_TX,  0, 150, False, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  1, 150, False, 200, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  0, None, False, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  1, None, False, 200, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  0, 150, True, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 300.0, 0.0, rbk.C2MRP(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])), [1, 0, 0]), # Case with pointing Earth and sun blocked
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 300.0, 1.0, rbk.C2MRP(np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])), [0, 1, 0]), # Case with pointing Earth and sun is visible but not in the cone
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 5.660060787285246, 1.0, rbk.C2MRP(np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])), [1, 0, 0]), # Case with pointing Moon
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 37.82128898104384, 1.0, rbk.C2MRP(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])), [-1, 0, 0]), # Case with pointing Sun and sun in the cone
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 147.9662263230431, 0.0, earth_horizon_sigma(), [1, 0, 0]), # Case with pointing Earth with partial coverage
    ]
)

def test_simpleAntenna(antennaState, env_type, T_AmbientSet, useHaslamMap, T_sky_true, eclipseValue, sigma_AB, scPosUnit, ACCURACY):
    r"""
    **Validation Test Description**

    This unit test validates the thermal noise temperature and sky contribution
    computed by the ``simpleAntenna`` module under a variety of antenna operating
    modes, environmental configurations, and pointing geometries. The test
    verifies correct behavior when the antenna is transmitting, receiving,
    both transmitting and receiving, or turned off, as well as when different
    celestial bodies (Earth, Sun, Moon) are inside or outside the antenna field
    of view.

    The test also exercises eclipse handling, ambient temperature settings,
    and antenna orientation effects using direction cosine matrices.

    **Test Parameters**

    The test is fully parameterized to cover multiple combinations of antenna
    state, environment type, ambient temperature configuration, and antenna
    pointing geometry.

    Args:
        antennaState (int):
            Antenna operational mode. One of
            ``ANTENNA_OFF``, ``ANTENNA_RX``, ``ANTENNA_TX``,
            or ``ANTENNA_RXTX``.

        env_type (int):
            Environment selection flag. This controls where antenna is attached:
            - 0: Spacecraft environment
            - 1: Ground station environment

        T_AmbientSet (float or None):
            Ambient antenna temperature in Kelvin. If ``None``, the module
            computes the ambient temperature internally using environmental
            models.

        useHaslamMap (bool):
            Flag indicating whether the Haslam sky map is used to model
            galactic background noise.

        T_sky_true (float):
            Truth value for the expected sky noise temperature (K) used
            for validation.

        eclipseValue (float):
            Eclipse factor in the range [0, 1], where 0 indicates full eclipse
            (Sun completely blocked) and 1 indicates no eclipse.

        sigma_AB (array-like or ``Eigen::MRP``):
            Antenna orientation relative to the spacecraft body frame,
            expressed as Modified Rodrigues Parameters (MRPs). This defines
            the antenna boresight direction used to determine which celestial
            bodies fall within the antenna cone.

        scPosUnit (array-like, shape (3,)):
            Unit vector of the spacecraft position expressed in the inertial
            frame. This is used to define the relative geometry between the
            spacecraft, Earth, Sun, and Moon.

        ACCURACY (float):
            Absolute numerical tolerance used when comparing the computed
            sky temperature against the truth value.

    **Description of Variables Being Tested**

    This test validates:
    - Correct handling of antenna operational modes (OFF, RX, TX, RXTX)
    - Ground station vs. spacecraft environment configurations
    - Proper eclipse handling when the Sun is partially or fully blocked
    - Correct use of antenna pointing geometry via DCM/MRP transformations
    - Robust behavior when ambient temperature is user-specified or internally computed
    - Numerical ACCURACY of sky temperature computation within the specified tolerance
    """
    [testResults, testMessage, _] = simpleAntennaTestFunction(antennaState, env_type, T_AmbientSet, useHaslamMap, T_sky_true, eclipseValue, sigma_AB, scPosUnit, ACCURACY)
    assert testResults < 1, testMessage

def test_simpleAntenna_haslamMap_three_runs_and_pointing_difference():
    """
    Haslam map coverage:
    Run one sim instance and change pointing while Haslam is enabled.
    Verify that sky temperature changes with pointing direction.
    """
    # --- Fixed configuration ---
    antennaState = simpleAntenna.ANTENNA_RXTX
    env_type = 0
    T_AmbientSet = 150.0
    eclipseValue = 1.0
    scPosUnit = [1, 0, 0]   # consistent with other tests
    frequency = 2.2e9
    bandwidth = 5e6
    T_E = 50.0
    eta_r = 0.6
    useHaslamMap = True

    # Direction A: identity (boresight along +Z in body frame as you use [0,0,1] for antenna position)
    sigma_AB_A = [0, 0, 0]
    # Direction B: rotate so body +Z maps to +Y (same matrix you used)
    sigma_AB_B = rbk.C2MRP(np.array([[1, 0, 0],
                                     [0, 0, 1],
                                     [0, -1, 0]]))

    # --- 1) Setup ONE simulation instance ---
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTaskName = "unitTaskHaslam"
    unitProcessName = "TestProcessHaslam"
    testProcessRate = macros.sec2nano(0.5)

    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = simpleAntenna.SimpleAntenna()
    module.ModelTag = "simpleAntennaHaslamTag"
    module.setAntennaName("simpleAntenna")

    # --- 2) Messages (same style as your main test) ---
    # Sun
    sunMessage = messaging.SpicePlanetStateMsgPayload()
    sunMessage.PlanetName = "sun"
    sunMessage.PositionVector = [0, 0, 0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunMessage)

    # Earth
    earthMessage = messaging.SpicePlanetStateMsgPayload()
    earthMessage.PlanetName = "earth"
    earthMessage.PositionVector = [orbitalMotion.AU * 1000, 0, 0]
    earthMsg = messaging.SpicePlanetStateMsg().write(earthMessage)

    # Moon
    moonMessage = messaging.SpicePlanetStateMsgPayload()
    moonMessage.PlanetName = "moon"
    moonMessage.PositionVector = [(orbitalMotion.AU + orbitalMotion.SMA_MOON) * 1000, 0, 0]
    moonMsg = messaging.SpicePlanetStateMsg().write(moonMessage)

    # Eclipse
    eclipseMessage = messaging.EclipseMsgPayload()
    eclipseMessage.illuminationFactor = eclipseValue
    eclipseMsg = messaging.EclipseMsg().write(eclipseMessage)

    # Antenna state
    antennaStateData = messaging.AntennaStateMsgPayload()
    antennaStateData.antennaState = antennaState
    antennaStateMsg = messaging.AntennaStateMsg().write(antennaStateData)

    # Spacecraft state
    scMessage = messaging.SCStatesMsgPayload()
    altitude = 800 * 1000  # [m]
    scPos = np.array(scPosUnit) * (altitude + orbitalMotion.REQ_EARTH * 1e3)
    scMessage.r_BN_N = [orbitalMotion.AU * 1000, 0, 0] + scPos
    scMessage.sigma_BN = [0, 0, 0]
    scMsg = messaging.SCStatesMsg().write(scMessage)

    # Subscribe
    module.scStateInMsg.subscribeTo(scMsg)
    module.antennaSetStateInMsg.subscribeTo(antennaStateMsg)
    module.sunInMsg.subscribeTo(sunMsg)
    module.sunEclipseInMsg.subscribeTo(eclipseMsg)
    module.addPlanetToModel(earthMsg)
    module.addPlanetToModel(moonMsg)

    # --- 3) Module parameters ---
    module.setAntennaFrequency(frequency)
    module.setAntennaBandwidth(bandwidth)
    module.setAntennaDirectivity_dB(20.0)
    module.setAntennaHpbwRatio(1.0)
    module.setAntennaP_Tx(100.0)
    module.setAntennaP_Rx(10.0)
    module.setAntennaEquivalentNoiseTemp(T_E)
    module.setAntennaRadEfficiency(eta_r)
    module.setAntennaPositionBodyFrame([0, 0, 1])
    module.setAntennaEnvironmentTemperature(T_AmbientSet)

    # Turn Haslam ON for this test (do not rely on the helper)
    module.setUseHaslamMap(useHaslamMap)

    # Add to task + recorder
    unitTestSim.AddModelToTask(unitTaskName, module)
    dataLog = module.antennaOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    def tsky_from_last_log(): #reverse engineer the sky temperature from the noise
        """Infer sky temperature from last logged noise power."""
        p_noise = float(dataLog.P_N[-1])
        T_S_sim = p_noise / (K_BOLTZMANN * bandwidth)
        return (T_S_sim - T_E - (1.0 - eta_r) * T_AmbientSet) / eta_r

    # --- RUN A: Direction A ---
    module.setAntennaOrientationBodyFrame(sigma_AB_A)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))
    unitTestSim.ExecuteSimulation()
    Tsky_on_A = tsky_from_last_log()

    # --- RUN B: Direction B (same sim, continue)
    module.setAntennaOrientationBodyFrame(sigma_AB_B)
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()
    Tsky_on_B = tsky_from_last_log()

    # --- RUN C: Direction A again to test repeatability in the same run
    module.setAntennaOrientationBodyFrame(sigma_AB_A)
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.5))
    unitTestSim.ExecuteSimulation()
    Tsky_on_A2 = tsky_from_last_log()

    # --- Prints before asserts (as requested) ---
    print(f"Tsky_on_A  = {Tsky_on_A}")
    print(f"Tsky_on_B  = {Tsky_on_B}")
    print(f"Tsky_on_A2 = {Tsky_on_A2}")

    # --- Assertions ---
    # Haslam should produce sky temps above CMB baseline in most directions
    assert Tsky_on_A > 2.7
    assert Tsky_on_B > 2.7

    # Make sure pointing changes the value by a noticeable amount
    assert abs(Tsky_on_A - Tsky_on_B) > 0.1
    # Make sure same inertial pointing as before but different timestep has similar temperature
    assert abs(Tsky_on_A - Tsky_on_A2) < 0.1

def simpleAntennaTestFunction(antennaState, env_type, T_AmbientSet, useHaslamMap, T_sky_true, eclipseValue, sigma_AB, scPosUnit, ACCURACY):
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Set up module to be tested
    module = simpleAntenna.SimpleAntenna()
    module.ModelTag = "simpleAntennaTag"
    module.setAntennaName("simpleAntenna")
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Set up Sun message
    sunMessage = messaging.SpicePlanetStateMsgPayload()
    sunMessage.PlanetName = "sun"
    sunMessage.PositionVector = [0, 0, 0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunMessage)

    # Set up Earth message
    earthMessage = messaging.SpicePlanetStateMsgPayload()
    earthMessage.PlanetName = "earth"
    earthMessage.PositionVector = [orbitalMotion.AU*1000, 0, 0]
    earthMsg = messaging.SpicePlanetStateMsg().write(earthMessage)

    # Set up Moon message
    moonMessage = messaging.SpicePlanetStateMsgPayload()
    moonMessage.PlanetName = "moon"
    moonMessage.PositionVector = [(orbitalMotion.AU+orbitalMotion.SMA_MOON)*1000, 0, 0]
    moonMsg = messaging.SpicePlanetStateMsg().write(moonMessage)

    # Set up eclipse message
    eclipseMessage = messaging.EclipseMsgPayload()
    eclipseMessage.illuminationFactor = eclipseValue  # Set it to be totally in shadow
    eclipseMsg = messaging.EclipseMsg().write(eclipseMessage)

    # Set up antenna state message (ANTENNA_OFF = 0, ANTENNA_RX = 1, ANTENNA_TX = 2, ANTENNA_RXTX = 3)
    antennaStateData = messaging.AntennaStateMsgPayload()
    antennaStateData.antennaState = antennaState
    antennaStateMsg = messaging.AntennaStateMsg().write(antennaStateData)

    # subscribe input messages to module
    if env_type == 0: # space environment
        # Set up spacecraft message
        scMessage = messaging.SCStatesMsgPayload()
        altitude = 800 * 1000  # [m] 800 km altitude
        scPos = np.array(scPosUnit) * (altitude + orbitalMotion.REQ_EARTH*1e3)
        scMessage.r_BN_N = [orbitalMotion.AU*1000, 0, 0] + scPos  # [m] position of spacecraft in inertial frame
        scMessage.sigma_BN = [0,0,0]
        scMsg = messaging.SCStatesMsg().write(scMessage)
        module.scStateInMsg.subscribeTo(scMsg)
    elif env_type == 1: # ground station environment
        # Set up ground station message
        groundMessage = messaging.GroundStateMsgPayload()
        groundMessage.r_LN_N = [orbitalMotion.AU *1000, 0, 0]
        groundMessage.r_LP_N = [orbitalMotion.REQ_EARTH*1000, 0, 0]
        groundMsg = messaging.GroundStateMsg().write(groundMessage)
        module.groundStateInMsg.subscribeTo(groundMsg)
    else:
        testFailCount += 1
        testMessages.append("Error: invalid env_type")

    module.antennaSetStateInMsg.subscribeTo(antennaStateMsg)
    module.sunInMsg.subscribeTo(sunMsg)
    module.addPlanetToModel(earthMsg)
    module.addPlanetToModel(moonMsg)
    module.sunEclipseInMsg.subscribeTo(eclipseMsg)

    # use setters to set module parameters
    frequency = 2.2e9       # Hz
    module.setAntennaFrequency(frequency)
    bandwidth = 5e6                  # Hz
    module.setAntennaBandwidth(bandwidth)
    directivity = 20.0              # dB
    module.setAntennaDirectivity_dB(directivity)
    hpbwRatio = 1.0                 # [-]
    module.setAntennaHpbwRatio(hpbwRatio)
    P_Tx = 100.0                    # W
    module.setAntennaP_Tx(P_Tx)
    P_Rx = 10.0                      # W
    module.setAntennaP_Rx(P_Rx)
    T_E = 50.0           # K
    module.setAntennaEquivalentNoiseTemp(T_E)
    eta_r = 0.6          # [-]
    module.setAntennaRadEfficiency(eta_r)
    module.setAntennaPositionBodyFrame([0, 0, 1]) # m
    module.setAntennaOrientationBodyFrame(sigma_AB)
    module.setUseHaslamMap(useHaslamMap) # Try both true and false

    if  T_AmbientSet is not None:
        T_Ambient = T_AmbientSet
        module.setAntennaEnvironmentTemperature(T_Ambient)
    else:
        if env_type == 0: # space environment
            T_Ambient = 150  # K
        elif env_type == 1: # ground station environment
            T_ground = 288.15 # [K]
            T_Ambient = T_ground
    # setup output message recorder objects
    dataLog = module.antennaOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    t_sky_test = module.calculateTsky()
    print('t_sky_test', t_sky_test)

    # compute truth values
    dir_lin  = pow(10, directivity / 10)
    HPBW_el  = np.sqrt((np.log(2) * 16) / (dir_lin * hpbwRatio))
    HPBW_az  = hpbwRatio * HPBW_el
    n = len(dataLog.HPBW_el)

    T_ant = (1.0 - eta_r) * T_Ambient + eta_r * T_sky_true
    T_S = T_E + T_ant
    G_TN = directivity + 10.0 * np.log10(eta_r) - 10.0 * np.log10(T_S)
    P_N = K_BOLTZMANN * T_S * bandwidth
    P_eirp = 10 * np.log10(P_Tx) + 10 * np.log10(eta_r) + directivity

    # Compute expected location and attitude values
    if env_type == 0:
        r_AP_N = [0.0, 0.0, 0.0]
        r_BN_N = [orbitalMotion.AU*1000, 0, 0] + scPos  # [m] position of spacecraft in inertial frame
        sigma_BN = [0,0,0]
        dcm_BN = rbk.MRP2C(sigma_BN)
        r_AB_N = dcm_BN.T @ [0, 0, 1]
        r_AN_N = r_BN_N + r_AB_N
        dcm_AB = rbk.MRP2C(sigma_AB)
        dcm_AN = dcm_AB @ dcm_BN
        sigma_AN = rbk.C2MRP(dcm_AN)
    elif env_type == 1:
        r_AN_N = [orbitalMotion.AU *1000, 0, 0]
        r_AP_N = [orbitalMotion.REQ_EARTH*1000, 0, 0]
        sigma_AN = [0, 0, 0]

    # check results
    checks = [
        (dataLog.environment, env_type, "environment"),
        (dataLog.antennaState, antennaState, "antennaState"),
        (dataLog.frequency, frequency, "frequency"),
        (dataLog.B, bandwidth, "bandwidth"),
        (dataLog.HPBW_el, HPBW_el, "HPBW_el"),
        (dataLog.HPBW_az, HPBW_az, "HPBW_az"),
        (dataLog.P_Tx,    P_Tx,    "P_Tx"),
        (dataLog.P_Rx,    P_Rx,    "P_Rx"),
        (dataLog.DdB,    directivity,    "DdB"),
        (dataLog.G_TN,    G_TN,    "G_TN"),
        (dataLog.P_N,     P_N,     "P_N"),
        (dataLog.P_eirp_dB, P_eirp, "P_eirp_dB"),
        (dataLog.T_Ambient, T_Ambient, "T_Ambient"),
        (dataLog.r_AN_N, r_AN_N, "r_AN_N"),
        (dataLog.sigma_AN, sigma_AN, "sigma_AN"),
        (dataLog.r_AP_N, r_AP_N, "r_AP_N"),
    ]

    # When Haslam is enabled, sky temperature is computed internally from the map, so G_TN/P_N is not validated  against a hard-coded T_sky_true.
    if useHaslamMap:
        checks = [c for c in checks if c[2] not in ("G_TN", "P_N")]

    for data, truth, name in checks:
        for i in range(n):
            sim_val = data[i]
            # If sim_val is iterable (like a vector), compare component-wise
            if np.ndim(truth) > 0:  # vector case
                for k in range(len(sim_val)):
                    if not unitTestSupport.isDoubleEqual(float(sim_val[k]), float(truth[k]), ACCURACY):
                        testFailCount += 1
                        testMessages.append(
                            f"Error: {name}[{k}] not set correctly at index {i} "
                            f"(got {sim_val[k]}, expected {truth[k]})"
                        )
            else:   # scalar case
                if not unitTestSupport.isDoubleEqual(float(sim_val), float(truth), ACCURACY):
                    testFailCount += 1
                    testMessages.append(
                        f"Error: {name} not set correctly at index {i} "
                        f"(got {sim_val}, expected {truth})"
                    )

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    # Infer sky temperature from the module output noise power P_N.
    P_N_sim = float(dataLog.P_N[-1])
    T_S_sim = P_N_sim / (K_BOLTZMANN * bandwidth)
    T_sky_sim = (T_S_sim - T_E - (1.0 - eta_r) * T_Ambient) / eta_r

    return [testFailCount, "".join(testMessages), T_sky_sim]

if __name__ == "__main__":
    test_cases = [
        (simpleAntenna.ANTENNA_OFF, 0, 150, False, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RX,  0, 150, False, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_TX,  0, 150, False, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  1, 150, False, 200, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  0, None, False, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  1, None, False, 200, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  0, 150, True, 5.45, 1.0, [0,0,0], [1,0,0]),
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 300.0, 0.0, rbk.C2MRP(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])), [1, 0, 0]),
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 300.0, 1.0, rbk.C2MRP(np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])), [0, 1, 0]),
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 5.660060787285246, 1.0, rbk.C2MRP(np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])), [1, 0, 0]),
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 37.82128898104384, 1.0, rbk.C2MRP(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])), [-1, 0, 0]),
        (simpleAntenna.ANTENNA_RXTX,  0, 150, False, 147.9662263230431, 0.0, earth_horizon_sigma(), [1, 0, 0]),
]

    print("=" * 60)
    print("Running all test_simpleAntenna parametrized cases")
    print("=" * 60)

    passed = 0
    failed = 0
    for i, (antennaState, env_type, T_AmbientSet, useHaslamMap, T_sky_true, eclipseValue, sigma_AB, scPosUnit) in enumerate(test_cases):
        try:
            test_simpleAntenna(antennaState, env_type, T_AmbientSet, useHaslamMap, T_sky_true, eclipseValue, sigma_AB, scPosUnit, ACCURACY)
            print(f"  Case {i+1:2d}: PASSED")
            passed += 1
        except AssertionError as e:
            print(f"  Case {i+1:2d}: FAILED - {e}")
            failed += 1

    print("-" * 60)
    print(f"test_simpleAntenna: {passed} passed, {failed} failed")
    print()

    print("=" * 60)
    print("Running test_simpleAntenna_haslamMap_three_runs_and_pointing_difference")
    print("=" * 60)
    # ADD THIS:
    try:
        test_simpleAntenna_haslamMap_three_runs_and_pointing_difference()
        print("  PASSED")
    except AssertionError as e:
        print(f"  FAILED - {e}")
