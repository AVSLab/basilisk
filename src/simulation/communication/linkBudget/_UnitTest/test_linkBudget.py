#
#  ISC License
#
#  Copyright (c) 2025, Department of Engineering Cybernetics, NTNU
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
**Test Description**

This unit test validates the ``linkBudget`` module which computes end-to-end
radio link budget between two antennas. The test writes directly to
``AntennaLogMsgPayload`` messages to isolate the linkBudget module from
simpleAntenna dependencies.

**Test Coverage**

- Free Space Path Loss (FSPL) calculations
- Carrier-to-Noise Ratio (CNR) calculations
- Pointing loss calculations
- Frequency offset loss calculations
- Space-to-space and space-to-ground link configurations
- Antenna state handling (Tx/Rx/RxTx combinations)
- Edge cases (no bandwidth overlap, co-located antennas, etc.)

"""

import numpy as np
import pytest

from Basilisk import __path__
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.utilities import macros, orbitalMotion
from Basilisk.simulation import linkBudget
from Basilisk.utilities import RigidBodyKinematics as rbk

bskPath = __path__[0]

# =============================================================================
# Physical Constants
# =============================================================================
SPEED_LIGHT = 299792458.0       # [m/s]
K_BOLTZMANN = 1.38064852e-23    # [J/K]

# Test tolerances
ACCURACY = 1e-6
ACCURACY_DB = 0.1               # [dB] tolerance for dB comparisons
ACCURACY_CNR_REL = 0.05         # 5% relative tolerance for CNR

# Antenna state enums (matching AntennaDefinitions.h)
ANTENNA_OFF = 0
ANTENNA_RX = 1
ANTENNA_TX = 2
ANTENNA_RXTX = 3

# Environment enums
ENV_SPACE = 0
ENV_EARTH = 1


# =============================================================================
# Helper Functions - Analytical Calculations
# =============================================================================

def compute_fspl(distance_m, frequency_Hz):
    """
    Compute Free Space Path Loss in dB.

    FSPL = 20*log10(4*pi*d/lambda) = 20*log10(4*pi*d*f/c)

    Args:
        distance_m: Distance between antennas [m]
        frequency_Hz: Operating frequency [Hz]

    Returns:
        FSPL in [dB]
    """
    wavelength = SPEED_LIGHT / frequency_Hz
    fspl = 20.0 * np.log10((4.0 * np.pi * distance_m) / wavelength)
    return fspl


def compute_pointing_loss(theta_az, theta_el, HPBW_az, HPBW_el):
    """
    Compute pointing loss for Gaussian beam pattern.

    L_point = 10*log10(e)*4*ln(2) * (theta_az^2/HPBW_az^2 + theta_el^2/HPBW_el^2)

    Args:
        theta_az: Azimuth pointing error [rad]
        theta_el: Elevation pointing error [rad]
        HPBW_az: Half-power beamwidth in azimuth [rad]
        HPBW_el: Half-power beamwidth in elevation [rad]

    Returns:
        Pointing loss [dB]
    """
    coeff = 10.0 * np.log10(np.exp(1.0)) * 4.0 * np.log(2.0)  # ≈ 12.04
    L_point = coeff * ((theta_az**2) / (HPBW_az**2) + (theta_el**2) / (HPBW_el**2))
    return L_point


def compute_cnr(P_eirp_dB, G_rx_dB, L_fspl, L_atm, L_point, L_freq, P_N_watts):
    """
    Compute Carrier-to-Noise Ratio.

    P_Rx [dBW] = P_eirp [dBW] + G_rx [dB] - L_fspl [dB] - L_atm [dB] - L_point [dB] - L_freq [dB]
    CNR = P_Rx / P_N  (linear)

    Args:
        P_eirp_dB: EIRP of transmitter [dBW]
        G_rx_dB: Receiver antenna gain [dB]
        L_fspl: Free space path loss [dB]
        L_atm: Atmospheric loss [dB]
        L_point: Pointing loss [dB]
        L_freq: Frequency offset loss [dB]
        P_N_watts: Noise power [W] (linear)

    Returns:
        CNR (linear, dimensionless)
    """
    P_rx_dBW = P_eirp_dB + G_rx_dB - L_fspl - L_atm - L_point - L_freq
    P_N_dBW = 10.0 * np.log10(P_N_watts)
    cnr_dB = P_rx_dBW - P_N_dBW
    cnr_linear = 10.0 ** (cnr_dB / 10.0)
    return cnr_linear


def compute_antenna_derived_params(directivity_dB, k, P_Tx, eta_r, T_E, T_Ambient, T_sky, bandwidth):
    """
    Compute derived antenna parameters for test setup.

    Args:
        directivity_dB: Antenna directivity [dB]
        k: HPBW ratio (az/el)
        P_Tx: Transmit power [W]
        eta_r: Radiation efficiency [-]
        T_E: Equivalent noise temperature [K]
        T_Ambient: Ambient temperature [K]
        T_sky: Sky temperature [K]
        bandwidth: Bandwidth [Hz]

    Returns:
        dict with HPBW_az, HPBW_el, P_eirp_dB, G_rx_dB, P_N, T_S
    """
    dir_lin = 10.0 ** (directivity_dB / 10.0)
    HPBW_el = np.sqrt((np.log(2) * 16) / (dir_lin * k))
    HPBW_az = k * HPBW_el

    P_eirp_dB = 10.0 * np.log10(P_Tx) + 10.0 * np.log10(eta_r) + directivity_dB
    G_rx_dB = directivity_dB + 10.0 * np.log10(eta_r)

    T_ant = (1.0 - eta_r) * T_Ambient + eta_r * T_sky
    T_S = T_E + T_ant
    P_N = K_BOLTZMANN * T_S * bandwidth

    return {
        'HPBW_az': HPBW_az,
        'HPBW_el': HPBW_el,
        'P_eirp_dB': P_eirp_dB,
        'G_rx_dB': G_rx_dB,
        'P_N': P_N,
        'T_S': T_S
    }


def compute_pointing_sigma(from_pos, to_pos):
    """
    Compute MRP (sigma) to point antenna boresight (+Z) from from_pos toward to_pos.

    The antenna frame convention is that +Z is the boresight direction.
    This function computes the MRP representing the rotation needed to
    align the antenna +Z axis with the direction from from_pos to to_pos.

    Args:
        from_pos: Antenna position [m] (list or array of 3 elements)
        to_pos: Target position [m] (list or array of 3 elements)

    Returns:
        MRP sigma as list [s1, s2, s3]
    """
    from_pos = np.array(from_pos, dtype=float)
    to_pos = np.array(to_pos, dtype=float)

    # Desired boresight direction (unit vector toward target)
    direction = to_pos - from_pos
    dist = np.linalg.norm(direction)

    if dist < 1e-6:
        # Co-located antennas, no rotation needed (or undefined)
        return [0.0, 0.0, 0.0]

    d_hat = direction / dist  # Unit vector toward target

    # Antenna boresight is +Z in antenna frame
    z_hat = np.array([0.0, 0.0, 1.0])

    # Find rotation from z_hat to d_hat
    dot = np.dot(z_hat, d_hat)

    if dot > 0.99999:
        # Already aligned with +Z, no rotation needed
        return [0.0, 0.0, 0.0]
    elif dot < -0.99999:
        # Opposite direction (-Z), 180° rotation about X-axis
        # MRP for 180° about X: sigma = tan(pi/4) * [1,0,0] = [1,0,0]
        return [1.0, 0.0, 0.0]
    else:
        # General case: rotation axis = z_hat × d_hat, angle = acos(dot)
        axis = np.cross(z_hat, d_hat)
        axis = axis / np.linalg.norm(axis)
        angle = np.arccos(np.clip(dot, -1.0, 1.0))

        # MRP: sigma = tan(angle/4) * axis
        sigma = np.tan(angle / 4.0) * axis
        return sigma.tolist()


def apply_pointing_error(base_sigma, error_az_rad, error_el_rad):
    """
    Apply pointing error to a base orientation.

    Args:
        base_sigma: Base MRP orientation [s1, s2, s3]
        error_az_rad: Azimuth error [rad] (rotation about Y in antenna frame)
        error_el_rad: Elevation error [rad] (rotation about X in antenna frame)

    Returns:
        New MRP with pointing error applied
    """
    # Convert base sigma to DCM
    dcm_base = rbk.MRP2C(base_sigma)

    # Create error rotation (small angle approximation or full rotation)
    # Error in antenna frame: first rotate about X (elevation), then Y (azimuth)
    dcm_err_el = rbk.euler1(error_el_rad)  # Rotation about X
    dcm_err_az = rbk.euler2(error_az_rad)  # Rotation about Y
    dcm_error = dcm_err_az @ dcm_err_el    # Combined error rotation

    # Apply error: new DCM = base DCM * error DCM
    dcm_new = dcm_base @ dcm_error

    # Convert back to MRP
    sigma_new = rbk.C2MRP(dcm_new)
    return sigma_new.tolist()


# =============================================================================
# Helper Functions - Message Creation
# =============================================================================

def create_antenna_msg_payload(
    name="Antenna",
    environment=ENV_SPACE,
    state=ANTENNA_OFF,
    frequency=2.2e9,
    bandwidth=5e6,
    directivity_dB=20.0,
    k=1.0,
    P_Tx=100.0,
    eta_r=0.6,
    T_E=50.0,
    T_Ambient=150.0,
    T_sky=5.0,
    position=[0, 0, 0],
    sigma=None,
    velocity=[0, 0, 0],
    r_AP_N=[0, 0, 0],
    nHat_LP_N=[0, 0, 1],
    target_position=None,
    pointing_error_az_deg=0.0,
    pointing_error_el_deg=0.0,
):
    """
    Create an AntennaLogMsgPayload with computed derived values.

    This allows tests to specify high-level parameters and automatically
    computes HPBW, P_eirp, P_N, etc.

    Args:
        name: Antenna name string
        environment: ENV_SPACE (0) or ENV_EARTH (1)
        state: ANTENNA_OFF/RX/TX/RXTX
        frequency: Operating frequency [Hz]
        bandwidth: Bandwidth [Hz]
        directivity_dB: Antenna directivity [dB]
        k: HPBW ratio (az/el), 1.0 for symmetric beam
        P_Tx: Transmit power [W]
        eta_r: Radiation efficiency [0-1]
        T_E: Equivalent noise temperature [K]
        T_Ambient: Ambient temperature [K]
        T_sky: Sky noise temperature [K]
        position: Antenna position in inertial frame [m]
        sigma: MRP orientation (overrides target_position if both given)
        velocity: Antenna velocity [m/s]
        r_AP_N: Position relative to planet (for ground stations) [m]
        nHat_LP_N: Surface normal vector (for ground stations)
        target_position: If provided, compute sigma to point toward this position
        pointing_error_az_deg: Azimuth pointing error [degrees]
        pointing_error_el_deg: Elevation pointing error [degrees]

    Returns:
        (payload, params) tuple where payload is AntennaLogMsgPayload
        and params is dict of derived parameters
    """
    # Compute derived parameters
    params = compute_antenna_derived_params(
        directivity_dB, k, P_Tx, eta_r, T_E, T_Ambient, T_sky, bandwidth
    )

    # Determine antenna orientation
    if sigma is not None:
        # Use explicitly provided sigma
        final_sigma = list(sigma)
    elif target_position is not None:
        # Compute sigma to point at target
        final_sigma = compute_pointing_sigma(position, target_position)
    else:
        # Default: no rotation (boresight along +Z)
        final_sigma = [0.0, 0.0, 0.0]

    # Apply pointing error if specified
    if pointing_error_az_deg != 0.0 or pointing_error_el_deg != 0.0:
        final_sigma = apply_pointing_error(
            final_sigma,
            np.deg2rad(pointing_error_az_deg),
            np.deg2rad(pointing_error_el_deg)
        )

    # Create payload
    payload = messaging.AntennaLogMsgPayload()
    payload.antennaName = name
    payload.environment = environment
    payload.antennaState = state
    payload.frequency = frequency
    payload.B = bandwidth
    payload.HPBW_az = params['HPBW_az']
    payload.HPBW_el = params['HPBW_el']
    payload.P_Tx = P_Tx
    payload.P_Rx = 0.0
    payload.DdB = directivity_dB
    payload.eta_r = eta_r
    payload.P_N = params['P_N']
    payload.P_eirp_dB = params['P_eirp_dB']
    payload.G_TN = directivity_dB + 10.0 * np.log10(eta_r) - 10.0 * np.log10(params['T_S'])
    payload.T_Ambient = T_Ambient
    payload.r_AN_N = position
    payload.sigma_AN = final_sigma
    payload.v_AN_N = velocity
    payload.r_AP_N = r_AP_N
    payload.r_AP_P = r_AP_N  # Simplified: same as r_AP_N for testing
    payload.nHat_LP_N = nHat_LP_N

    return payload, params


def setup_link_budget_sim(ant1_payload, ant2_payload, earth_pos=[0, 0, 0]):
    """
    Set up a simulation with linkBudget module and two antenna messages.

    Args:
        ant1_payload: AntennaLogMsgPayload for antenna 1
        ant2_payload: AntennaLogMsgPayload for antenna 2
        earth_pos: Earth position for spice message [m]

    Returns:
        (unitTestSim, linkBudgetModule, linkDataLog)
    """
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Write antenna messages
    ant1Msg = messaging.AntennaLogMsg().write(ant1_payload)
    ant2Msg = messaging.AntennaLogMsg().write(ant2_payload)

    # Create link budget module
    linkBudgetModule = linkBudget.LinkBudget()
    linkBudgetModule.ModelTag = "linkBudget"
    unitTestSim.AddModelToTask(unitTaskName, linkBudgetModule)

    # Subscribe to antenna messages
    linkBudgetModule.antennaInPayload_1.subscribeTo(ant1Msg)
    linkBudgetModule.antennaInPayload_2.subscribeTo(ant2Msg)

    # Earth/planet state message (required by linkBudget)
    earthPayload = messaging.SpicePlanetStateMsgPayload()
    earthPayload.PlanetName = "earth"
    earthPayload.PositionVector = earth_pos
    earthPayload.J20002Pfix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    earthMsg = messaging.SpicePlanetStateMsg().write(earthPayload)
    linkBudgetModule.spicePlanetStateInMsg.subscribeTo(earthMsg)

    # Output recorder
    linkDataLog = linkBudgetModule.linkBudgetOutPayload.recorder()
    unitTestSim.AddModelToTask(unitTaskName, linkDataLog)

    return unitTestSim, linkBudgetModule, linkDataLog


def run_simulation(unitTestSim, duration_sec=1.0):
    """Initialize and run simulation."""
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(duration_sec))
    unitTestSim.ExecuteSimulation()


# =============================================================================
# Parameterized Tests
# =============================================================================

@pytest.mark.parametrize(
    "ant1_state, ant2_state, link_type, distance_km, freq_offset_Hz, pointing_error_deg",
    [
        # Basic space-to-space links
        (ANTENNA_TX, ANTENNA_RX, "space_space", 1000, 0, 0),
        (ANTENNA_RX, ANTENNA_TX, "space_space", 1000, 0, 0),
        (ANTENNA_RXTX, ANTENNA_RXTX, "space_space", 1000, 0, 0),

        # Different distances
        (ANTENNA_TX, ANTENNA_RX, "space_space", 100, 0, 0),
        (ANTENNA_TX, ANTENNA_RX, "space_space", 10000, 0, 0),
        (ANTENNA_TX, ANTENNA_RX, "space_space", 36000, 0, 0),  # ~GEO altitude

        # With pointing errors (antenna 1 only)
        (ANTENNA_TX, ANTENNA_RX, "space_space", 1000, 0, 5),
        (ANTENNA_TX, ANTENNA_RX, "space_space", 1000, 0, 10),

        # Space-to-ground link
        (ANTENNA_TX, ANTENNA_RX, "space_ground", 400, 0, 0),  # LEO pass

        # Both antennas OFF - CNR should be -1
        (ANTENNA_OFF, ANTENNA_OFF, "space_space", 1000, 0, 0),

        # Mismatched states - TX to TX (no receiver) - both CNR should be -1
        (ANTENNA_TX, ANTENNA_TX, "space_space", 1000, 0, 0),

        # RX to RX (no transmitter) - CNR should be low/zero
        (ANTENNA_RX, ANTENNA_RX, "space_space", 1000, 0, 0),
    ]
)
def test_linkBudget(ant1_state, ant2_state, link_type, distance_km, freq_offset_Hz, pointing_error_deg):
    r"""
    **Validation Test Description**

    This unit test validates the link budget calculations between two antennas
    under various configurations including different antenna states, distances,
    pointing errors, and link types (space-to-space, space-to-ground).

    **Test Parameters**

    Args:
        ant1_state: State of antenna 1 (OFF/RX/TX/RXTX)
        ant2_state: State of antenna 2 (OFF/RX/TX/RXTX)
        link_type: Type of link ("space_space" or "space_ground")
        distance_km: Distance between antennas [km]
        freq_offset_Hz: Frequency offset between antennas [Hz]
        pointing_error_deg: Pointing error for antenna 1 [degrees]
    """
    [testResults, testMessage] = linkBudgetTestFunction(
        ant1_state, ant2_state, link_type, distance_km,
        freq_offset_Hz, pointing_error_deg
    )
    assert testResults < 1, testMessage


def linkBudgetTestFunction(ant1_state, ant2_state, link_type, distance_km,
                           freq_offset_Hz, pointing_error_deg):
    """Main test function for parameterized link budget tests."""
    testFailCount = 0
    testMessages = []

    # Common parameters
    frequency = 2.2e9           # Hz (S-band)
    bandwidth = 5e6             # Hz
    directivity_dB = 20.0       # dB
    k = 1.0                     # Symmetric beam
    P_Tx = 100.0                # W
    eta_r = 0.6                 # Radiation efficiency
    T_E = 50.0                  # K
    T_Ambient = 150.0           # K
    T_sky = 5.0                 # K (deep space)

    distance_m = distance_km * 1000.0

    # Compute expected derived parameters
    params = compute_antenna_derived_params(
        directivity_dB, k, P_Tx, eta_r, T_E, T_Ambient, T_sky, bandwidth
    )

    # Define positions
    ant1_position = [0.0, 0.0, 0.0]
    ant2_position = [0.0, 0.0, distance_m]

    # === Create Antenna 1 Payload ===
    # Antenna 1 points toward antenna 2, with optional pointing error
    ant1_payload, _ = create_antenna_msg_payload(
        name="Antenna1",
        environment=ENV_SPACE,
        state=ant1_state,
        frequency=frequency,
        bandwidth=bandwidth,
        directivity_dB=directivity_dB,
        k=k,
        P_Tx=P_Tx,
        eta_r=eta_r,
        T_E=T_E,
        T_Ambient=T_Ambient,
        T_sky=T_sky,
        position=ant1_position,
        target_position=ant2_position,
        pointing_error_az_deg=pointing_error_deg,
        pointing_error_el_deg=0.0,
    )

    # === Create Antenna 2 Payload ===
    env2 = ENV_EARTH if (link_type == "space_ground") else ENV_SPACE

    # For ground station, set surface properties
    if link_type == "space_ground":
        r_AP_N = [orbitalMotion.REQ_EARTH * 1000, 0, 0]
        nHat = [1, 0, 0]
        T_Ambient_2 = 288.0  # Ground temperature
        T_sky_2 = 200.0      # Ground sky temperature
    else:
        r_AP_N = [0, 0, 0]
        nHat = [0, 0, 1]
        T_Ambient_2 = T_Ambient
        T_sky_2 = T_sky

    # Antenna 2 points toward antenna 1 (perfect pointing)
    ant2_payload, _ = create_antenna_msg_payload(
        name="Antenna2",
        environment=env2,
        state=ant2_state,
        frequency=frequency + freq_offset_Hz,
        bandwidth=bandwidth,
        directivity_dB=directivity_dB,
        k=k,
        P_Tx=P_Tx,
        eta_r=eta_r,
        T_E=T_E,
        T_Ambient=T_Ambient_2,
        T_sky=T_sky_2,
        position=ant2_position,
        target_position=ant1_position,
        r_AP_N=r_AP_N,
        nHat_LP_N=nHat,
    )

    # === Run Simulation ===
    unitTestSim, linkBudgetModule, linkDataLog = setup_link_budget_sim(
        ant1_payload, ant2_payload
    )
    run_simulation(unitTestSim)

    # === Extract Results ===
    dist_sim = float(linkDataLog.distance[-1])
    cnr1_sim = float(linkDataLog.CNR1[-1])
    cnr2_sim = float(linkDataLog.CNR2[-1])
    fspl_sim = linkBudgetModule.getL_FSPL()
    L_point_sim = linkBudgetModule.getL_point()

    # === Validate Results ===

    # 1. Check distance
    if abs(dist_sim - distance_m) > ACCURACY:  # 1 meter tolerance
        testFailCount += 1
        testMessages.append(
            f"Distance error: got {dist_sim/1e3:.3f} km, expected {distance_km:.3f} km"
        )

    # 2. Check FSPL
    fspl_truth = compute_fspl(distance_m, frequency)
    if abs(fspl_sim - fspl_truth) > ACCURACY_DB:
        testFailCount += 1
        testMessages.append(
            f"FSPL error: got {fspl_sim:.2f} dB, expected {fspl_truth:.2f} dB"
        )

    # 3. Check CNR based on antenna states
    ant1_is_rx = ant1_state in [ANTENNA_RX, ANTENNA_RXTX]
    ant2_is_rx = ant2_state in [ANTENNA_RX, ANTENNA_RXTX]
    ant1_is_tx = ant1_state in [ANTENNA_TX, ANTENNA_RXTX]
    ant2_is_tx = ant2_state in [ANTENNA_TX, ANTENNA_RXTX]

    # CNR1: Antenna 1 receiving from Antenna 2
    if ant1_is_rx and ant2_is_tx:
        if cnr1_sim <= 0:
            testFailCount += 1
            testMessages.append(f"CNR1 should be positive when ant1=RX and ant2=TX, got {cnr1_sim}")
    # When ant1 is TX only or OFF, CNR1 should be 0
    elif not ant1_is_rx:
        if cnr1_sim != 0.0:
            testFailCount += 1
            testMessages.append(f"CNR1 should be 0 when ant1 is not RX, got {cnr1_sim}")

    # CNR2: Antenna 2 receiving from Antenna 1
    if ant2_is_rx and ant1_is_tx:
        if cnr2_sim <= 0:
            testFailCount += 1
            testMessages.append(f"CNR2 should be positive when ant2=RX and ant1=TX, got {cnr2_sim}")
    # When ant2 is TX only or OFF, CNR2 should be 0
    elif not ant2_is_rx:
        if cnr2_sim != -1.0:
            testFailCount += 1
            testMessages.append(f"CNR2 should be -1 when ant2 is not RX, got {cnr2_sim}")

    # 4. Check pointing loss (only for space-space with pointing error)
    if pointing_error_deg > 0 and link_type == "space_space":
        # Antenna 1 has the pointing error, antenna 2 is perfect
        pointing_error_rad = np.deg2rad(pointing_error_deg)
        L_point_truth = compute_pointing_loss(
            pointing_error_rad, 0, params['HPBW_az'], params['HPBW_el']
        )
        # Allow for some tolerance due to different calculation methods
        if abs(L_point_sim - L_point_truth) > 2 * ACCURACY_DB:
            testFailCount += 1
            testMessages.append(
                f"Pointing loss error: got {L_point_sim:.2f} dB, expected {L_point_truth:.2f} dB"
            )

    # Report results
    if testFailCount == 0:
        print(f"PASSED: linkBudget (d={distance_km}km, type={link_type}, "
              f"states={ant1_state}/{ant2_state})")
    else:
        print(f"FAILED: {testFailCount} errors")
        for msg in testMessages:
            print(f"  {msg}")

    return [testFailCount, "\n".join(testMessages)]


# =============================================================================
# Focused Validation Tests
# =============================================================================

def test_linkBudget_fspl_analytical():
    """
    Validate FSPL calculation against analytical formula.

    Tests multiple distance/frequency combinations to ensure
    FSPL = 20*log10(4*pi*d/lambda) is computed correctly.
    """
    testFailCount = 0
    testMessages = []

    # Test cases: (distance_m, frequency_Hz, description)
    test_cases = [
        (1000e3, 2.2e9, "1000 km at S-band"),
        (400e3, 2.2e9, "400 km LEO at S-band"),
        (36000e3, 2.2e9, "GEO at S-band"),
        (1000e3, 8.0e9, "1000 km at X-band"),
        (1000e3, 400e6, "1000 km at UHF"),
        (100e3, 2.2e9, "100 km at S-band"),
    ]

    for distance_m, frequency_Hz, desc in test_cases:
        # Analytical FSPL
        fspl_truth = compute_fspl(distance_m, frequency_Hz)

        # Define positions
        pos1 = [0.0, 0.0, 0.0]
        pos2 = [0.0, 0.0, distance_m]

        # Create antenna payloads with proper pointing
        ant1_payload, _ = create_antenna_msg_payload(
            name="Ant1", state=ANTENNA_TX, frequency=frequency_Hz,
            position=pos1, target_position=pos2
        )
        ant2_payload, _ = create_antenna_msg_payload(
            name="Ant2", state=ANTENNA_RX, frequency=frequency_Hz,
            position=pos2, target_position=pos1
        )

        # Run simulation
        unitTestSim, linkBudgetModule, _ = setup_link_budget_sim(ant1_payload, ant2_payload)
        run_simulation(unitTestSim)

        fspl_sim = linkBudgetModule.getL_FSPL()

        if abs(fspl_sim - fspl_truth) > ACCURACY_DB:
            testFailCount += 1
            testMessages.append(
                f"FSPL mismatch ({desc}): got {fspl_sim:.2f} dB, expected {fspl_truth:.2f} dB"
            )

    if testFailCount == 0:
        print("PASSED: test_linkBudget_fspl_analytical")
    else:
        print(f"FAILED: {testFailCount} errors")
        for msg in testMessages:
            print(f"  {msg}")

    assert testFailCount == 0, "\n".join(testMessages)


def test_linkBudget_cnr_calculation():
    """
    Validate CNR calculation against analytical formula.

    Sets up a simple link with known parameters and verifies
    the CNR output matches the expected value.
    """
    testFailCount = 0
    testMessages = []

    # Known parameters
    distance_m = 1000e3         # 1000 km
    frequency_Hz = 2.2e9        # S-band
    bandwidth_Hz = 5e6          # 5 MHz
    directivity_dB = 20.0       # 20 dBi
    P_Tx = 100.0                # 100 W
    eta_r = 0.6                 # 60% efficiency
    T_E = 50.0                  # 50 K
    T_Ambient = 150.0           # 150 K
    T_sky = 5.0                 # 5 K
    k = 1.0

    # Compute expected values
    params = compute_antenna_derived_params(
        directivity_dB, k, P_Tx, eta_r, T_E, T_Ambient, T_sky, bandwidth_Hz
    )
    fspl_truth = compute_fspl(distance_m, frequency_Hz)

    # Perfect pointing, no atmospheric loss, no frequency offset
    cnr_truth = compute_cnr(
        params['P_eirp_dB'],
        params['G_rx_dB'],
        fspl_truth,
        L_atm=0.0,
        L_point=0.0,
        L_freq=0.0,
        P_N_watts=params['P_N']
    )

    # Define positions
    pos1 = [0.0, 0.0, 0.0]
    pos2 = [0.0, 0.0, distance_m]

    # Create antenna payloads with proper pointing
    ant1_payload, _ = create_antenna_msg_payload(
        name="TxAnt", state=ANTENNA_TX, frequency=frequency_Hz,
        bandwidth=bandwidth_Hz, directivity_dB=directivity_dB, k=k,
        P_Tx=P_Tx, eta_r=eta_r, T_E=T_E, T_Ambient=T_Ambient, T_sky=T_sky,
        position=pos1, target_position=pos2
    )
    ant2_payload, _ = create_antenna_msg_payload(
        name="RxAnt", state=ANTENNA_RX, frequency=frequency_Hz,
        bandwidth=bandwidth_Hz, directivity_dB=directivity_dB, k=k,
        P_Tx=P_Tx, eta_r=eta_r, T_E=T_E, T_Ambient=T_Ambient, T_sky=T_sky,
        position=pos2, target_position=pos1
    )

    # Run simulation
    unitTestSim, linkBudgetModule, linkDataLog = setup_link_budget_sim(
        ant1_payload, ant2_payload
    )
    run_simulation(unitTestSim)

    cnr_sim = float(linkDataLog.CNR2[-1])  # Antenna 2 is receiver

    # Compare with relative tolerance
    if cnr_truth > 0:
        rel_error = abs(cnr_sim - cnr_truth) / cnr_truth
        if rel_error > ACCURACY_CNR_REL:
            testFailCount += 1
            testMessages.append(
                f"CNR mismatch: got {cnr_sim:.4e}, expected {cnr_truth:.4e} "
                f"(rel error: {rel_error*100:.2f}%)"
            )

    if testFailCount == 0:
        print(f"PASSED: test_linkBudget_cnr_calculation (CNR = {cnr_sim:.4e})")
    else:
        print(f"FAILED: {testMessages}")

    assert testFailCount == 0, "\n".join(testMessages)


def test_linkBudget_pointing_loss():
    """
    Validate pointing loss calculation for various off-axis angles.
    """
    testFailCount = 0
    testMessages = []

    # Parameters
    frequency_Hz = 2.2e9
    distance_m = 1000e3
    directivity_dB = 20.0
    k = 1.0

    params = compute_antenna_derived_params(
        directivity_dB, k, 100.0, 0.6, 50.0, 150.0, 5.0, 5e6
    )

    # Define positions
    pos1 = [0.0, 0.0, 0.0]
    pos2 = [0.0, 0.0, distance_m]

    # Test various pointing errors
    pointing_errors_deg = [0, 2, 5, 8, 10]

    for error_deg in pointing_errors_deg:
        # Antenna 1 with pointing error, antenna 2 with perfect pointing
        ant1_payload, _ = create_antenna_msg_payload(
            name="Ant1", state=ANTENNA_TX, frequency=frequency_Hz,
            directivity_dB=directivity_dB, k=k,
            position=pos1, target_position=pos2,
            pointing_error_az_deg=error_deg
        )
        ant2_payload, _ = create_antenna_msg_payload(
            name="Ant2", state=ANTENNA_RX, frequency=frequency_Hz,
            directivity_dB=directivity_dB, k=k,
            position=pos2, target_position=pos1
        )

        unitTestSim, linkBudgetModule, _ = setup_link_budget_sim(
            ant1_payload, ant2_payload
        )
        run_simulation(unitTestSim)

        L_point_sim = linkBudgetModule.getL_point()

        # Expected: only antenna 1 contributes (antenna 2 has perfect pointing)
        error_rad = np.deg2rad(error_deg)
        L_point_truth = compute_pointing_loss(
            error_rad, 0, params['HPBW_az'], params['HPBW_el']
        )

        if abs(L_point_sim - L_point_truth) > ACCURACY_DB:
            testFailCount += 1
            testMessages.append(
                f"Pointing loss at {error_deg}°: got {L_point_sim:.3f} dB, "
                f"expected {L_point_truth:.3f} dB"
            )

    if testFailCount == 0:
        print("PASSED: test_linkBudget_pointing_loss")
    else:
        print(f"FAILED: {testFailCount} errors")
        for msg in testMessages:
            print(f"  {msg}")

    assert testFailCount == 0, "\n".join(testMessages)


def test_linkBudget_no_bandwidth_overlap():
    """
    Test behavior when antennas have no overlapping bandwidth.
    CNR should be 0 or indicate link failure.
    """
    testFailCount = 0
    testMessages = []

    # Antenna 1: 2.2 GHz ± 2.5 MHz -> [2.1975, 2.2025] GHz
    # Antenna 2: 2.3 GHz ± 2.5 MHz -> [2.2975, 2.3025] GHz
    # Gap of ~97.5 MHz -> no overlap

    distance_m = 1000e3
    pos1 = [0.0, 0.0, 0.0]
    pos2 = [0.0, 0.0, distance_m]

    ant1_payload, _ = create_antenna_msg_payload(
        name="Ant1", state=ANTENNA_RXTX, frequency=2.2e9, bandwidth=5e6,
        position=pos1, target_position=pos2
    )
    ant2_payload, _ = create_antenna_msg_payload(
        name="Ant2", state=ANTENNA_RXTX, frequency=2.3e9, bandwidth=5e6,
        position=pos2, target_position=pos1
    )

    unitTestSim, linkBudgetModule, linkDataLog = setup_link_budget_sim(
        ant1_payload, ant2_payload
    )
    run_simulation(unitTestSim)

    cnr1 = float(linkDataLog.CNR1[-1])
    cnr2 = float(linkDataLog.CNR2[-1])
    bandwidth_overlap = float(linkDataLog.bandwidth[-1])

    # With no bandwidth overlap, link should fail
    if bandwidth_overlap > 0:
        testFailCount += 1
        testMessages.append(
            f"Expected zero or negative bandwidth overlap, got {bandwidth_overlap/1e6:.2f} MHz"
        )

    # CNR should indicate failed link (0 or very small)
    if cnr1 > 1e-10 or cnr2 > 1e-10:
        testFailCount += 1
        testMessages.append(
            f"Expected CNR~0 for no bandwidth overlap, got CNR1={cnr1}, CNR2={cnr2}"
        )

    if testFailCount == 0:
        print("PASSED: test_linkBudget_no_bandwidth_overlap")
    else:
        print(f"FAILED: {testMessages}")

    assert testFailCount == 0, "\n".join(testMessages)


def test_linkBudget_partial_bandwidth_overlap():
    """
    Test behavior with partial bandwidth overlap.
    Should see frequency offset loss.
    """
    testFailCount = 0
    testMessages = []

    # Antenna 1: 2.200 GHz ± 5 MHz -> [2.195, 2.205] GHz
    # Antenna 2: 2.203 GHz ± 5 MHz -> [2.198, 2.208] GHz
    # Overlap: [2.198, 2.205] = 7 MHz overlap out of 10 MHz

    bandwidth = 10e6  # 10 MHz each
    freq1 = 2.200e9
    freq2 = 2.203e9   # 3 MHz offset

    distance_m = 1000e3
    pos1 = [0.0, 0.0, 0.0]
    pos2 = [0.0, 0.0, distance_m]

    ant1_payload, _ = create_antenna_msg_payload(
        name="Ant1", state=ANTENNA_TX, frequency=freq1, bandwidth=bandwidth,
        position=pos1, target_position=pos2
    )
    ant2_payload, _ = create_antenna_msg_payload(
        name="Ant2", state=ANTENNA_RX, frequency=freq2, bandwidth=bandwidth,
        position=pos2, target_position=pos1
    )

    unitTestSim, linkBudgetModule, linkDataLog = setup_link_budget_sim(
        ant1_payload, ant2_payload
    )
    run_simulation(unitTestSim)

    bandwidth_overlap = float(linkDataLog.bandwidth[-1])
    L_freq = linkBudgetModule.getL_freq()

    # Expected overlap: 7 MHz
    expected_overlap = 7e6
    if abs(bandwidth_overlap - expected_overlap) > 1e3:  # 1 kHz tolerance
        testFailCount += 1
        testMessages.append(
            f"Bandwidth overlap: got {bandwidth_overlap/1e6:.3f} MHz, "
            f"expected {expected_overlap/1e6:.3f} MHz"
        )

    # Frequency loss should be: 10*log10(7/10) = -1.55 dB
    expected_L_freq = 10.0 * np.log10(expected_overlap / bandwidth)
    if abs(L_freq - expected_L_freq) > ACCURACY_DB:
        testFailCount += 1
        testMessages.append(
            f"Frequency loss: got {L_freq:.3f} dB, expected {expected_L_freq:.3f} dB"
        )

    if testFailCount == 0:
        print("PASSED: test_linkBudget_partial_bandwidth_overlap")
    else:
        print(f"FAILED: {testMessages}")

    assert testFailCount == 0, "\n".join(testMessages)


def test_linkBudget_distance_calculation():
    """
    Validate distance calculation between antenna positions.
    """
    testFailCount = 0
    testMessages = []

    # Test cases: (pos1, pos2, expected_distance_m, description)
    test_cases = [
        ([0, 0, 0], [1000e3, 0, 0], 1000e3, "Along X"),
        ([0, 0, 0], [0, 1000e3, 0], 1000e3, "Along Y"),
        ([0, 0, 0], [0, 0, 1000e3], 1000e3, "Along Z"),
        ([1e6, 2e6, 3e6], [2e6, 2e6, 3e6], 1e6, "Offset along X"),
        ([0, 0, 0], [1e6, 1e6, 1e6], np.sqrt(3)*1e6, "Diagonal"),
        ([100e3, 200e3, 300e3], [400e3, 600e3, 800e3],
         np.sqrt(300e3**2 + 400e3**2 + 500e3**2), "General 3D"),
    ]

    for pos1, pos2, expected_dist, desc in test_cases:
        # Create payloads with proper pointing
        ant1_payload, _ = create_antenna_msg_payload(
            name="Ant1", state=ANTENNA_TX,
            position=pos1, target_position=pos2
        )
        ant2_payload, _ = create_antenna_msg_payload(
            name="Ant2", state=ANTENNA_RX,
            position=pos2, target_position=pos1
        )

        unitTestSim, _, linkDataLog = setup_link_budget_sim(
            ant1_payload, ant2_payload
        )
        run_simulation(unitTestSim)

        dist_sim = float(linkDataLog.distance[-1])

        if abs(dist_sim - expected_dist) > 1.0:  # 1 meter tolerance
            testFailCount += 1
            testMessages.append(
                f"Distance ({desc}): got {dist_sim:.1f} m, expected {expected_dist:.1f} m"
            )

    if testFailCount == 0:
        print("PASSED: test_linkBudget_distance_calculation")
    else:
        print(f"FAILED: {testFailCount} errors")
        for msg in testMessages:
            print(f"  {msg}")

    assert testFailCount == 0, "\n".join(testMessages)


def test_linkBudget_symmetric_bidirectional():
    """
    Test that a bidirectional link (RXTX <-> RXTX) produces
    symmetric CNR values for identical antennas.
    """
    testFailCount = 0
    testMessages = []

    distance_m = 1000e3
    pos1 = [0.0, 0.0, 0.0]
    pos2 = [0.0, 0.0, distance_m]

    # Identical antennas, symmetric geometry, both pointing at each other
    ant1_payload, _ = create_antenna_msg_payload(
        name="Ant1", state=ANTENNA_RXTX,
        position=pos1, target_position=pos2
    )
    ant2_payload, _ = create_antenna_msg_payload(
        name="Ant2", state=ANTENNA_RXTX,
        position=pos2, target_position=pos1
    )

    unitTestSim, _, linkDataLog = setup_link_budget_sim(
        ant1_payload, ant2_payload
    )
    run_simulation(unitTestSim)

    cnr1 = float(linkDataLog.CNR1[-1])
    cnr2 = float(linkDataLog.CNR2[-1])

    # Both should be positive
    if cnr1 <= 0 or cnr2 <= 0:
        testFailCount += 1
        testMessages.append(f"CNR values should be positive: CNR1={cnr1}, CNR2={cnr2}")

    # Should be approximately equal (symmetric link)
    if cnr1 > 0 and cnr2 > 0:
        rel_diff = abs(cnr1 - cnr2) / max(cnr1, cnr2)
        if rel_diff > 0.01:  # 1% tolerance
            testFailCount += 1
            testMessages.append(
                f"Symmetric link should have equal CNR: CNR1={cnr1:.4e}, CNR2={cnr2:.4e}"
            )

    if testFailCount == 0:
        print(f"PASSED: test_linkBudget_symmetric_bidirectional (CNR={cnr1:.4e})")
    else:
        print(f"FAILED: {testMessages}")

    assert testFailCount == 0, "\n".join(testMessages)


def test_linkBudget_fspl_vs_distance():
    """
    Verify FSPL increases by 6 dB when distance doubles (inverse square law).
    """
    testFailCount = 0
    testMessages = []

    frequency = 2.2e9
    base_distance = 1000e3  # 1000 km

    pos1 = [0.0, 0.0, 0.0]
    pos2_base = [0.0, 0.0, base_distance]
    pos2_double = [0.0, 0.0, 2 * base_distance]

    # Get FSPL at base distance
    ant1_payload, _ = create_antenna_msg_payload(
        name="Ant1", state=ANTENNA_TX, frequency=frequency,
        position=pos1, target_position=pos2_base
    )
    ant2_payload, _ = create_antenna_msg_payload(
        name="Ant2", state=ANTENNA_RX, frequency=frequency,
        position=pos2_base, target_position=pos1
    )

    unitTestSim, linkBudgetModule, _ = setup_link_budget_sim(ant1_payload, ant2_payload)
    run_simulation(unitTestSim)
    fspl_base = linkBudgetModule.getL_FSPL()

    # Get FSPL at double distance
    ant1_payload_2, _ = create_antenna_msg_payload(
        name="Ant1", state=ANTENNA_TX, frequency=frequency,
        position=pos1, target_position=pos2_double
    )
    ant2_payload_2x, _ = create_antenna_msg_payload(
        name="Ant2", state=ANTENNA_RX, frequency=frequency,
        position=pos2_double, target_position=pos1
    )

    unitTestSim2, linkBudgetModule2, _ = setup_link_budget_sim(ant1_payload_2, ant2_payload_2x)
    run_simulation(unitTestSim2)
    fspl_double = linkBudgetModule2.getL_FSPL()

    # FSPL should increase by ~6.02 dB when distance doubles
    expected_increase = 20.0 * np.log10(2)  # = 6.02 dB
    actual_increase = fspl_double - fspl_base

    if abs(actual_increase - expected_increase) > 0.05:  # 0.05 dB tolerance
        testFailCount += 1
        testMessages.append(
            f"FSPL increase for 2x distance: got {actual_increase:.3f} dB, "
            f"expected {expected_increase:.3f} dB"
        )

    if testFailCount == 0:
        print(f"PASSED: test_linkBudget_fspl_vs_distance (Δ={actual_increase:.3f} dB)")
    else:
        print(f"FAILED: {testMessages}")

    assert testFailCount == 0, "\n".join(testMessages)


def test_linkBudget_fspl_vs_frequency():
    """
    Verify FSPL increases by 6 dB when frequency doubles.
    """
    testFailCount = 0
    testMessages = []

    distance_m = 1000e3
    base_frequency = 2.2e9

    pos1 = [0.0, 0.0, 0.0]
    pos2 = [0.0, 0.0, distance_m]

    # Get FSPL at base frequency
    ant1_payload, _ = create_antenna_msg_payload(
        name="Ant1", state=ANTENNA_TX, frequency=base_frequency,
        position=pos1, target_position=pos2
    )
    ant2_payload, _ = create_antenna_msg_payload(
        name="Ant2", state=ANTENNA_RX, frequency=base_frequency,
        position=pos2, target_position=pos1
    )

    unitTestSim, linkBudgetModule, _ = setup_link_budget_sim(ant1_payload, ant2_payload)
    run_simulation(unitTestSim)
    fspl_base = linkBudgetModule.getL_FSPL()

    # Get FSPL at double frequency
    ant1_payload_2, _ = create_antenna_msg_payload(
        name="Ant1", state=ANTENNA_TX, frequency=2*base_frequency,
        position=pos1, target_position=pos2
    )
    ant2_payload_2, _ = create_antenna_msg_payload(
        name="Ant2", state=ANTENNA_RX, frequency=2*base_frequency,
        position=pos2, target_position=pos1
    )

    unitTestSim2, linkBudgetModule2, _ = setup_link_budget_sim(ant1_payload_2, ant2_payload_2)
    run_simulation(unitTestSim2)
    fspl_double = linkBudgetModule2.getL_FSPL()

    # FSPL should increase by ~6.02 dB when frequency doubles
    expected_increase = 20.0 * np.log10(2)  # = 6.02 dB
    actual_increase = fspl_double - fspl_base

    if abs(actual_increase - expected_increase) > 0.05:  # 0.05 dB tolerance
        testFailCount += 1
        testMessages.append(
            f"FSPL increase for 2x frequency: got {actual_increase:.3f} dB, "
            f"expected {expected_increase:.3f} dB"
        )

    if testFailCount == 0:
        print(f"PASSED: test_linkBudget_fspl_vs_frequency (Δ={actual_increase:.3f} dB)")
    else:
        print(f"FAILED: {testMessages}")

    assert testFailCount == 0, "\n".join(testMessages)


def test_linkBudget_antenna_state_transitions():
    """
    Test that CNR values correctly reflect antenna state combinations.
    """
    testFailCount = 0
    testMessages = []

    distance_m = 1000e3
    pos1 = [0.0, 0.0, 0.0]
    pos2 = [0.0, 0.0, distance_m]

    # Test matrix: (ant1_state, ant2_state, expect_cnr1_valid, expect_cnr2_valid)
    test_matrix = [
        (ANTENNA_OFF, ANTENNA_OFF, False, False),
        (ANTENNA_OFF, ANTENNA_RX, False, False),   # No TX
        (ANTENNA_OFF, ANTENNA_TX, False, False),   # No RX
        (ANTENNA_OFF, ANTENNA_RXTX, False, False), # Ant1 is OFF
        (ANTENNA_RX, ANTENNA_OFF, False, False),   # Ant2 is OFF
        (ANTENNA_RX, ANTENNA_RX, False, False),    # No TX
        (ANTENNA_RX, ANTENNA_TX, True, False),     # Valid: 2->1
        (ANTENNA_RX, ANTENNA_RXTX, True, False),   # Valid: 2->1, but 1 not TX
        (ANTENNA_TX, ANTENNA_OFF, False, False),   # Ant2 is OFF
        (ANTENNA_TX, ANTENNA_RX, False, True),     # Valid: 1->2
        (ANTENNA_TX, ANTENNA_TX, False, False),    # No RX
        (ANTENNA_TX, ANTENNA_RXTX, False, True),   # Valid: 1->2
        (ANTENNA_RXTX, ANTENNA_OFF, False, False), # Ant2 is OFF
        (ANTENNA_RXTX, ANTENNA_RX, False, True),   # Valid: 1->2
        (ANTENNA_RXTX, ANTENNA_TX, True, False),   # Valid: 2->1
        (ANTENNA_RXTX, ANTENNA_RXTX, True, True),  # Both directions valid
    ]

    for ant1_state, ant2_state, expect_cnr1, expect_cnr2 in test_matrix:
        ant1_payload, _ = create_antenna_msg_payload(
            name="Ant1", state=ant1_state,
            position=pos1, target_position=pos2
        )
        ant2_payload, _ = create_antenna_msg_payload(
            name="Ant2", state=ant2_state,
            position=pos2, target_position=pos1
        )

        unitTestSim, _, linkDataLog = setup_link_budget_sim(ant1_payload, ant2_payload)
        run_simulation(unitTestSim)

        cnr1 = float(linkDataLog.CNR1[-1])
        cnr2 = float(linkDataLog.CNR2[-1])

        # Check CNR1
        if expect_cnr1:
            if cnr1 <= 0:
                testFailCount += 1
                testMessages.append(
                    f"State {ant1_state}/{ant2_state}: CNR1 should be positive, got {cnr1}"
                )
        else:
            if cnr1 > 0 and cnr1 != -1.0:
                testFailCount += 1
                testMessages.append(
                    f"State {ant1_state}/{ant2_state}: CNR1 should be -1 or 0, got {cnr1}"
                )

        # Check CNR2
        if expect_cnr2:
            if cnr2 <= 0:
                testFailCount += 1
                testMessages.append(
                    f"State {ant1_state}/{ant2_state}: CNR2 should be positive, got {cnr2}"
                )
        else:
            if cnr2 > 0 and cnr2 != -1.0:
                testFailCount += 1
                testMessages.append(
                    f"State {ant1_state}/{ant2_state}: CNR2 should be -1 or 0, got {cnr2}"
                )

    if testFailCount == 0:
        print("PASSED: test_linkBudget_antenna_state_transitions")
    else:
        print(f"FAILED: {testFailCount} errors")
        for msg in testMessages:
            print(f"  {msg}")

    assert testFailCount == 0, "\n".join(testMessages)


# =============================================================================
# Main Execution
# =============================================================================

if __name__ == "__main__":
    # Parameterized test cases
    param_test_cases = [
        (ANTENNA_TX, ANTENNA_RX, "space_space", 1000, 0, 0),
        (ANTENNA_RX, ANTENNA_TX, "space_space", 1000, 0, 0),
        (ANTENNA_RXTX, ANTENNA_RXTX, "space_space", 1000, 0, 0),
        (ANTENNA_TX, ANTENNA_RX, "space_space", 100, 0, 0),
        (ANTENNA_TX, ANTENNA_RX, "space_space", 10000, 0, 0),
        (ANTENNA_TX, ANTENNA_RX, "space_space", 36000, 0, 0),
        (ANTENNA_TX, ANTENNA_RX, "space_space", 1000, 0, 5),
        (ANTENNA_TX, ANTENNA_RX, "space_space", 1000, 0, 10),
        (ANTENNA_TX, ANTENNA_RX, "space_ground", 400, 0, 0),
        (ANTENNA_OFF, ANTENNA_OFF, "space_space", 1000, 0, 0),
        (ANTENNA_TX, ANTENNA_TX, "space_space", 1000, 0, 0),
        (ANTENNA_RX, ANTENNA_RX, "space_space", 1000, 0, 0),
    ]

    print("=" * 70)
    print("Running parameterized test_linkBudget cases")
    print("=" * 70)

    passed = 0
    failed = 0
    for i, (ant1, ant2, ltype, dist, foff, perr) in enumerate(param_test_cases):
        try:
            test_linkBudget(ant1, ant2, ltype, dist, foff, perr)
            passed += 1
        except AssertionError as e:
            print(f"  Case {i+1:2d}: FAILED")
            failed += 1
        except Exception as e:
            print(f"  Case {i+1:2d}: ERROR - {type(e).__name__}: {e}")
            failed += 1

    print("-" * 70)
    print(f"Parameterized tests: {passed} passed, {failed} failed")
    print()

    # Focused validation tests
    print("=" * 70)
    print("Running focused validation tests")
    print("=" * 70)

    focused_tests = [
        ("FSPL Analytical", test_linkBudget_fspl_analytical),
        ("CNR Calculation", test_linkBudget_cnr_calculation),
        ("Pointing Loss", test_linkBudget_pointing_loss),
        ("No Bandwidth Overlap", test_linkBudget_no_bandwidth_overlap),
        ("Partial Bandwidth Overlap", test_linkBudget_partial_bandwidth_overlap),
        ("Distance Calculation", test_linkBudget_distance_calculation),
        ("Symmetric Bidirectional", test_linkBudget_symmetric_bidirectional),
        ("FSPL vs Distance (6dB rule)", test_linkBudget_fspl_vs_distance),
        ("FSPL vs Frequency (6dB rule)", test_linkBudget_fspl_vs_frequency),
        ("Antenna State Transitions", test_linkBudget_antenna_state_transitions),
    ]

    passed_focused = 0
    failed_focused = 0
    for name, test_func in focused_tests:
        try:
            test_func()
            passed_focused += 1
        except AssertionError as e:
            print(f"  {name}: FAILED")
            failed_focused += 1
        except Exception as e:
            print(f"  {name}: ERROR - {type(e).__name__}: {e}")
            failed_focused += 1

    print("-" * 70)
    print(f"Focused tests: {passed_focused} passed, {failed_focused} failed")
    print("=" * 70)
