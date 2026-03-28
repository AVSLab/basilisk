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

import inspect
import os

import numpy as np
import pytest
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.simulation import stripLocation
from Basilisk.simulation import spacecraft
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = __path__[0]


def test_range_degenerate_strip(show_plots):
    """
    Tests stripLocation with a degenerate strip (start == end):

    1. Computes range correctly by evaluating slantRange;
    2. Tests whether elevation is correctly evaluated;
    3. Tests whether range limits impact access;
    4. Tests whether multiple spacecraft are supported in parallel.

    This is the stripLocation equivalent of the groundLocation test_range test,
    verifying that a strip with identical start and end points behaves
    like a single ground location.

    :return:
    """

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTime = macros.sec2nano(10.)
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Create a strip target with start == end
    stripTarget = stripLocation.StripLocation()
    stripTarget.ModelTag = "stripTarget"
    stripTarget.planetRadius = orbitalMotion.REQ_EARTH * 1000.
    stripTarget.maximumRange = 100e3  # meters
    stripTarget.minimumElevation = np.radians(80.)
    stripTarget.specifyLocationStart(np.radians(0.), np.radians(0.), 0.)
    stripTarget.specifyLocationEnd(np.radians(0.), np.radians(0.), 0.)
    stripTarget.acquisitionSpeed = 0  # stationary
    scSim.AddModelToTask(simTaskName, stripTarget)

    # Write out mock spacecraft position messages
    sc1_message = messaging.SCStatesMsgPayload()
    sc1_message.r_BN_N = [orbitalMotion.REQ_EARTH * 1e3 + 100e3, 0, 0]  # SC1 directly overhead, in range
    sc1Msg = messaging.SCStatesMsg().write(sc1_message)

    sc2_message = messaging.SCStatesMsgPayload()
    sc2_message.r_BN_N = [orbitalMotion.REQ_EARTH * 1e3 + 101e3, 0, 0]  # SC2 out of range
    sc2Msg = messaging.SCStatesMsg().write(sc2_message)

    sc3_message = messaging.SCStatesMsgPayload() # SC3 is inside the altitude limit,  but outside the visibility cone
    sc3_message.r_BN_N = rbk.euler3(np.radians(11.)).dot(np.array([100e3, 0, 0])) + np.array(
        [orbitalMotion.REQ_EARTH * 1e3, 0, 0])
    sc3Msg = messaging.SCStatesMsg().write(sc3_message)

    stripTarget.addSpacecraftToModel(sc1Msg)
    stripTarget.addSpacecraftToModel(sc2Msg)
    stripTarget.addSpacecraftToModel(sc3Msg)

    # Log the access indicator
    numDataPoints = 2
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog0 = stripTarget.accessOutMsgs[0].recorder(samplingTime)
    dataLog1 = stripTarget.accessOutMsgs[1].recorder(samplingTime)
    dataLog2 = stripTarget.accessOutMsgs[2].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog0)
    scSim.AddModelToTask(simTaskName, dataLog1)
    scSim.AddModelToTask(simTaskName, dataLog2)

    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Get the logged data
    sc1_access = dataLog0.hasAccess
    sc1_slant = dataLog0.slantRange
    sc1_elevation = dataLog0.elevation

    sc2_access = dataLog1.hasAccess
    sc2_slant = dataLog1.slantRange
    sc2_elevation = dataLog1.elevation

    sc3_access = dataLog2.hasAccess
    sc3_slant = dataLog2.slantRange
    sc3_elevation = dataLog2.elevation

    # Compare to expected values
    accuracy = 1e-8
    ref_ranges = [100e3, 101e3, 100e3]
    ref_elevation = [np.radians(90.), np.radians(90.), np.radians(79.)]
    ref_access = [1, 0, 0]

    test_ranges = [sc1_slant[1], sc2_slant[1], sc3_slant[1]]
    test_elevation = [sc1_elevation[1], sc2_elevation[1], sc3_elevation[1]]
    test_access = [sc1_access[1], sc2_access[1], sc3_access[1]]

    range_worked = test_ranges == pytest.approx(ref_ranges, accuracy)
    elevation_worked = test_elevation == pytest.approx(ref_elevation, accuracy)
    access_worked = test_access == pytest.approx(ref_access, abs=1e-16)

    assert range_worked, f"Range check failed: {test_ranges} vs {ref_ranges}"
    assert elevation_worked, f"Elevation check failed: {test_elevation} vs {ref_elevation}"
    assert access_worked, f"Access check failed: {test_access} vs {ref_access}"


def test_rotation_degenerate_strip(show_plots):
    """
    Tests whether stripLocation correctly accounts for planet rotation:

    1. Computes the current strip location based on the initial position and the rotation
       state of the planet it is attached to.

    A strip at (0, 10deg) with the planet rotated by -10deg around z should appear
    at (0, 0) in inertial space, directly beneath a spacecraft on the x-axis.

    :return:
    """
    simTime = 1.

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTime = macros.sec2nano(simTime)
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Create a degenerate strip at (0, 10deg)
    stripTarget = stripLocation.StripLocation()
    stripTarget.ModelTag = "stripTarget"
    stripTarget.planetRadius = orbitalMotion.REQ_EARTH * 1000.
    stripTarget.maximumRange = 200e3  # meters
    stripTarget.minimumElevation = np.radians(10.)
    stripTarget.specifyLocationStart(np.radians(0.), np.radians(10.), 0.)
    stripTarget.specifyLocationEnd(np.radians(0.), np.radians(10.), 0.)
    stripTarget.acquisitionSpeed = 0
    scSim.AddModelToTask(simTaskName, stripTarget)

    # Write out mock planet rotation and spacecraft position messages
    sc1_message = messaging.SCStatesMsgPayload()
    sc1_message.r_BN_N = np.array([orbitalMotion.REQ_EARTH * 1e3 + 90e3, 0, 0])
    scMsg = messaging.SCStatesMsg().write(sc1_message)
    stripTarget.addSpacecraftToModel(scMsg)

    # Rotate planet by -10 deg so the (0, 10deg) ground point aligns with inertial x-axis
    planet_message = messaging.SpicePlanetStateMsgPayload()
    planet_message.J20002Pfix = rbk.euler3(np.radians(-10.)).tolist()
    planetMsg = messaging.SpicePlanetStateMsg().write(planet_message)
    stripTarget.planetInMsg.subscribeTo(planetMsg)

    # Log the access indicator
    numDataPoints = 2
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = stripTarget.accessOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)

    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Get the logged data
    sc1_access = dataLog.hasAccess
    sc1_slant = dataLog.slantRange
    sc1_elevation = dataLog.elevation

    # Compare to expected values
    accuracy = 1e-8
    ref_ranges = [90e3]
    ref_elevation = [np.radians(90.)]
    ref_access = [1]

    test_ranges = [sc1_slant[1]]
    test_elevation = [sc1_elevation[1]]
    test_access = [sc1_access[1]]

    range_worked = test_ranges == pytest.approx(ref_ranges, accuracy)
    elevation_worked = test_elevation == pytest.approx(ref_elevation, accuracy)
    access_worked = test_access == pytest.approx(ref_access, abs=1e-16)

    assert range_worked, f"Range check failed: {test_ranges} vs {ref_ranges}"
    assert elevation_worked, f"Elevation check failed: {test_elevation} vs {ref_elevation}"
    assert access_worked, f"Access check failed: {test_access} vs {ref_access}"


def test_strip_target_motion(show_plots):
    """
    Tests that the strip target position moves along the great-circle arc
    from start to end as time progresses (No pre-imaging is considered in this test):

    1. Verifies that the target position at the midpoint of the traversal
       matches the expected SLERP interpolation on the sphere.
    2. Verifies that the target starts at the start point and finishes at the end point.
    3. Verifies that a spacecraft placed above the midpoint sees ~90deg elevation
       when the target arrives there.

    :return:
    """

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dt = 1.0
    simulationTimeStep = macros.sec2nano(dt)
    totalTimeSec = 10.0
    simulationTime = macros.sec2nano(totalTimeSec)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    R = orbitalMotion.REQ_EARTH * 1000.  # planet radius in meters

    # Create a strip from (lat=0, lon=0) to (lat=0, lon=90deg) on the equator
    stripTarget = stripLocation.StripLocation()
    stripTarget.ModelTag = "stripTarget"
    stripTarget.planetRadius = R
    stripTarget.maximumRange = -1  # unlimited range
    stripTarget.minimumElevation = np.radians(10.)
    stripTarget.specifyLocationStart(np.radians(0.), np.radians(0.), 0.)
    stripTarget.specifyLocationEnd(np.radians(0.), np.radians(90.), 0.)

    # Set acquisition speed so traversal takes exactly totalTimeSec seconds
    strip_length = (np.pi / 2.0) * R  # 90-degree arc length
    acquisitionSpeed = strip_length / macros.sec2nano(totalTimeSec)  # m/ns
    stripTarget.acquisitionSpeed = acquisitionSpeed
    stripTarget.preImagingTime = 0
    scSim.AddModelToTask(simTaskName, stripTarget)

    # Place SC above the midpoint (lon=45deg on the equator)
    midpoint_dir = np.array([np.cos(np.radians(45.)), np.sin(np.radians(45.)), 0.])
    sc_altitude = 200e3  # meters
    sc1_message = messaging.SCStatesMsgPayload()
    sc1_message.r_BN_N = (R + sc_altitude) * midpoint_dir
    sc1Msg = messaging.SCStatesMsg().write(sc1_message)
    stripTarget.addSpacecraftToModel(sc1Msg)

    # Log strip state and access at every simulation step
    stateLog = stripTarget.currentStripStateOutMsg.recorder(simulationTimeStep)
    accessLog = stripTarget.accessOutMsgs[0].recorder(simulationTimeStep)
    scSim.AddModelToTask(simTaskName, stateLog)
    scSim.AddModelToTask(simTaskName, accessLog)

    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # --- Check target positions at key time points ---
    r_LP_N_logged = stateLog.r_LP_N
    pos_accuracy = 1.0  # 1 meter tolerance

    # At t=0s (index 0): target should be at start (lon=0) -> PCPF = [R, 0, 0]
    expected_start = np.array([R, 0., 0.])
    start_error = np.linalg.norm(r_LP_N_logged[0] - expected_start)
    assert start_error < pos_accuracy, \
        f"Target position error at start: {start_error:.6f} m (expected < {pos_accuracy} m)"

    # At t=5s (index 5): target should be at midpoint (lon=45) -> [R/sqrt(2), R/sqrt(2), 0]
    expected_mid = R * midpoint_dir
    mid_error = np.linalg.norm(r_LP_N_logged[5] - expected_mid)
    assert mid_error < pos_accuracy, \
        f"Target position error at midpoint: {mid_error:.6f} m (expected < {pos_accuracy} m)"

    # At t=10s (index 10): target should be at end (lon=90) -> [0, R, 0]
    expected_end = np.array([0., R, 0.])
    end_error = np.linalg.norm(r_LP_N_logged[10] - expected_end)
    assert end_error < pos_accuracy, \
        f"Target position error at end: {end_error:.6f} m (expected < {pos_accuracy} m)"

    # At midpoint, SC should be directly overhead -> elevation ~ 90 deg
    elevation_at_midpoint = accessLog.elevation[5]
    assert elevation_at_midpoint == pytest.approx(np.radians(90.), abs=0.01), \
        f"Elevation at midpoint should be ~90 deg, got {np.degrees(elevation_at_midpoint):.2f} deg"

    # SC above the midpoint should have access when the target is at the midpoint
    assert accessLog.hasAccess[5] == 1, "SC should have access when target is at midpoint"


def test_pre_imaging_blocks_access(show_plots):
    """
    Tests that access is blocked during the pre-imaging phase:

    1. A strip target with preImagingTime > 0 should deny access
       during the initial pre-imaging period, even if the spacecraft
       is geometrically visible.
    2. After the pre-imaging time has elapsed, access should be granted
       if the elevation/range criteria are met.

    :return:
    """

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dt = 1.0
    simulationTimeStep = macros.sec2nano(dt)
    totalTimeSec = 10.0
    simulationTime = macros.sec2nano(totalTimeSec)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    R = orbitalMotion.REQ_EARTH * 1000.

    # Create a stationary strip (start == end) with non-zero preImagingTime
    stripTarget = stripLocation.StripLocation()
    stripTarget.ModelTag = "stripTarget"
    stripTarget.planetRadius = R
    stripTarget.maximumRange = -1  # unlimited
    stripTarget.minimumElevation = np.radians(10.)
    stripTarget.specifyLocationStart(np.radians(0.), np.radians(0.), 0.)
    stripTarget.specifyLocationEnd(np.radians(0.), np.radians(0.), 0.)
    stripTarget.acquisitionSpeed = 0  # stationary
    stripTarget.preImagingTime = 5.0e9  # 5 seconds in nanoseconds
    scSim.AddModelToTask(simTaskName, stripTarget)

    # Place SC directly overhead
    sc1_message = messaging.SCStatesMsgPayload()
    sc1_message.r_BN_N = [R + 100e3, 0, 0]
    sc1Msg = messaging.SCStatesMsg().write(sc1_message)
    stripTarget.addSpacecraftToModel(sc1Msg)

    # Log access at every step
    accessLog = stripTarget.accessOutMsgs[0].recorder(simulationTimeStep)
    scSim.AddModelToTask(simTaskName, accessLog)

    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    access = accessLog.hasAccess

    # During pre-imaging (t=0s to t=4s): duration < preImagingTime -> no access
    # At t=0: duration_strip_imaging = 0 < 5e9 -> access = 0
    # At t=4: duration_strip_imaging = 4e9 < 5e9 -> access = 0
    for i in range(5):  # indices 0..4 correspond to t=0s..t=4s
        assert access[i] == 0, \
            f"Access should be blocked at t={i}s (pre-imaging), got {access[i]}"

    # After pre-imaging (t >= 5s): duration >= preImagingTime -> access granted
    # At t=5: duration_strip_imaging = 5e9 >= 5e9 -> access = 1
    for i in range(5, 11):  # indices 5..10 correspond to t=5s..t=10s
        assert access[i] == 1, \
            f"Access should be granted at t={i}s (post pre-imaging), got {access[i]}"


def test_strip_velocity_output(show_plots):
    """
    Tests that the strip state output message correctly reports the
    velocity of the moving target along the strip:

    1. For a non-degenerate strip, velocity magnitude should equal the acquisition speed.
    2. The velocity vector should be perpendicular to the position vector
       (tangent to the sphere).
    3. The velocity vector should lie in the great-circle plane defined by the
       strip start and end points (i.e. perpendicular to the orbit normal).
    4. The velocity vector should be oriented in the direction of motion
       (from start toward end), verified by checking that r x v is parallel
       to the orbit normal.

    :return:
    """

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dt = 1.0
    simulationTimeStep = macros.sec2nano(dt)
    totalTimeSec = 10.0
    simulationTime = macros.sec2nano(totalTimeSec)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    R = orbitalMotion.REQ_EARTH * 1000.

    # Create a moving strip from (lat=0, lon=0) to (lat=0, lon=90)
    stripTarget = stripLocation.StripLocation()
    stripTarget.ModelTag = "stripTarget"
    stripTarget.planetRadius = R
    stripTarget.maximumRange = -1
    stripTarget.minimumElevation = np.radians(10.)
    stripTarget.specifyLocationStart(np.radians(0.), np.radians(0.), 0.)
    stripTarget.specifyLocationEnd(np.radians(0.), np.radians(90.), 0.)

    strip_length = (np.pi / 2.0) * R
    acquisitionSpeed = strip_length / macros.sec2nano(totalTimeSec)  # m/ns
    stripTarget.acquisitionSpeed = acquisitionSpeed
    stripTarget.preImagingTime = 0
    scSim.AddModelToTask(simTaskName, stripTarget)

    # Need at least one spacecraft for the module to function
    sc1_message = messaging.SCStatesMsgPayload()
    sc1_message.r_BN_N = [R + 200e3, 0, 0]
    sc1Msg = messaging.SCStatesMsg().write(sc1_message)
    stripTarget.addSpacecraftToModel(sc1Msg)

    # Log the strip state
    stateLog = stripTarget.currentStripStateOutMsg.recorder(simulationTimeStep)
    scSim.AddModelToTask(simTaskName, stateLog)

    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    v_LP_N_logged = stateLog.v_LP_N
    r_LP_N_logged = stateLog.r_LP_N

    # Compute the orbit-plane normal from the strip endpoints
    # Start: (lat=0, lon=0)  -> R * [1, 0, 0]
    # End:   (lat=0, lon=90) -> R * [0, 1, 0]
    r_start = R * np.array([1.0, 0.0, 0.0])
    r_end = R * np.array([0.0, 1.0, 0.0])
    orbit_normal = np.cross(r_start, r_end)
    orbit_normal = orbit_normal / np.linalg.norm(orbit_normal)  # [0, 0, 1]

    # Check at several time indices (skip index 0 which is before motion starts)
    for idx in [1, 3, 5, 7, 9]:
        v = v_LP_N_logged[idx]
        r = r_LP_N_logged[idx]
        v_mag = np.linalg.norm(v)
        r_mag = np.linalg.norm(r)

        # 1. Velocity magnitude should equal acquisitionSpeed (in m/ns), convert to m/s for comparison
        assert v_mag == pytest.approx(acquisitionSpeed * 1e9, rel=1e-6), \
            f"[idx={idx}] Velocity magnitude {v_mag} doesn't match acquisition speed {acquisitionSpeed * 1e9} (m/s)"

        # 2. Velocity should be perpendicular to the position vector
        dot_rv = np.dot(r, v) / (r_mag * v_mag)
        assert abs(dot_rv) < 1e-6, \
            f"[idx={idx}] Velocity should be perpendicular to position, dot product = {dot_rv}"

        # 3. Velocity should lie in the great-circle (orbit) plane:
        #    v . n_orbit ≈ 0
        dot_vn = np.dot(v, orbit_normal) / v_mag
        assert abs(dot_vn) < 1e-6, \
            f"[idx={idx}] Velocity should lie in the orbit plane, v · n = {dot_vn}"

        # 4. Velocity should be oriented in the direction of motion:
        #    r x v should be parallel to orbit_normal (same sign)
        rxv = np.cross(r, v)
        rxv_hat = rxv / np.linalg.norm(rxv)
        dot_direction = np.dot(rxv_hat, orbit_normal)
        assert dot_direction > 0.99, \
            f"[idx={idx}] Velocity should point in direction of motion (start->end), " \
            f"(r x v) · n = {dot_direction}"


def test_new_start_pre_imaging(show_plots):
    """
    Tests that ``newpstart()`` correctly extends the strip backward so that
    the target reaches the *original* start point after exactly
    ``preImagingTime`` seconds of travel:

    1. Verifies that at t=0 the target position equals the extended
       (new) start point, which sits ``preImagingTime * speed``
       arc-length behind the original start along the great circle.
    2. Verifies that at t=preImagingTime the target arrives at the
       original start point (lat=0, lon=0).
    3. Verifies that at the end of the updated traversal the target
       reaches the original end point (lat=0, lon=90°).

    :return:
    """

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dt = 1.0
    simulationTimeStep = macros.sec2nano(dt)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    R = orbitalMotion.REQ_EARTH * 1000.  # planet radius in metres

    # ---- strip geometry ----
    # Original strip along the equator from lon=0 to lon=90 deg
    lat_start, lon_start = 0., 0.
    lat_end, lon_end = 0., 90.
    theta_orig = np.radians(lon_end - lon_start)          # 90 deg arc
    strip_length_orig = theta_orig * R                     # original arc length

    # Speeds chosen so the *original* strip takes 10 s to traverse
    totalOriginalTimeSec = 10.0
    acquisitionSpeed = strip_length_orig / macros.sec2nano(totalOriginalTimeSec)  # m / ns

    # Pre-imaging time = 2 s  ⇒  target is extended 2 s of travel behind start
    pre_imaging_sec = 2.0
    pre_imaging_ns = macros.sec2nano(pre_imaging_sec)

    # Expected arc-length behind the original start
    arc_behind = acquisitionSpeed * pre_imaging_ns        # metres
    angle_behind = arc_behind / R                          # radians (≈ 18 deg)

    # Expected new start position: lon = -angle_behind on the equator
    expected_new_start = R * np.array([np.cos(-angle_behind),
                                        np.sin(-angle_behind),
                                        0.])

    # Expected time to traverse the *updated* strip
    # Updated arc covers (theta_orig + angle_behind) radians
    theta_updated = theta_orig + angle_behind
    totalUpdatedTimeSec = (theta_updated * R) / (acquisitionSpeed * 1e9)  # seconds

    # Run sim long enough to cover the extended strip
    simulationTime = macros.sec2nano(np.ceil(totalUpdatedTimeSec))

    # ---- module setup ----
    stripTarget = stripLocation.StripLocation()
    stripTarget.ModelTag = "stripTargetNewStart"
    stripTarget.planetRadius = R
    stripTarget.maximumRange = -1
    stripTarget.minimumElevation = np.radians(10.)
    stripTarget.specifyLocationStart(np.radians(lat_start), np.radians(lon_start), 0.)
    stripTarget.specifyLocationEnd(np.radians(lat_end), np.radians(lon_end), 0.)
    stripTarget.acquisitionSpeed = acquisitionSpeed
    stripTarget.preImagingTime = pre_imaging_ns
    scSim.AddModelToTask(simTaskName, stripTarget)

    # A spacecraft is needed to drive the module (placed arbitrarily)
    sc_msg = messaging.SCStatesMsgPayload()
    sc_msg.r_BN_N = [(R + 200e3), 0, 0]
    scMsg = messaging.SCStatesMsg().write(sc_msg)
    stripTarget.addSpacecraftToModel(scMsg)

    # Log strip state at every step
    stateLog = stripTarget.currentStripStateOutMsg.recorder(simulationTimeStep)
    scSim.AddModelToTask(simTaskName, stateLog)

    # ---- run ----
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # ---- assertions ----
    r_LP_N_logged = stateLog.r_LP_N
    pos_tol = 1.0  # 1 metre tolerance

    # 1) At t = 0 s (index 0): target should be at the *new* start (lon ≈ -18 deg)
    new_start_error = np.linalg.norm(r_LP_N_logged[0] - expected_new_start)
    assert new_start_error < pos_tol, \
        f"New start position error: {new_start_error:.4f} m (tol {pos_tol} m). " \
        f"Expected lon ≈ {np.degrees(-angle_behind):.2f} deg"

    # 2) At t = pre_imaging_sec (index = pre_imaging_sec / dt): target should be
    #    at the *original* start (lat=0, lon=0) -> PCPF [R, 0, 0]
    idx_orig_start = int(pre_imaging_sec / dt)
    expected_orig_start = np.array([R, 0., 0.])
    orig_start_error = np.linalg.norm(r_LP_N_logged[idx_orig_start] - expected_orig_start)
    assert orig_start_error < pos_tol, \
        f"Target should be at original start at t={pre_imaging_sec}s, " \
        f"error = {orig_start_error:.4f} m (tol {pos_tol} m)"

    # 3) At the final step the target should be at the original end (lat=0, lon=90 deg)
    #    -> PCPF [0, R, 0]
    idx_end = int(np.ceil(totalUpdatedTimeSec / dt))
    expected_end = np.array([0., R, 0.])
    end_error = np.linalg.norm(r_LP_N_logged[idx_end] - expected_end)
    assert end_error < pos_tol, \
        f"Target should be at end at t={totalUpdatedTimeSec:.1f}s, " \
        f"error = {end_error:.4f} m (tol {pos_tol} m)"

    # 4) Verify the angular offset between new start and original start ≈ angle_behind
    new_start_pos = r_LP_N_logged[0]
    cos_angle_actual = np.dot(new_start_pos, expected_orig_start) / (
        np.linalg.norm(new_start_pos) * np.linalg.norm(expected_orig_start))
    angle_actual = np.arccos(np.clip(cos_angle_actual, -1.0, 1.0))
    assert angle_actual == pytest.approx(angle_behind, abs=1e-6), \
        f"Angle between new start and original start: {np.degrees(angle_actual):.4f} deg, " \
        f"expected {np.degrees(angle_behind):.4f} deg"


def test_AzElR_rates():
    """
    Tests that the Az, El, range rates are correct by using 1-step Euler integration.
    Uses a stationary strip (start == end at Boulder, CO) to test rate computation,
    analogous to the groundLocation Az/El/R rate test.

    :return:
    """
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dt = 1.0
    simulationTimeStep = macros.sec2nano(dt)
    simulationTime = simulationTimeStep
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    mu = planet.mu
    planet.isCentralBody = True
    timeInitString = '2021 MAY 04 06:47:48.965 (UTC)'
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                     timeInitString)
    gravFactory.spiceObject.zeroBase = 'Earth'
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject, -1)

    scObject = spacecraft.Spacecraft()
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    scSim.AddModelToTask(simTaskName, scObject)
    oe = orbitalMotion.ClassicElements()
    r_sc = planet.radEquator + 100 * 1E3
    oe.a = r_sc
    oe.e = 0.00001
    oe.i = 53.0 * macros.D2R
    oe.Omega = 115.0 * macros.D2R
    oe.omega = 5.0 * macros.D2R
    oe.f = 75. * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN

    # Use a stationary strip (start == end) at Boulder, CO
    stripStation = stripLocation.StripLocation()
    stripStation.planetRadius = planet.radEquator
    stripStation.specifyLocationStart(np.radians(40.009971), np.radians(-105.243895), 1624)
    stripStation.specifyLocationEnd(np.radians(40.009971), np.radians(-105.243895), 1624)
    stripStation.acquisitionSpeed = 0
    stripStation.planetInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    stripStation.minimumElevation = np.radians(60.)
    stripStation.addSpacecraftToModel(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, stripStation)

    # Log the Az, El, R and rates info
    numDataPoints = 2
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = stripStation.accessOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)

    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Get logged data
    sc_range = dataLog.slantRange
    sc_elevation = dataLog.elevation
    sc_azimuth = dataLog.azimuth
    sc_range_rate = dataLog.range_dot
    sc_el_rate = dataLog.el_dot
    sc_az_rate = dataLog.az_dot

    # Euler integration: value(t+dt) ≈ value(t) + rate(t)*dt
    sc_Euler_range = sc_range[0] + sc_range_rate[0] * dt
    sc_Euler_elev = sc_elevation[0] + sc_el_rate[0] * dt
    sc_Euler_azimuth = sc_azimuth[0] + sc_az_rate[0] * dt

    range_rate_worked = sc_range[1] == pytest.approx(sc_Euler_range, rel=1e-5)
    el_rate_worked = sc_elevation[1] == pytest.approx(sc_Euler_elev, rel=1e-5)
    az_rate_worked = sc_azimuth[1] == pytest.approx(sc_Euler_azimuth, rel=1e-5)

    assert range_rate_worked, "Range rate check failed"
    assert el_rate_worked, "Elevation rate check failed"
    assert az_rate_worked, "Azimuth rate check failed"


if __name__ == '__main__':
    test_range_degenerate_strip(False)
    test_rotation_degenerate_strip(False)
    test_strip_target_motion(False)
    test_pre_imaging_blocks_access(False)
    test_strip_velocity_output(False)
    test_new_start_pre_imaging(False)
    test_AzElR_rates()
