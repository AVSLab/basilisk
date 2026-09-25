
# ISC License
#
# Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import spacecraftLocation
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("defaultPolarRadius", [False, True])
@pytest.mark.parametrize("defaultPlanet", [False, True])
@pytest.mark.parametrize("latitude", [55, 65, 115])
@pytest.mark.parametrize("maxRange", [-1, 3000.*1000])
@pytest.mark.parametrize("cone", [0, 1, -1])
def test_spacecraftLocation(show_plots, defaultPolarRadius, defaultPlanet, latitude, maxRange, cone):
    """
    Tests whether spacecraftLocation:

    1. defaults planet polar radius to equatorial radius if the polar radius is not set
    2. checks that the zero default planet states are used if the planet is not provided
    3. checks that the planet oblateness is accounted for
    4. checks if the optional sensor boresight axis is properly accounted for

    :return:
    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, defaultPolarRadius, defaultPlanet, latitude, maxRange, cone)
    assert testResults < 1, testMessage


def run(showplots, defaultPolarRadius, defaultPlanet, latitude, maxRange, cone):

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #   Initialize new atmosphere and drag model, add them to task
    module = spacecraftLocation.SpacecraftLocation()
    module.ModelTag = "scLocation"
    module.rEquator = orbitalMotion.REQ_EARTH * 1000.
    if not defaultPolarRadius:
        module.rPolar = orbitalMotion.REQ_EARTH * 1000. * 0.5
    if maxRange:
        module.maximumRange = maxRange
    if cone != 0:
        module.aHat_B = [0, 0, cone]
        module.theta = 80. * macros.D2R

    scSim.AddModelToTask(simTaskName, module)

    # create planet message
    planetPos = np.array([0.0, 0.0, 0.0])
    if not defaultPlanet:
        planet_message = messaging.SpicePlanetStateMsgPayload()
        planet_message.J20002Pfix = rbk.euler3(np.radians(-90.)).tolist()
        planetPos = np.array([orbitalMotion.AU * 1000, 0.0, 0.0])
        planet_message.PositionVector = planetPos
        planetMsg = messaging.SpicePlanetStateMsg().write(planet_message)
        module.planetInMsg.subscribeTo(planetMsg)

    # create primary spacecraft state message
    alt = 1000. * 1000      # meters
    scMsgData = messaging.SCStatesMsgPayload()
    r = orbitalMotion.REQ_EARTH * 1000. + alt
    scMsgData.r_BN_N = planetPos + np.array([r, 0.0, 0.0])
    scMsgData.sigma_BN = [1.0, 0.0, 0.0]
    scMsg = messaging.SCStatesMsg().write(scMsgData)
    module.primaryScStateInMsg.subscribeTo(scMsg)

    sc1MsgData = messaging.SCStatesMsgPayload()
    angle = np.radians(latitude)
    sc1MsgData.r_BN_N = planetPos + np.array([r * np.cos(angle), 0.0, r * np.sin(angle)])
    sc1Msg = messaging.SCStatesMsg().write(sc1MsgData)
    module.addSpacecraftToModel(sc1Msg)

    # Run the sim
    scSim.InitializeSimulation()

    # check polar planet radius default behavior
    if defaultPolarRadius and module.rPolar < 0:
        testFailCount += 1
        testMessages.append("FAILED: " + module.ModelTag + " Module failed default polar radius check.")

    scSim.TotalSim.SingleStepProcesses()

    accessMsg = module.accessOutMsgs[0].read()

    if latitude == 55 or (latitude == 65 and not defaultPolarRadius):
        trueAccess = 1
        if maxRange > 0:
            if accessMsg.slantRange > maxRange:
                trueAccess = 0
        if cone == 1:
            trueAccess = 0

        if accessMsg.hasAccess != trueAccess:
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed access test.")

        if accessMsg.slantRange <= 1e-6:
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed positive slant range test.")
    if (latitude == 65 and defaultPolarRadius) or latitude == 115:
        # should not have access
        if accessMsg.hasAccess != 0:
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed negative have access test.")

        if np.abs(accessMsg.slantRange) > 1e-6:
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed negative slant range test.")

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, ''.join(testMessages)]


def test_spacecraftLocationColinearAccess(show_plots):
    """
    Regression test for issues #644 / #946.

    Two spacecraft that are radially aligned (on the same ray from the planet
    center) at different altitudes have an unobstructed line of sight, so
    ``hasAccess`` must be 1.

    Before the #946 fix (commit ``d432682135``), ``rClose`` -- the closest point
    on the inter-spacecraft segment to the planet center -- was computed from the
    *unclamped* line parameter. For a radial alignment that parameter is negative,
    placing the "closest point" at the planet center (``rClose`` ~ 0), so the
    ``rClose.norm() > rEquator`` gate failed and access was wrongly denied.

    This geometry keeps every other access pathway neutral -- no planet message
    (identity orientation at the origin), a spherical planet so ``zScale == 1``,
    no range cap, no boresight cone, and no sun message -- which isolates the
    ``rClose`` computation. The existing :func:`test_spacecraftLocation` cases
    cannot catch this regression: their equal-radius geometry holds the line
    parameter at ~0.5, inside ``[0, 1]``, where the clamp is a no-op.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("simProcess")
    dynProcess.addTask(scSim.CreateNewTask("simTask", macros.sec2nano(1.)))

    module = spacecraftLocation.SpacecraftLocation()
    module.ModelTag = "scLocationColinear"
    module.rEquator = orbitalMotion.REQ_EARTH * 1000.  # [m]
    # rPolar left unset -> spherical planet (zScale = 1); no range cap, cone, or sun.
    scSim.AddModelToTask("simTask", module)

    # primary spacecraft on the +x axis, 1000 km altitude
    rPrimary = orbitalMotion.REQ_EARTH * 1000. + 1000.0e3  # [m]
    primaryMsgData = messaging.SCStatesMsgPayload()
    primaryMsgData.r_BN_N = [rPrimary, 0.0, 0.0]  # [m]
    primaryMsg = messaging.SCStatesMsg().write(primaryMsgData)
    module.primaryScStateInMsg.subscribeTo(primaryMsg)

    # other spacecraft farther out on the SAME ray -> unobstructed line of sight
    rOther = orbitalMotion.REQ_EARTH * 1000. + 2000.0e3  # [m]
    otherMsgData = messaging.SCStatesMsgPayload()
    otherMsgData.r_BN_N = [rOther, 0.0, 0.0]  # [m]
    otherMsg = messaging.SCStatesMsg().write(otherMsgData)
    module.addSpacecraftToModel(otherMsg)

    scSim.InitializeSimulation()
    scSim.TotalSim.SingleStepProcesses()

    accessMsg = module.accessOutMsgs[0].read()

    assert accessMsg.hasAccess == 1, (
        "radially-aligned spacecraft must have line-of-sight access (issues #644/#946); "
        "hasAccess=0 indicates the rClose clamp-ordering regression"
    )
    # for a pure radial separation the slant range is just the altitude difference
    np.testing.assert_allclose(accessMsg.slantRange, rOther - rPrimary, rtol=1e-6)


@pytest.mark.parametrize(
    "caseName, sunPolarAngle, sunAzimuth, viewPolarAngle, viewAzimuth, expectedGlareFactor, "
    "glareThreshold, expectedHasGlare, useGlareConstraint, expectedAccess",
    [
        (
            "normal_incidence",
            0.0,  # [rad]
            0.0,  # [rad]
            0.0,  # [rad]
            0.0,  # [rad]
            1.0,  # [-]
            0.95,  # [-]
            1,  # [-]
            True,
            0,  # [-]
        ),
        (
            "exact_oblique",
            np.pi / 6,  # [rad]
            0.0,  # [rad]
            np.pi / 6,  # [rad]
            np.pi,  # [rad]
            1.0,  # [-]
            0.95,  # [-]
            1,  # [-]
            False,
            1,  # [-]
        ),
        (
            "near_miss",
            np.pi / 6,  # [rad]
            0.0,  # [rad]
            np.pi / 18,  # [rad]
            np.pi,  # [rad]
            np.cos(np.pi / 9),  # [-]
            0.95,  # [-]
            0,  # [-]
            True,
            1,  # [-]
        ),
        (
            "out_of_plane",
            np.pi / 6,  # [rad]
            0.0,  # [rad]
            np.pi / 6,  # [rad]
            np.pi / 2,  # [rad]
            0.75,  # [-]
            0.95,  # [-]
            0,  # [-]
            True,
            1,  # [-]
        ),
        (
            "back_facing_sun",
            2 * np.pi / 3,  # [rad]
            0.0,  # [rad]
            0.0,  # [rad]
            0.0,  # [rad]
            0.0,  # [-]
            0.95,  # [-]
            0,  # [-]
            True,
            1,  # [-]
        ),
        (
            "back_facing_sun_zero_threshold",
            2 * np.pi / 3,  # [rad]
            0.0,  # [rad]
            0.0,  # [rad]
            0.0,  # [rad]
            0.0,  # [-]
            0.0,  # [-]
            0,  # [-]
            True,
            1,  # [-]
        ),
        (
            "back_facing_view_zero_threshold",
            0.0,  # [rad]
            0.0,  # [rad]
            2 * np.pi / 3,  # [rad]
            0.0,  # [rad]
            0.0,  # [-]
            0.0,  # [-]
            0,  # [-]
            True,
            1,  # [-]
        ),
        (
            "threshold_boundary",
            0.0,  # [rad]
            0.0,  # [rad]
            np.arccos(0.95),  # [rad]
            0.0,  # [rad]
            0.95,  # [-]
            0.95,  # [-]
            1,  # [-]
            False,
            1,  # [-]
        ),
    ],
)
def test_spacecraftLocationGlare(
        caseName,
        sunPolarAngle,
        sunAzimuth,
        viewPolarAngle,
        viewAzimuth,
        expectedGlareFactor,
        glareThreshold,
        expectedHasGlare,
        useGlareConstraint,
        expectedAccess):
    """Verify specular-glare geometry and its optional access constraint.

    Validation Test Description
    ---------------------------
    The test places an offset body-fixed location far from the inertial origin,
    then supplies point-to-Sun and point-to-viewer directions with known mirror
    geometry. This verifies the glare factor, threshold flag, access behavior,
    and the use of the body-fixed location as the origin of both directions.

    Test Parameter Discussion
    -------------------------
    The cases cover normal and oblique exact reflections, an in-plane near
    miss, an out-of-plane view, Sun and observer directions behind the surface,
    the exact threshold boundary, and zero-threshold back-facing geometry.
    ``useGlareConstraint`` also verifies that glare rejection is opt-in.

    Expected Results
    ----------------
    Exact reflections produce a glare factor of one. Off-specular and
    back-facing cases produce the analytical factor supplied by the test and
    do not meet the configured glare threshold. Back-facing geometry is never
    classified as glare, including when the threshold is zero.
    """
    def direction_from_normal(polarAngle, azimuth):
        """Return a unit direction measured from the positive x-axis.

        :param polarAngle: Polar angle from the surface normal in radians.
        :param azimuth: Azimuth about the surface normal in radians.
        :return: Unit direction vector.
        """
        return np.array([
            np.cos(polarAngle),
            np.sin(polarAngle) * np.cos(azimuth),
            np.sin(polarAngle) * np.sin(azimuth),
        ])

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("simProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    dynProcess.addTask(scSim.CreateNewTask("simTask", simulationTimeStep))

    module = spacecraftLocation.SpacecraftLocation()
    module.ModelTag = "scLocationGlare_" + caseName
    module.rEquator = 1.0  # [m]
    module.rPolar = 1.0  # [m]
    locationOffset_B = np.array([2.0, -3.0, 1.0])  # [m]
    module.r_LB_B = locationOffset_B
    module.aHat_B = [1.0, 0.0, 0.0]  # [-]
    module.theta = np.pi  # [rad]
    module.glareThreshold = glareThreshold
    module.useGlareConstraint = useGlareConstraint
    scSim.AddModelToTask("simTask", module)

    primaryPosition_N = np.array([1.0e9, -2.0e9, 3.0e9])  # [m]
    primaryMsgData = messaging.SCStatesMsgPayload()
    primaryMsgData.r_BN_N = primaryPosition_N
    primaryMsg = messaging.SCStatesMsg().write(primaryMsgData)
    module.primaryScStateInMsg.subscribeTo(primaryMsg)

    locationPosition_N = primaryPosition_N + locationOffset_B  # [m]
    sunDirection_N = direction_from_normal(sunPolarAngle, sunAzimuth)
    sunDistance = 1.0e6  # [m]
    sunMsgData = messaging.SpicePlanetStateMsgPayload()
    sunMsgData.PositionVector = locationPosition_N + sunDistance * sunDirection_N
    sunMsg = messaging.SpicePlanetStateMsg().write(sunMsgData)
    module.sunInMsg.subscribeTo(sunMsg)

    viewDirection_N = direction_from_normal(viewPolarAngle, viewAzimuth)
    viewingDistance = 1.0e3  # [m]
    otherMsgData = messaging.SCStatesMsgPayload()
    otherMsgData.r_BN_N = locationPosition_N + viewingDistance * viewDirection_N
    otherMsg = messaging.SCStatesMsg().write(otherMsgData)
    module.addSpacecraftToModel(otherMsg)

    scSim.InitializeSimulation()
    scSim.TotalSim.SingleStepProcesses()
    accessMsg = module.accessOutMsgs[0].read()

    np.testing.assert_allclose(accessMsg.sunIncidenceAngle, sunPolarAngle, atol=1e-10)
    np.testing.assert_allclose(accessMsg.scViewAngle, viewPolarAngle, atol=1e-10)
    np.testing.assert_allclose(accessMsg.glareFactor, expectedGlareFactor, atol=1e-10)
    assert accessMsg.hasGlare == expectedHasGlare
    assert accessMsg.hasAccess == expectedAccess


@pytest.mark.parametrize(
    "incidence_angle",
    np.deg2rad([0.0, 2.0, 25.0, 43.0, 65.0, 89.0]),  # [rad]
)
@pytest.mark.parametrize(
    "sigma_BN", [[0.0, 0.0, 0.0], [0.1, -0.2, 0.3]],  # [-]
)
@pytest.mark.parametrize("use_glare_constraint", [False, True])
@pytest.mark.parametrize(
    "view_offset, glare_threshold, expected_glare",
    [  # Viewing offset [rad], glare threshold [-], expected glare flag.
        pytest.param(0.0, 1.0, 1, id="exact_reflection"),
        pytest.param(1.0e-6, 1.0, 0, id="near_exact_reflection"),
        pytest.param(np.arccos(0.95), 0.95, 1, id="threshold_boundary"),
        pytest.param(np.arccos(0.95) + 1.0e-6, 0.95, 0, id="below_threshold"),
    ],
)
def test_spacecraft_location_glare_roundoff(
        incidence_angle, sigma_BN, use_glare_constraint,
        view_offset, glare_threshold, expected_glare):
    """Verify glare thresholds at and just outside exact reflection geometry.

    Validation Test Description
    ---------------------------
    Construct the Sun direction and its ideal mirror reflection in the body
    frame, then rotate both into the inertial frame about an offset surface
    point. Shift the observer by a known angle from the ideal reflection.

    Test Parameter Discussion
    -------------------------
    Incidence angles span normal through nearly grazing illumination, with
    identity and nonzero spacecraft attitudes. The cases exercise exact
    reflection at threshold one, the default threshold boundary, and views
    one microradian beyond each boundary. Glare rejection is enabled and
    disabled independently.

    Expected Results
    ----------------
    The glare factor equals the cosine of the reflected-ray viewing offset.
    Roundoff must not prevent boundary views from being classified as glare.
    Nearby views outside the boundary retain access, and detected glare only
    prevents access when the constraint is enabled.
    """
    module = spacecraftLocation.SpacecraftLocation()
    module.rEquator = 1.0  # [m]
    module.aHat_B = [1.0, 0.0, 0.0]  # [-]
    location_offset_B = np.array([2.0, -3.0, 1.0])  # [m]
    module.r_LB_B = location_offset_B
    module.theta = np.pi  # [rad]
    module.glareThreshold = glare_threshold
    module.useGlareConstraint = use_glare_constraint

    primary_position_N = np.array([100.0, 200.0, -300.0])  # [m]
    primary_payload = messaging.SCStatesMsgPayload()
    primary_payload.r_BN_N = primary_position_N
    primary_payload.sigma_BN = sigma_BN
    primary_msg = messaging.SCStatesMsg().write(primary_payload)
    module.primaryScStateInMsg.subscribeTo(primary_msg)

    dcm_NB = rbk.MRP2C(sigma_BN).T
    location_position_N = primary_position_N + dcm_NB @ location_offset_B
    sun_direction_B = np.array([
        np.cos(incidence_angle), np.sin(incidence_angle), 0.0,
    ])
    sun_distance = 1.0e6  # [m]
    sun_payload = messaging.SpicePlanetStateMsgPayload()
    sun_payload.PositionVector = (
        location_position_N + sun_distance * (dcm_NB @ sun_direction_B)
    )
    sun_msg = messaging.SpicePlanetStateMsg().write(sun_payload)
    module.sunInMsg.subscribeTo(sun_msg)

    view_direction_B = np.array([
        np.cos(incidence_angle - view_offset),
        -np.sin(incidence_angle - view_offset),
        0.0,
    ])
    viewing_distance = 1.0e3  # [m]
    other_payload = messaging.SCStatesMsgPayload()
    other_payload.r_BN_N = (
        location_position_N + viewing_distance * (dcm_NB @ view_direction_B)
    )
    other_msg = messaging.SCStatesMsg().write(other_payload)
    module.addSpacecraftToModel(other_msg)

    module.Reset(0)
    module.UpdateState(0)
    access_msg = module.accessOutMsgs[0].read()

    np.testing.assert_allclose(
        access_msg.glareFactor, np.cos(view_offset), rtol=0.0, atol=2.0e-14,
    )
    assert access_msg.hasGlare == expected_glare
    expected_access = int(not (use_glare_constraint and expected_glare))
    assert access_msg.hasAccess == expected_access


@pytest.mark.parametrize(
    "viewAngle, viewAngleLimit, connectSun, expectedAccess",
    [
        (np.pi / 6, np.pi / 4, False, 1),  # [rad], [rad], [-]
        (np.pi / 3, np.pi / 4, False, 0),  # [rad], [rad], [-]
        (0.0, 0.0, True, 1),  # [rad], [rad], [-]
        (np.pi / 3, np.pi / 4, True, 0),  # [rad], [rad], [-]
        (np.pi / 3, -1.0, False, 1),  # [rad], [rad], [-]
    ],
)
def test_spacecraftLocationViewAngleRequirement(
        viewAngle, viewAngleLimit, connectSun, expectedAccess):
    """Verify the optional surface-normal-to-observer access requirement.

    Validation Test Description
    ---------------------------
    Place the tracked spacecraft at a known angle from the body-fixed normal,
    with and without the optional Sun message.

    Test Parameter Discussion
    -------------------------
    The cases cover accepted and rejected views, the exact limit, and the
    negative default that disables the requirement.

    Expected Results
    ----------------
    ``scViewAngle`` reports the viewing geometry even without a Sun message.
    Access is denied only when the enabled viewing-angle limit is exceeded.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("simProcess")
    dynProcess.addTask(scSim.CreateNewTask("simTask", macros.sec2nano(1.0)))  # [s]

    module = spacecraftLocation.SpacecraftLocation()
    module.ModelTag = "scLocationViewAngle"
    module.rEquator = 1.0  # [m]
    module.rPolar = 1.0  # [m]
    module.aHat_B = [1.0, 0.0, 0.0]  # [-]
    module.theta = np.pi  # [rad]
    module.theta_view = viewAngleLimit
    scSim.AddModelToTask("simTask", module)

    locationPosition_N = np.array([1.0e9, 0.0, 0.0])  # [m]
    primaryMsgData = messaging.SCStatesMsgPayload()
    primaryMsgData.r_BN_N = locationPosition_N
    primaryMsg = messaging.SCStatesMsg().write(primaryMsgData)
    module.primaryScStateInMsg.subscribeTo(primaryMsg)

    viewingDistance = 1.0e3  # [m]
    viewDirection_N = np.array([np.cos(viewAngle), np.sin(viewAngle), 0.0])  # [-]
    otherMsgData = messaging.SCStatesMsgPayload()
    otherMsgData.r_BN_N = locationPosition_N + viewingDistance * viewDirection_N
    otherMsg = messaging.SCStatesMsg().write(otherMsgData)
    module.addSpacecraftToModel(otherMsg)

    if connectSun:
        sunMsgData = messaging.SpicePlanetStateMsgPayload()
        sunMsgData.PositionVector = locationPosition_N + np.array([1.0e6, 0.0, 0.0])  # [m]
        sunMsg = messaging.SpicePlanetStateMsg().write(sunMsgData)
        module.sunInMsg.subscribeTo(sunMsg)

    scSim.InitializeSimulation()
    scSim.TotalSim.SingleStepProcesses()
    accessMsg = module.accessOutMsgs[0].read()

    np.testing.assert_allclose(accessMsg.scViewAngle, viewAngle, atol=1e-10)
    assert accessMsg.hasAccess == expectedAccess
    if not connectSun:
        assert accessMsg.hasIllumination == 0
        assert accessMsg.glareFactor == 0.0
        assert accessMsg.hasGlare == 0


def test_spacecraftLocationViewAngleRequiresNormal():
    """Verify that an enabled viewing-angle limit requires a surface normal.

    Validation Test Description
    ---------------------------
    Initialize a module with a viewing-angle limit but no ``aHat_B``.

    Test Parameter Discussion
    -------------------------
    A positive limit enables the otherwise optional viewing-angle check.

    Expected Results
    ----------------
    ``Reset()`` emits ``BSK_ERROR`` and initialization stops.
    """
    module = spacecraftLocation.SpacecraftLocation()
    module.rEquator = 1.0  # [m]
    module.theta_view = np.pi / 4  # [rad]

    primaryMsg = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    otherMsg = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    module.primaryScStateInMsg.subscribeTo(primaryMsg)
    module.addSpacecraftToModel(otherMsg)

    with pytest.raises(BasiliskError):
        module.Reset(0)


@pytest.mark.parametrize(
    "caseName, sunPolarAngle, solarAngleLimit, eclipseFactor, minimumIllumination, "
    "connectSun, connectEclipse, expectedIllumination, expectedAccess",
    [
        (
            "inside_solar_angle",
            np.pi / 6,  # [rad]
            np.pi / 4,  # [rad]
            1.0,  # [-]
            0.5,  # [-]
            True,
            False,
            1,  # [-]
            1,  # [-]
        ),
        (
            "outside_solar_angle",
            np.pi / 3,  # [rad]
            np.pi / 4,  # [rad]
            1.0,  # [-]
            0.5,  # [-]
            True,
            False,
            0,  # [-]
            0,  # [-]
        ),
        (
            "eclipse_above_threshold",
            np.pi / 6,  # [rad]
            np.pi / 4,  # [rad]
            0.75,  # [-]
            0.5,  # [-]
            True,
            True,
            1,  # [-]
            1,  # [-]
        ),
        (
            "eclipse_below_threshold",
            np.pi / 6,  # [rad]
            np.pi / 4,  # [rad]
            0.25,  # [-]
            0.5,  # [-]
            True,
            True,
            0,  # [-]
            0,  # [-]
        ),
        (
            "eclipse_optional",
            np.pi / 6,  # [rad]
            np.pi / 4,  # [rad]
            0.0,  # [-]
            0.5,  # [-]
            True,
            False,
            1,  # [-]
            1,  # [-]
        ),
        (
            "sun_optional",
            0.0,  # [rad]
            -1.0,  # [rad]
            1.0,  # [-]
            -1.0,  # [-]
            False,
            False,
            0,  # [-]
            1,  # [-]
        ),
    ],
)
def test_spacecraftLocationIlluminationRequirements(
        caseName,
        sunPolarAngle,
        solarAngleLimit,
        eclipseFactor,
        minimumIllumination,
        connectSun,
        connectEclipse,
        expectedIllumination,
        expectedAccess):
    """Verify the optional illumination-angle and eclipse requirements.

    Validation Test Description
    ---------------------------
    The test provides a surface point, its normal, a tracked imaging
    spacecraft, and optional Sun and eclipse messages. It evaluates the two
    illumination gates independently and together.

    Test Parameter Discussion
    -------------------------
    The cases place the Sun inside or outside ``theta_solar``, put the eclipse
    illumination factor above or below ``min_illumination_factor``, omit the
    optional eclipse input, and omit the optional Sun input.

    Expected Results
    ----------------
    An illuminated point retains access only when every configured and
    connected illumination requirement is satisfied. Without a Sun message,
    illumination and glare remain unreported while geometric access is
    unchanged.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("simProcess")
    simulationTimeStep = macros.sec2nano(1.0)  # [ns]
    dynProcess.addTask(scSim.CreateNewTask("simTask", simulationTimeStep))

    module = spacecraftLocation.SpacecraftLocation()
    module.ModelTag = "scLocationIllumination_" + caseName
    module.rEquator = 1.0  # [m]
    module.rPolar = 1.0  # [m]
    module.aHat_B = [1.0, 0.0, 0.0]  # [-]
    module.theta = np.pi / 2  # [rad]
    module.theta_solar = solarAngleLimit
    module.min_illumination_factor = minimumIllumination
    scSim.AddModelToTask("simTask", module)

    locationPosition_N = np.array([1.0e9, 0.0, 0.0])  # [m]
    primaryMsgData = messaging.SCStatesMsgPayload()
    primaryMsgData.r_BN_N = locationPosition_N
    primaryMsg = messaging.SCStatesMsg().write(primaryMsgData)
    module.primaryScStateInMsg.subscribeTo(primaryMsg)

    otherMsgData = messaging.SCStatesMsgPayload()
    otherMsgData.r_BN_N = locationPosition_N + np.array([1.0e3, 0.0, 0.0])  # [m]
    otherMsg = messaging.SCStatesMsg().write(otherMsgData)
    module.addSpacecraftToModel(otherMsg)

    if connectSun:
        sunDirection_N = np.array([
            np.cos(sunPolarAngle),
            np.sin(sunPolarAngle),
            0.0,
        ])
        sunMsgData = messaging.SpicePlanetStateMsgPayload()
        sunMsgData.PositionVector = locationPosition_N + 1.0e6 * sunDirection_N  # [m]
        sunMsg = messaging.SpicePlanetStateMsg().write(sunMsgData)
        module.sunInMsg.subscribeTo(sunMsg)

    if connectEclipse:
        eclipseMsgData = messaging.EclipseMsgPayload()
        eclipseMsgData.illuminationFactor = eclipseFactor
        eclipseMsg = messaging.EclipseMsg().write(eclipseMsgData)
        module.eclipseInMsg.subscribeTo(eclipseMsg)

    scSim.InitializeSimulation()
    scSim.TotalSim.SingleStepProcesses()
    accessMsg = module.accessOutMsgs[0].read()

    assert accessMsg.hasIllumination == expectedIllumination
    assert accessMsg.hasAccess == expectedAccess
    if connectSun:
        np.testing.assert_allclose(accessMsg.sunIncidenceAngle, sunPolarAngle, atol=1e-10)
    else:
        assert accessMsg.hasGlare == 0
        assert accessMsg.glareFactor == 0.0
        assert accessMsg.scViewAngle == 0.0


@pytest.mark.parametrize("glareThreshold", [-0.01, 1.01])
def test_spacecraftLocationGlareThresholdValidation(glareThreshold):
    """Verify that the geometric glare threshold is limited to [0, 1].

    Validation Test Description
    ---------------------------
    The test initializes an otherwise valid module with a glare threshold
    outside the supported dimensionless alignment range.

    Test Parameter Discussion
    -------------------------
    Values immediately below zero and above one exercise both invalid bounds.

    Expected Results
    ----------------
    ``Reset()`` emits ``BSK_ERROR``, which raises ``BasiliskError`` and stops
    initialization.
    """
    module = spacecraftLocation.SpacecraftLocation()
    module.rEquator = 1.0  # [m]
    module.glareThreshold = glareThreshold

    primaryMsg = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    otherMsg = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    module.primaryScStateInMsg.subscribeTo(primaryMsg)
    module.addSpacecraftToModel(otherMsg)

    with pytest.raises(BasiliskError):
        module.Reset(0)


if __name__ == '__main__':
    run(False
        , False      # defaultPolarRadius
        , True     # defaultPlanet
        , 55        # true latitude angle (deg)
        , -7000.*1000 # max range
        , 1            # cone case, 0-> no cone, 1 -> [0, 1, 0], -1 -> [0, -1, 0]
        )
    test_spacecraftLocationColinearAccess(False)
