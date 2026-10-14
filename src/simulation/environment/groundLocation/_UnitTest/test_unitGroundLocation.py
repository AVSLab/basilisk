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
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.simulation import groundLocation
from Basilisk.simulation import spacecraft
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simHelpers
from matplotlib import pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = __path__[0]


def test_range(show_plots):
    """
    Tests whether groundLocation:

    1. Computes range correctly by evaluating slantRange;
    2. Tests whether elevation is correctly evaluated;
    3. Tests whether range limits impact access.
    4. Tests whether multiple spacecraft are supported in parallel;
    5. Checks that unmodeled illumination and glare outputs remain zero for
       every spacecraft at every recorded time step.

    :return:
    """

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTime = macros.sec2nano(10.)
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #   Initialize new atmosphere and drag model, add them to task
    groundTarget = groundLocation.GroundLocation()
    groundTarget.ModelTag = "groundTarget"
    groundTarget.planetRadius = orbitalMotion.REQ_EARTH * 1000.
    groundTarget.maximumRange = 100e3 # meters
    groundTarget.minimumElevation = np.radians(80.)
    groundTarget.specifyLocation(np.radians(0.), np.radians(0.), 0.)
    scSim.AddModelToTask(simTaskName, groundTarget)

    #   Write out mock planet rotation, spacecraft position messages
    sc1_message = messaging.SCStatesMsgPayload()
    sc1_message.r_BN_N = [orbitalMotion.REQ_EARTH*1e3 + 100e3, 0, 0]  # SC1 is in range
    sc1Msg = messaging.SCStatesMsg().write(sc1_message)

    sc2_message = messaging.SCStatesMsgPayload()
    #   SC2 is placed inside/outside the visibility cone for the ground station
    sc2_message.r_BN_N = [orbitalMotion.REQ_EARTH*1e3 + 101e3,0, 0]
    sc2Msg = messaging.SCStatesMsg().write(sc2_message)

    sc3_message = messaging.SCStatesMsgPayload()
    #   SC3 is inside the altitude limit,  but outside the visibility cone
    sc3_message.r_BN_N = rbk.euler3(np.radians(11.)).dot(np.array([100e3, 0, 0])) + np.array(
        [orbitalMotion.REQ_EARTH * 1e3, 0, 0])
    sc3Msg = messaging.SCStatesMsg().write(sc3_message)

    groundTarget.addSpacecraftToModel(sc1Msg)
    groundTarget.addSpacecraftToModel(sc2Msg)
    groundTarget.addSpacecraftToModel(sc3Msg)

    # Log the access indicator
    numDataPoints = 2
    samplingTime = simHelpers.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog0 = groundTarget.accessOutMsgs[0].recorder(samplingTime)
    dataLog1 = groundTarget.accessOutMsgs[1].recorder(samplingTime)
    dataLog2 = groundTarget.accessOutMsgs[2].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog0)
    scSim.AddModelToTask(simTaskName, dataLog1)
    scSim.AddModelToTask(simTaskName, dataLog2)

    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Illumination and glare are not computed by this module.
    for access_log in (dataLog0, dataLog1, dataLog2):
        for field_name in ("hasIllumination", "sunIncidenceAngle", "scViewAngle", "hasGlare", "glareFactor"):
            np.testing.assert_array_equal(getattr(access_log, field_name), 0)

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

    #   Compare to expected values
    accuracy = 1e-8
    ref_ranges = [100e3, 101e3, 100e3]
    ref_elevation = [np.radians(90.), np.radians(90.), np.radians(79.)]
    ref_access = [1, 0, 0]

    test_ranges = [sc1_slant[1], sc2_slant[1], sc3_slant[1]]
    test_elevation = [sc1_elevation[1],sc2_elevation[1],sc3_elevation[1]]
    test_access = [sc1_access[1],sc2_access[1],sc3_access[1]]

    range_worked = test_ranges == pytest.approx(ref_ranges, accuracy)
    elevation_worked = test_elevation == pytest.approx(ref_elevation, accuracy)
    access_worked = test_access == pytest.approx(ref_access, abs=1e-16)

    assert (range_worked and elevation_worked and access_worked)


def test_rotation(show_plots):
    """
    Tests whether groundLocation:

    1. Computes the current location based on the initial position and the rotation rate of the planet
       it is attached to.

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

    #   Initialize new atmosphere and drag model, add them to task
    groundTarget = groundLocation.GroundLocation()
    groundTarget.ModelTag = "groundTarget"
    groundTarget.planetRadius = orbitalMotion.REQ_EARTH * 1000.
    groundTarget.maximumRange = 200e3 # meters
    groundTarget.minimumElevation = np.radians(10.)
    groundTarget.specifyLocation(np.radians(0.), np.radians(10.), 0.)
    scSim.AddModelToTask(simTaskName, groundTarget)

    #   Write out mock planet rotation, spacecraft position messages
    sc1_message = messaging.SCStatesMsgPayload()
    sc1_message.r_BN_N = np.array([orbitalMotion.REQ_EARTH*1e3 + 90e3, 0, 0])  # SC1 is in range
    scMsg = messaging.SCStatesMsg().write(sc1_message)
    groundTarget.addSpacecraftToModel(scMsg)

    planet_message = messaging.SpicePlanetStateMsgPayload()
    planet_message.J20002Pfix = rbk.euler3(np.radians(-10.)).tolist()
    planetMsg = messaging.SpicePlanetStateMsg().write(planet_message)
    groundTarget.planetInMsg.subscribeTo(planetMsg)

    # Log the access indicator
    numDataPoints = 2
    samplingTime = simHelpers.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = groundTarget.accessOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)

    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Get the logged data
    sc1_access = dataLog.hasAccess
    sc1_slant = dataLog.slantRange
    sc1_elevation = dataLog.elevation
    sc1_azimuth = dataLog.azimuth

    #   Compare to expected values
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

    assert (range_worked and elevation_worked and access_worked)

def test_AzElR_rates():
    """
    Tests that the Az,El,range rates are correct by using 1-step Euler integration

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
    gravFactory.createSpiceInterface(time=timeInitString)
    gravFactory.spiceObject.zeroBase = 'Earth'
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject, -1)

    scObject = spacecraft.Spacecraft()
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    scSim.AddModelToTask(simTaskName, scObject)
    oe = orbitalMotion.ClassicElements()
    r_sc = planet.radEquator + 100*1E3
    oe.a = r_sc
    oe.e = 0.00001
    oe.i = 53.0*macros.D2R
    oe.Omega = 115.0*macros.D2R
    oe.omega = 5.0*macros.D2R
    oe.f = 75.*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN

    groundStation = groundLocation.GroundLocation()
    groundStation.planetRadius = planet.radEquator
    groundStation.specifyLocation(np.radians(40.009971), np.radians(-105.243895), 1624)
    groundStation.planetInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    groundStation.minimumElevation = np.radians(60.)
    groundStation.addSpacecraftToModel(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, groundStation)

    # Log the Az,El,R and rates info
    numDataPoints = 2
    samplingTime = simHelpers.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = groundStation.accessOutMsgs[0].recorder(samplingTime)
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

    # Euler integration
    sc_Euler_range = sc_range[0] + sc_range_rate[0]*dt
    sc_Euler_elev = sc_elevation[0] + sc_el_rate[0]*dt
    sc_Euler_azimuth = sc_azimuth[0] + sc_az_rate[0]*dt

    range_rate_worked = sc_range[1] == pytest.approx(sc_Euler_range, rel=1e-5)
    el_rate_worked = sc_elevation[1] == pytest.approx(sc_Euler_elev, rel=1e-5)
    az_rate_worked = sc_azimuth[1] == pytest.approx(sc_Euler_azimuth, rel=1e-5)

    assert (range_rate_worked and el_rate_worked and az_rate_worked)


@pytest.mark.parametrize("tilt_deg", [0.0, 30.0, 60.0])
@pytest.mark.parametrize("v_PN_N", [[0.0, 0.0, 0.0], [3.0e4, -1.0e3, 2.0e3]])  # [m/s]
def test_planetFixedSpacecraftZeroRates(tilt_deg, v_PN_N):
    """
    Tests that a spacecraft fixed in the planet frame has zero range, azimuth and elevation rates and zero
    velocity relative to the ground location, when the planet spin axis is tilted away from the inertial
    3-axis.

    :param tilt_deg: [deg] angle between the planet spin axis and the inertial 3-axis
    :param v_PN_N: [m/s] planet velocity relative to the inertial frame origin, in N components
    :return:
    """
    radius = 6378136.3  # [m] planet radius
    omega = 7.2921159e-5  # [rad/s] planet spin rate
    tilt = np.radians(tilt_deg)  # [rad] angle between the planet spin axis and the inertial 3-axis

    def skew(v):
        return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

    c, s = np.cos(tilt), np.sin(tilt)
    dcm_PN = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])  # [-] N to P, the P 3-axis is the spin axis
    w_PN_P = np.array([0.0, 0.0, omega])  # [rad/s] angular velocity of P relative to N, in P components
    w_PN_N = dcm_PN.T @ w_PN_P  # [rad/s] same vector in N components
    dcm_PN_dot = -skew(w_PN_P) @ dcm_PN  # [1/s] time derivative of the N to P DCM

    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.VelocityVector = v_PN_N
    planetPayload.J20002Pfix = dcm_PN.tolist()
    planetPayload.J20002Pfix_dot = dcm_PN_dot.tolist()
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload)

    r_BN_N = np.array([radius + 600e3, 0.0, 1.0e6])  # [m] spacecraft position, fixed in P
    scPayload = messaging.SCStatesMsgPayload()
    scPayload.r_BN_N = r_BN_N.tolist()
    scPayload.v_BN_N = (np.array(v_PN_N) + np.cross(w_PN_N, r_BN_N)).tolist()  # [m/s]
    scMsg = messaging.SCStatesMsg().write(scPayload)

    scSim = SimulationBaseClass.SimBaseClass()
    scSim.CreateNewProcess("simProcess").addTask(scSim.CreateNewTask("simTask", macros.sec2nano(1.0)))
    groundStation = groundLocation.GroundLocation()
    groundStation.planetRadius = radius
    groundStation.minimumElevation = -np.pi / 2
    groundStation.specifyLocation(np.radians(10.0), np.radians(20.0), 0.0)
    groundStation.planetInMsg.subscribeTo(planetMsg)
    groundStation.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("simTask", groundStation)
    dataLog = groundStation.accessOutMsgs[0].recorder()
    scSim.AddModelToTask("simTask", dataLog)

    scSim.InitializeSimulation()
    scSim.TotalSim.SingleStepProcesses()

    np.testing.assert_allclose(dataLog.v_BL_L[0], np.zeros(3), atol=1e-6)  # [m/s]
    assert dataLog.range_dot[0] == pytest.approx(0.0, abs=1e-6)  # [m/s]
    assert dataLog.az_dot[0] == pytest.approx(0.0, abs=1e-11)  # [rad/s]
    assert dataLog.el_dot[0] == pytest.approx(0.0, abs=1e-11)  # [rad/s]


@pytest.mark.parametrize("tilt_deg", [0.0, 30.0, 60.0])
@pytest.mark.parametrize("v_PN_N", [[0.0, 0.0, 0.0], [3.0e4, -1.0e3, 2.0e3]])  # [m/s]
def test_overheadSpacecraft(tilt_deg, v_PN_N):
    """
    Tests that a spacecraft exactly overhead of the ground location, with a tilted planet frame, a planet far from
    the inertial origin and a nonzero relative velocity, gets zero azimuth, azimuth rate and elevation rate despite
    the floating-point round-off left in the horizontal components of the SEZ position.

    :param tilt_deg: [deg] angle between the planet spin axis and the inertial 3-axis
    :param v_PN_N: [m/s] planet velocity relative to the inertial frame origin, in N components
    :return:
    """
    radius = 6378136.3  # [m] planet radius
    altitude = 600e3  # [m] spacecraft altitude above the ground location
    omega = 7.2921159e-5  # [rad/s] planet spin rate
    r_PN_N = np.array([1.5e11, -2.0e10, 5.0e9])  # [m] planet position relative to the inertial origin
    lat, lon = np.radians(35.0), np.radians(-70.0)  # [rad] ground location latitude and longitude
    tilt = np.radians(tilt_deg)  # [rad]
    c, s = np.cos(tilt), np.sin(tilt)
    dcm_PN = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])  # [-] N to P
    w_PN_P = np.array([0.0, 0.0, omega])  # [rad/s]
    w_skew = np.array([[0, -w_PN_P[2], w_PN_P[1]], [w_PN_P[2], 0, -w_PN_P[0]], [-w_PN_P[1], w_PN_P[0], 0]])

    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.PositionVector = list(r_PN_N)
    planetPayload.VelocityVector = list(v_PN_N)
    planetPayload.J20002Pfix = dcm_PN.tolist()
    planetPayload.J20002Pfix_dot = (-w_skew @ dcm_PN).tolist()
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload)

    rHat_P = np.array([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)])  # [-]
    r_BP_N = dcm_PN.T @ ((radius + altitude) * rHat_P)  # [m] exactly overhead
    # spacecraft moves relative to the planet, so the overhead singularity is hit with a nonzero velocity
    v_BN_N = np.array(v_PN_N) + np.cross(dcm_PN.T @ w_PN_P, r_BP_N) + dcm_PN.T @ np.array([1.0e3, -2.0e3, 5.0e2])  # [m/s]
    scPayload = messaging.SCStatesMsgPayload()
    scPayload.r_BN_N = (r_PN_N + r_BP_N).tolist()
    scPayload.v_BN_N = v_BN_N.tolist()
    scMsg = messaging.SCStatesMsg().write(scPayload)

    scSim = SimulationBaseClass.SimBaseClass()
    scSim.CreateNewProcess("simProcess").addTask(scSim.CreateNewTask("simTask", macros.sec2nano(1.0)))
    groundStation = groundLocation.GroundLocation()
    groundStation.planetRadius = radius
    groundStation.minimumElevation = -np.pi / 2
    groundStation.specifyLocation(lat, lon, 0.0)
    groundStation.planetInMsg.subscribeTo(planetMsg)
    groundStation.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("simTask", groundStation)
    dataLog = groundStation.accessOutMsgs[0].recorder()
    scSim.AddModelToTask("simTask", dataLog)

    scSim.InitializeSimulation()
    scSim.TotalSim.SingleStepProcesses()

    assert np.all(np.isfinite(dataLog.v_BL_L[0]))
    assert np.hypot(dataLog.r_BL_L[0][0], dataLog.r_BL_L[0][1]) < 1e-3  # [m] overhead up to round-off
    assert dataLog.azimuth[0] == 0.0  # [rad]
    assert dataLog.az_dot[0] == 0.0  # [rad/s]
    assert dataLog.el_dot[0] == 0.0  # [rad/s]
    assert dataLog.slantRange[0] == pytest.approx(altitude, rel=1e-6)  # [m]


def test_nonzeroRelativeStateOutputs():
    """
    Tests the published SEZ position, SEZ velocity, range rate and surface normal for a stationary planet, a ground
    location at zero latitude and longitude, and a moving spacecraft.

    :return:
    """
    radius = 6378136.3  # [m] planet radius
    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.J20002Pfix = np.eye(3).tolist()
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload)

    scPayload = messaging.SCStatesMsgPayload()
    scPayload.r_BN_N = [radius + 600e3, 1.0e5, 2.0e5]  # [m]
    scPayload.v_BN_N = [10.0, 20.0, 30.0]  # [m/s]
    scMsg = messaging.SCStatesMsg().write(scPayload)

    scSim = SimulationBaseClass.SimBaseClass()
    scSim.CreateNewProcess("simProcess").addTask(scSim.CreateNewTask("simTask", macros.sec2nano(1.0)))
    groundStation = groundLocation.GroundLocation()
    groundStation.planetRadius = radius
    groundStation.minimumElevation = -np.pi / 2
    groundStation.specifyLocation(0.0, 0.0, 0.0)
    groundStation.planetInMsg.subscribeTo(planetMsg)
    groundStation.addSpacecraftToModel(scMsg)
    scSim.AddModelToTask("simTask", groundStation)
    accessLog = groundStation.accessOutMsgs[0].recorder()
    stateLog = groundStation.currentGroundStateOutMsg.recorder()
    scSim.AddModelToTask("simTask", accessLog)
    scSim.AddModelToTask("simTask", stateLog)

    scSim.InitializeSimulation()
    scSim.TotalSim.SingleStepProcesses()

    r_BL_L_expected = np.array([-2.0e5, 1.0e5, 6.0e5])  # [m] SEZ components at lat = lon = 0
    v_BL_L_expected = np.array([-30.0, 20.0, 10.0])  # [m/s]
    np.testing.assert_allclose(accessLog.r_BL_L[0], r_BL_L_expected, atol=1e-6)  # [m]
    np.testing.assert_allclose(accessLog.v_BL_L[0], v_BL_L_expected, atol=1e-9)  # [m/s]
    assert accessLog.range_dot[0] == pytest.approx(
        v_BL_L_expected @ r_BL_L_expected / np.linalg.norm(r_BL_L_expected), rel=1e-9)  # [m/s]
    np.testing.assert_allclose(stateLog.rHat_LP_N[0], [1.0, 0.0, 0.0], atol=1e-12)  # [-]
    np.testing.assert_allclose(stateLog.r_LP_N[0], [radius, 0.0, 0.0], rtol=1e-12)  # [m]


def plot_geometry(groundLocation, scLocations, minimumElevation):
    """
    Plots the location of a ground station, its field of view,  and the positions of two spacecraft to verify whether
    the spacecraft have access to the ground station.

    :param groundLocation: [3,] : an ECI ground position.
    :param scLocations: [3,2] : two spacecraft position vectors
    :param minimumElevation: double : minimum view elevation angle in degrees.
    :return:
    """
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # draw sphere
    u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:20j]
    x = orbitalMotion.REQ_EARTH*1000 * np.cos(u) * np.sin(v)
    y = orbitalMotion.REQ_EARTH*1000 *np.sin(u) * np.sin(v)
    z = orbitalMotion.REQ_EARTH*1000 *np.cos(v)
    ax.plot_wireframe(x, y, z, color="g")

    # draw a point0
    ax.scatter(groundLocation[0],groundLocation[1],groundLocation[2], color="r", s=100)

    # draw a vector

    for location in scLocations:
        ax.scatter(location[0],location[1],location[2],color='k',s=100)

        ax.quiver(groundLocation[0],groundLocation[1],groundLocation[2],
              location[0],location[1],location[2], length=1.0, normalize=True)
    #ax.add_artist(a)

    plt.show()


if __name__ == '__main__':
    test_rotation(False)
    # test_range(True)
