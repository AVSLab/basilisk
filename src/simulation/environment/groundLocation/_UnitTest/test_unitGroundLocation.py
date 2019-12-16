
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''

import os, inspect
import numpy as np
import pytest
from matplotlib import pyplot as plt

from mpl_toolkits.mplot3d import Axes3D


from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import orbitalMotion
from Basilisk.simulation import simMessages
from Basilisk.simulation import groundLocation

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

def test_range(show_plots):
    '''
    Tests whether groundLocation:
        1. Computes range correctly by evaluating slantRange;
        2. Tests whether elevation is correctly evaluated;
        3. Tests whether range limits impact access.
        4. Tests whether multiple spacecraft are supported in parallel
    :return:
    '''
    testFailCount = 0
    testMessages = []

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()
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
    sc1_message = simMessages.SCPlusStatesSimMsg()
    sc1_message.r_BN_N = [orbitalMotion.REQ_EARTH*1e3 + 100e3, 0, 0]  # SC1 is in range
    sc1_message_name = "sc1_msg"

    sc2_message = simMessages.SCPlusStatesSimMsg()
    #   SC2 is placed inside/outside the visibility cone for the ground station
    sc2_message.r_BN_N = [orbitalMotion.REQ_EARTH*1e3 + 101e3,0, 0]
    sc2_message_name = "sc2_msg"

    sc3_message = simMessages.SCPlusStatesSimMsg()
    #   SC3 is inside the altitude limit,  but outside the visibility cone
    sc3_message.r_BN_N = rbk.euler3(np.radians(11.)).dot(np.array([orbitalMotion.REQ_EARTH * 1e3 + 100e3, 0, 0]))
    sc3_message_name = "sc3_msg"

    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, sc1_message_name, sc1_message)
    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, sc2_message_name, sc2_message)
    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, sc3_message_name, sc3_message)

    groundTarget.addSpacecraftToModel(sc1_message_name)
    groundTarget.addSpacecraftToModel(sc2_message_name)
    groundTarget.addSpacecraftToModel(sc3_message_name)

    # Log the access indicator
    numDataPoints = 2
    samplingTime = int(simulationTime / (numDataPoints - 1))
    scSim.TotalSim.logThisMessage(groundTarget.accessOutMsgNames[0], samplingTime)
    scSim.TotalSim.logThisMessage(groundTarget.accessOutMsgNames[1], samplingTime)
    scSim.TotalSim.logThisMessage(groundTarget.accessOutMsgNames[2], samplingTime)
    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()
    # Get the logged data
    sc1_access = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.hasAccess',range(1))
    sc1_slant = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.slantRange',range(1))
    sc1_elevation =scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.elevation',range(1))

    sc2_access = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[1] + '.hasAccess',range(1))
    sc2_slant = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[1] + '.slantRange',range(1))
    sc2_elevation = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[1] + '.elevation',range(1))

    sc3_access = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[2] + '.hasAccess', range(1))
    sc3_slant = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[2] + '.slantRange',range(1))
    sc3_elevation = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[2] + '.elevation',range(1))

    #   Compare to expected values
    accuracy = 1e-8
    ref_ranges = [100e3, 0, 0]
    ref_elevation = [np.radians(90.),0, 0]
    ref_access = [1, 0, 0]

    test_ranges = [sc1_slant[0,1], sc2_slant[0,1], sc3_slant[0,1]]
    test_elevation = [sc1_elevation[0,1],sc2_elevation[0,1],sc3_elevation[0,1]]
    test_access = [sc1_access[0,1],sc2_access[0,1],sc3_access[0,1]]

    range_worked = test_ranges == pytest.approx(ref_ranges, accuracy)
    elevation_worked = test_elevation == pytest.approx(ref_elevation, test_elevation)
    access_worked = test_access == pytest.approx(ref_access, abs=1e-16)

    assert (range_worked and elevation_worked and access_worked)


def test_rotation(show_plots):
    '''
    Tests whether groundLocation:
        1. Computes the current location based on the initial position and the rotation rate of the planet
        it is attached to.
    :return:
    '''
    testFailCount = 0
    testMessages = []

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTime = macros.sec2nano(10. * 60.)
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #   Initialize new atmosphere and drag model, add them to task
    groundTarget = groundLocation.GroundLocation()
    groundTarget.ModelTag = "groundTarget"
    groundTarget.planetRadius = orbitalMotion.REQ_EARTH * 1000.
    groundTarget.maximumRange = 100e3 # meters
    groundTarget.minimumElevation = np.radians(89.)
    groundTarget.specifyLocation(np.radians(0.), np.radians(-10), 0.)
    scSim.AddModelToTask(simTaskName, groundTarget)

    #   Write out mock planet rotation, spacecraft position messages
    sc1_message = simMessages.SCPlusStatesSimMsg()
    sc1_message.r_BN_N = np.array([orbitalMotion.REQ_EARTH*1e3 + 90e3, 0, 0])  # SC1 is in range
    sc1_message_name = "sc1_msg"

    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, sc1_message_name, sc1_message)
    groundTarget.addSpacecraftToModel(sc1_message_name)

    planet_message = simMessages.SpicePlanetStateSimMsg()
    planet_message_name = "test_planet"
    planet_message.J20002Pfix = rbk.euler3(np.radians(10.)).tolist()

    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, planet_message_name, planet_message)
    groundTarget.planetInMsgName = planet_message_name

    # Log the access indicator
    numDataPoints = 2
    samplingTime = int(simulationTime / (numDataPoints - 1))
    scSim.TotalSim.logThisMessage(groundTarget.accessOutMsgNames[0], samplingTime)
    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()
    # Get the logged data
    sc1_access = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.hasAccess',range(1))
    sc1_slant = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.slantRange',range(1))
    sc1_elevation =scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.elevation',range(1))

    #   Compare to expected values
    accuracy = 1e-8
    ref_ranges = [90e3]
    ref_elevation = [np.radians(90.)]
    ref_access = [1]

    test_ranges = [sc1_slant[0,1]]
    test_elevation = [sc1_elevation[0,1]]
    test_access = [sc1_access[0,1]]

    range_worked = test_ranges == pytest.approx(ref_ranges, accuracy)
    elevation_worked = test_elevation == pytest.approx(ref_elevation, test_elevation)
    access_worked = test_access == pytest.approx(ref_access, abs=1e-16)

    assert (range_worked and elevation_worked and access_worked)

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
    ax = fig.gca(projection='3d')

    # draw sphere
    u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:20j]
    x = orbitalMotion.REQ_EARTH*1000 * np.cos(u) * np.sin(v)
    y = orbitalMotion.REQ_EARTH*1000 *np.sin(u) * np.sin(v)
    z = orbitalMotion.REQ_EARTH*1000 *np.cos(v)
    ax.plot_wireframe(x, y, z, color="g")

    # draw a point0
    ax.scatter(groundLocation[0],groundLocation[1],groundLocation[2], color="r", s=100)


    # draw a vector
    from matplotlib.patches import FancyArrowPatch
    from mpl_toolkits.mplot3d import proj3d

    for location in scLocations:
        ax.scatter(location[0],location[1],location[2],color='k',s=100)

        ax.quiver(groundLocation[0],groundLocation[1],groundLocation[2],
              location[0],location[1],location[2], length=1.0, normalize=True)
    #ax.add_artist(a)

    plt.show()

if __name__ == '__main__':
    test_rotation()