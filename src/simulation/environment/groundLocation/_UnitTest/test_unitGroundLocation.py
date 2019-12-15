
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
from Basilisk.utilities import orbitalMotion
from Basilisk.simulation import simMessages
from Basilisk.simulation import groundLocation

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


#@pytest.mark.parameterize("multiSc",[1,2])
#@pytest.mark.parametrize("elevation", [])
#@pytest.mark.parametrize("scAttitude", [[0,0,0], rbk.C2MRP(rbk.euler3212C([0,np.radians(60.),0])), rbk.C2MRP(rbk.euler3212C([0,np.radians(90.),0]))])
# provide a unique test method name, starting with test_
def test_groundLocation(show_plots):
    """
    Tests the groundLocation() class. In the reference case, a spacecraft is placed directly above the groundLocation. Multiple
    variants are used to test branches in the code:
        1. Support for multiple spacecraft. Adds another spacecraft on the opposite side of the Earth.
        2. Support for lat/long/altitude and ECEF position specification. 1st case is Boulder; the second is nega-Boulder.
        3. Elevation limitations:
        4. Range limitations: In case 1, the range is unlimited. In case 2, the range is set to 1km less than the s/c altitude.
        5. Ground propagation: This tests whether groundLocations are correctly propagated through inertial space using SPICEPlanetSimMsgs.

    These tests are handled parametrically to ensure full coverage.
    """

    showVal = False

    [testResults, testMessage] = test_range()

    assert testResults < 1, testMessage

def test_range():
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
    groundTarget.maximumRange = 90e3 # meters
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

    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, sc1_message_name, sc1_message)
    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, sc2_message_name, sc2_message)

    groundTarget.addSpacecraftToModel(sc1_message_name)
    groundTarget.addSpacecraftToModel(sc2_message_name)

    # Log the access indicator
    numDataPoints = 2
    samplingTime = int(simulationTime / (numDataPoints - 1))
    scSim.TotalSim.logThisMessage(groundTarget.accessOutMsgNames[-1], samplingTime)
    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()
    # Get the logged data
    #sc1_slant = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.slantRange',range(1))
    #sc1_elevation =scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.elevation',range(1))

    sc2_access = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[1] + '.hasAccess',range(1))
    sc2_slant = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[1] + '.slantRange',range(1))
    sc2_elevation = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[1] + '.elevation',range(1))
    #   Compare to expected values
    accuracy = 1e-8
    ref_ranges = [100e3, 101e3]
    ref_elevation = [np.radians(90.),np.radians(90.)]
    ref_access = [1, 0]

    return testFailCount, testMessages


def run(show_plots, satelliteLocation):
    '''Call this routine directly to run the script.'''
    testFailCount = 0
    testMessages = []

    #  Create a new simulationBaseClass, process, and task for the sim
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
    groundTarget.specifyLocation(np.radians(40.),np.radians(105.16),1624.0) #   Put the ground location in Boulder, CO

    scSim.AddModelToTask(simTaskName, groundTarget)

    #   Write out mock planet rotation, spacecraft position messages
    sc1_message = simMessages.SCPlusStatesSimMsg()
    sc1_message.r_BN_N = [-7100e3, 0, 0] #  SC1 is on the opposite side of earth, and should have no access.
    sc1_message_name = "sc1_msg"

    sc2_message = simMessages.SCPlusStatesSimMsg()
    #   SC2 is placed inside/outside the visibility cone for the ground station
    sc2_message.r_BN_N = [7100e3,0,0]
    sc2_message_name = "sc2_msg"

    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, sc1_message_name, sc1_message)
    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, sc2_message_name, sc2_message)

    groundTarget.addSpacecraftToModel(sc1_message_name)
    groundTarget.addSpacecraftToModel(sc2_message_name)


    #Log the access indicator
    numDataPoints = 2
    samplingTime = int(simulationTime / (numDataPoints-1))
    scSim.TotalSim.logThisMessage(groundTarget.accessOutMsgNames[-1], samplingTime)
    #Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()
    #Get the logged data
    simResults = scSim.pullMultiMessageLogData([groundTarget.accessOutMsgNames[-1]+'.hasAccess',
                                                groundTarget.accessOutMsgNames[-1]+'.slantRange',
                                                groundTarget.accessOutMsgNames[-1]+'.elevation'
                                               ], [range(1),range(1),range(1)],1)
    accessData = simResults[groundTarget.accessOutMsgNames[-1]+'.hasAccess']
    slantData = simResults[groundTarget.accessOutMsgNames[-1]+'.slantRange']
    elevationData = simResults[groundTarget.accessOutMsgNames[-1]+'.elevation']

    #   Compare to expected values
    accuracy = 1e-8

    if show_plots:
        #plot_geomertry(groundLocation.r_LP_Init, np.vstack(sc1_message.r_BN_N, sc2_message.r_BN_N), 10.)
        print('WIP - plotting.')
        plot_geometry(groundTarget.r_LP_P_Init, np.vstack([sc1_message.r_BN_N, sc2_message.r_BN_N]), 0.0)
    return [testFailCount, ''.join(testMessages)]

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
    run(True,
        "Visible")          # setEpoch