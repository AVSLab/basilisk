''' '''
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


@pytest.mark.parametrize("satelliteLocation", ["vis","marginal","non-vis"])#  Satellite locations correspond to visible, marginal, and non-visible cases

# provide a unique test method name, starting with test_
def test_groundLocation(show_plots, satelliteLocation):
    '''Tests the groundLocation() class. Specifically examines:
        1. Support for multiple spacecraft. Tested by making the groundLocation subscribe to multiple messages, and evaluating its output.
        2. Support for lat/long/altitude and ECEF position specification. Tested using pytest.mark.parameterize to use different input methods.
        3. Access computation: One spacecraft is placed on the other side of the earth. Depending on the flag, another spacecraft will be placed marginally inside or outside the
           cone specified by the groundLocation base class to verify that access is correctly computed.'''
    # each test method requires a single assert method to be called
    showVal = False

    [testResults, testMessage] = run(showVal, satelliteLocation)

    assert testResults < 1, testMessage

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
    accessData = scSim.pullMessageLogData(groundLocation.envOutMsgNames[-1]+'.hasAccess')
    slantData = scSim.pullMessageLogData(groundLocation.envOutMsgNames[-1]+'.slantRange')
    elevationData = scSim.pullMessageLogData(groundLocation.envOutMsgNames[-1]+'.elvation')

    #   Compare to expected values
    accuracy = 1e-8

    if show_plots:
        plot_geomertry(groundLocation.r_LP_Init, np.vstack(sc1_message.r_BN_N, sc2_message.r_BN_N), 10.)

    return [testFailCount, ''.join(testMessages)]

def plot_geomertry(groundLocation, scLocations, minimumElevation):
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
    u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
    x = np.cos(u) * np.sin(v)
    y = np.sin(u) * np.sin(v)
    z = np.cos(v)
    ax.plot_wireframe(x, y, z, color="g")

    # draw a point
    ax.scatter(groundLocation[0],groundLocation[1],groundLocation[2], color="r", s=100)

    # draw a vector
    from matplotlib.patches import FancyArrowPatch
    from mpl_toolkits.mplot3d import proj3d

    class Arrow3D(FancyArrowPatch):

        def __init__(self, xs, ys, zs, *args, **kwargs):
            FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
            self._verts3d = xs, ys, zs

        def draw(self, renderer):
            xs3d, ys3d, zs3d = self._verts3d
            xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
            self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
            FancyArrowPatch.draw(self, renderer)

    a = Arrow3D([0, 1], [0, 1], [0, 1], mutation_scale=20,
                lw=1, arrowstyle="-|>", color="k")
    ax.add_artist(a)
    plt.show()

if __name__ == '__main__':
    run(True,
        "Visible")          # setEpoch