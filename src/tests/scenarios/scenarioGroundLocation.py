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
from Basilisk.simulation import (groundLocation, spacecraftPlus, planetEphemeris)

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

def run(show_plots):

    #  Create a new simulationBaseClass, process, and task for the sim
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTime = macros.sec2nano(24*60*60) #    Let's run for a full day.
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #   Set up an orbit propagator for Earth.

    earth = planetEphemeris.PlanetEphemeris()
    earth.planetNames = planetEphemeris.StringVector(["earth"])
    earth.rotRate = planetEphemeris.DoubleVector([orbitalMotion.OMEGA_EARTH])

    #   Set up a pair of spacecraft whose orbits might cross Boulder once or twice...

    oe1 = orbitalMotion.ClassicElements()
    oe1.a = 6771.0e3 #m
    oe1.e = 0
    oe1.i = np.radians(51.6)
    oe1.omega = 0
    oe1.Omega = 0
    oe1.f = 0

    oe2 = orbitalMotion.ClassicElements()
    oe2.a = 7300.0e3 #m
    oe2.e = 0.44
    oe2.i = np.radians(63.64)
    oe2.omega = 0
    oe2.Omega = np.radians(40)
    oe2.f = 0

    iss_r, iss_v = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH*1000**3.0, oe1)
    tacSat_r, tacSat_v = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH*1000**3.0, oe2)

    iss = spacecraftPlus.SpacecraftPlus()
    iss.ModelTag = "InternationalSpaceStation"
    iss.hub.r_CN_NInit = iss_r  # m   - r_CN_N
    iss.hub.v_CN_NInit = iss_v  # m/s - v_CN_N
    iss.scStateOutMsgName = "issStateMsg"
    scSim.AddModelToTask(simTaskName,iss)

    tacSat= spacecraftPlus.SpacecraftPlus()
    tacSat.ModelTag = "tacSat"
    tacSat.hub.r_CN_NInit = tacSat_r  # m   - r_CN_N
    tacSat.hub.v_CN_NInit = tacSat_v # m/s - v_CN_N
    tacSat.scStateOutMsgName = "tacSatStateMsg"
    scSim.AddModelToTask(simTaskName,tacSat)

    #   Initialize the ground location in Boulder, CO.
    groundTarget = groundLocation.GroundLocation()
    groundTarget.ModelTag = "groundTarget"
    groundTarget.planetRadius = orbitalMotion.REQ_EARTH * 1000.
    groundTarget.specifyLocation(np.radians(40.),np.radians(105.16),1624.0) #   Put the ground location in Boulder, CO
    groundTarget.planetInMsgName = "earth_planet_data"
    scSim.AddModelToTask(simTaskName, groundTarget)

    groundTarget.addSpacecraftToModel(tacSat.scStateOutMsgName)
    groundTarget.addSpacecraftToModel(iss.scStateOutMsgName)


    #Log the access indicator
    numDataPoints = 2
    samplingTime = int(simulationTime / (numDataPoints-1))
    scSim.TotalSim.logThisMessage(groundTarget.accessOutMsgNames[-1], samplingTime)
    #Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()
    #Get the logged data

    #dynamicsResults = scSim.pullMultiMessageLogData([iss.scStateOutMsgName+'.r_BN_N',
    #                                tacSat.scStateOutMsgName+'.r_BN_N'],
    #                              [range(3),range(3)], ['double','double'])

    #issPosition = dynamicsResults[iss.scStateOutMsgName+'.r_BN_N']
    #tacSatPosition = dynamicsResults[tacSat.scStateOutMsgName+'.r_BN_N']

    accessResults = {}

    for messageName in groundTarget.accessOutMsgNames:
        print(messageName)
        tmpResults = scSim.pullMultiMessageLogData([messageName+'.hasAccess',
                                                messageName+'.slantRange',
                                                messageName+'.elevation'
                                                ], [range(1),range(1),range(1)],['bool','double','double'])
        accessResults.update(tmpResults)
    issAccessData = accessResults[groundTarget.accessOutMsgNames[0]+'.hasAccess']
    issSlantData = accessResults[groundTarget.accessOutMsgNames[0]+'.slantRange']
    issElevationData = accessResults[groundTarget.accessOutMsgNames[0]+'.elevation']
    tacSatAccessData = accessResults[groundTarget.accessOutMsgNames[1]+'.hasAccess']
    tacSatSlantData = accessResults[groundTarget.accessOutMsgNames[1]+'.slantRange']
    tacSatElevationData = accessResults[groundTarget.accessOutMsgNames[1]+'.elevation']

    if show_plots:
        #plot_geomertry(groundLocation.r_LP_Init, np.vstack(sc1_message.r_BN_N, sc2_message.r_BN_N), 10.)
        print('WIP - plotting.')
        plot_geometry(groundTarget.r_LP_P_Init, issPosition, issAccessData, tacSatPosition, tacSatAccessData)

def plot_geometry(groundLocation, sc1Pos, sc1Acc, sc2Pos, sc2Acc):
    """
    Plots the location of a ground station, its field of view,  and the positions of two spacecraft to verify whether
    the spacecraft have access to the ground station.
    :param groundLocation: [3,] : an ECI ground position.
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

    for idx in range(0,len(sc1Acc)):

        if sc1Acc[idx,1] == 1:
            ax.scatter(sc1Pos[idx,0],sc1Pos[idx,1],sc1Pos[idx,2],'g*')
        else:
            ax.scatter(sc1Pos[idx,0],sc1Pos[idx,1],sc1Pos[idx,2],'r*')

        if sc2Acc[idx,1] == 1:
            ax.scatter(sc2Pos[idx,0],sc2Pos[idx,1],sc2Pos[idx,2],'g*')
        else:
            ax.scatter(sc2Pos[idx,0],sc2Pos[idx,1],sc2Pos[idx,2],'r*')
    plt.show()

if __name__ == '__main__':
    run(True)          # setEpoch