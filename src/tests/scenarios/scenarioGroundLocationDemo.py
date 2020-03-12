'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
## \defgroup scenarioSimplePowerDemo
## @{
## Illustration of how to use simplePower modules to perform orbit power analysis considering attitude and orbital coupling.
#
# Ground Interaction System Demonstration {#scenarioGroundLocationDemo}
# ====
#
# Scenario Description
# -----
# This scenario demonstrates the use of a Basilisk groundLocation instance to determine the visibility of a spacecraft
# from a point on the surface of the Earth. In the future, it will be extended to include the access of instruments on-board
# the spacecraft to locations on the ground.
#
# For example, a ground location representing a viewer in Boulder, CO would be instantiated as:
# ~~~~~~~~~~~~~{.py}
#    # Create a ground location representing Boulder, CO
#     boulder = groundLocation.GroundLocation()
#     boulder.ModelTag = "boulder"
#     boulder.planetRadius = orbitalMotion.REQ_EARTH * 1000.
#     boulder.specifyLocation(np.radians(40.0149856),np.radians(-105.2705456 ), 1624.0)
#     boulder.addSpacecraftToModel(scObject.scStateOutMsgName)
#     boulder.planetInMsgName = planet.bodyInMsgName
#     boulder.maximumRange = 1e9 # m
#     boulder.minimumElevation = np.radians(10.)
#     scenarioSim.AddModelToTask(taskName, boulder)
# ~~~~~~~~~~~~~
#
# The outputs of the simplePowerSystem can be logged by calling:
# ~~~~~~~~~~~~~{.py}
# # Log the subsystem output messages at each sim timestep
#     scenarioSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)
#
# ...Sim Execution...
#
# # Pull the logged message attributes that we want
#locationData = scenarioSim.pullMultiMessageLogData([boulder.accessOutMsgNames[0]+'.hasAccess',
#                                                      boulder.accessOutMsgNames[0] + '.slantRange',
#                                                      boulder.accessOutMsgNames[0] + '.elevation',
#                                                        boulder.accessOutMsgNames[0] + '.azimuth'],
#                                                      [range(1),range(1),range(1),range(1)], ['int','double','double','double'])
#accessHist = locationData[boulder.accessOutMsgNames[0] + '.hasAccess']
#rangeHist = locationData[boulder.accessOutMsgNames[0] + '.slantRange']
#elevationHist = locationData[boulder.accessOutMsgNames[0] + '.elevation']
# ~~~~~~~~~~~~~
# To run the scenario , call the python script through
#
#       python3 scenarioGroundLocationDemo.py
#
# When the simulation completes, a subfigure diagram showing the ISS' visibility, relative slant range, and elevation will
# be shown for the next two days.
## @}
import os, inspect
import numpy as np
from matplotlib import pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import groundLocation
from Basilisk.simulation import eclipse
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import astroFunctions
from Basilisk import __path__
bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))

def run():
    taskName = "unitTask"
    processname = "TestProcess"

    scenarioSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(10.0)
    testProc = scenarioSim.CreateNewProcess(processname)
    testProc.addTask(scenarioSim.CreateNewTask(taskName, testProcessRate))

    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    gravFactory = simIncludeGravBody.gravBodyFactory()

    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

    #   setup orbit using orbitalMotion library
    oe = orbitalMotion.ClassicElements()
    oe.a = 6798920.68
    oe.e = .0013218
    oe.i = 51.51020*macros.D2R
    oe.Omega = 201.17770*macros.D2R
    oe.omega = 31.27778*macros.D2R
    oe.f     = 304.40301*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    n = np.sqrt(mu/(oe.a**3.0))
    P = 2.*np.pi/n

    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)

    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.00], [-0.00], [0.00]]
    scenarioSim.AddModelToTask(taskName, scObject)


    #   Create an eclipse object so the panels don't always work
    eclipseObject = eclipse.Eclipse()
    eclipseObject.addPositionMsgName(scObject.scStateOutMsgName)
    eclipseObject.addPlanetName('earth')

    scenarioSim.AddModelToTask(taskName, eclipseObject)


    # setup Spice interface for some solar system bodies
    timeInitString = '2019 DEC 11 11:24:50.319 (UTC)'
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/'
                                     , timeInitString
                                     , spicePlanetNames = ["sun", "earth"]
                                     )
    scenarioSim.AddModelToTask(taskName, gravFactory.spiceObject, None, -1)

    # Create a ground location representing Boulder, CO
    boulder = groundLocation.GroundLocation()
    boulder.ModelTag = "boulder"
    boulder.planetRadius = orbitalMotion.REQ_EARTH * 1000.
    boulder.specifyLocation(np.radians(40.0149856),np.radians(-105.2705456 ), 1624.0)
    boulder.addSpacecraftToModel(scObject.scStateOutMsgName)
    boulder.planetInMsgName = planet.bodyInMsgName
    boulder.maximumRange = 1e9 # m
    boulder.minimumElevation = np.radians(10.)
    scenarioSim.AddModelToTask(taskName, boulder)

    # Setup logging on the ground location
    scenarioSim.TotalSim.logThisMessage(boulder.accessOutMsgNames[0], testProcessRate)

    # Also log attitude/orbit parameters
    scenarioSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)
    scenarioSim.TotalSim.logThisMessage(planet.bodyInMsgName, testProcessRate)

    n_days = 0.5
    n_seconds = n_days * 24. * 60. * 60.

    scenarioSim.InitializeSimulation()
    scenarioSim.ConfigureStopTime(macros.sec2nano(n_seconds))
    scenarioSim.ExecuteSimulation()

    locationData = scenarioSim.pullMultiMessageLogData([boulder.accessOutMsgNames[0]+'.hasAccess',
                                                      boulder.accessOutMsgNames[0] + '.slantRange',
                                                      boulder.accessOutMsgNames[0] + '.elevation',
                                                        boulder.accessOutMsgNames[0] + '.azimuth'],
                                                      [range(1),range(1),range(1),range(1)], ['int','double','double','double'])

    accessHist = locationData[boulder.accessOutMsgNames[0]+'.hasAccess']
    rangeHist = locationData[boulder.accessOutMsgNames[0]+'.slantRange']
    elevationHist = locationData[boulder.accessOutMsgNames[0]+'.elevation']

    scOrbit = scenarioSim.pullMessageLogData(scObject.scStateOutMsgName + ".r_BN_N", list(range(3)))
    planetOrbit = scenarioSim.pullMessageLogData(planet.bodyInMsgName+".PositionVector", list(range(3)))

    tvec = accessHist[:,0]
    tvec = tvec * macros.NANO2HOUR

    #   Plot the power states
    figureList = {}
    plt.close("all")  # clears out plots from earlier test runs
    f, (ax1, ax2, ax3) = plt.subplots(3,1,sharex=True)
    ax1.plot(tvec, accessHist[:,1])
    ax1.set_ylabel("Access Indicator")

    ax2.plot(tvec,rangeHist[:,1]/1000.)
    ax2.set_ylabel("Range (km)")
    ax3.plot(tvec,np.degrees(elevationHist[:,1]))
    ax3.set_xlabel('Time (Hr)')
    ax3.set_ylabel('Elevation (deg)')

    pltName = "scenarioGroundLocation"

    plt.plot(projection='3d')

    figureList[pltName] = plt.figure(1)

    return figureList

#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    fig = run()
    plt.show()
