''' '''
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
#
# Ephemeris Converter Unit Test
#
# Purpose:  Test the proper function of the ephemeris_converter module.
# Author:   Thibaud Teil
#

from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import spice_interface
from Basilisk.simulation import ephemeris_converter
from Basilisk.utilities import macros
from Basilisk import __path__
bskPath = __path__[0]


# provide a unique test method name, starting with test_
def test_ephemConvert(show_plots):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitephemeris_converter(show_plots)
    assert testResults < 1, testMessage


def unitephemeris_converter(show_plots):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()
    sim.TotalSim.terminateSimulation()

    simulationTime = macros.sec2nano(30.)
    numDataPoints = 600
    samplingTime = simulationTime / (numDataPoints-1)
    DynUnitTestProc = sim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(sim.CreateNewTask(unitTaskName, samplingTime))

    # List of planets tested
    planets = ['sun', 'earth', 'mars barycenter']

    # Initialize the spice module
    spiceObject = spice_interface.SpiceInterface()
    spiceObject.ModelTag = "SpiceInterfaceData"
    spiceObject.SPICEDataPath = bskPath + '/supportData/EphemerisData/'
    spiceObject.outputBufferCount = 10000
    spiceObject.planetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun"])
    spiceObject.UTCCalInit = "2015 February 10, 00:00:00.0 TDB"
    sim.AddModelToTask(unitTaskName, spiceObject)

    # Initialize the ephermis module
    ephemObject = ephemeris_converter.EphemerisConverter()
    ephemObject.ModelTag = 'EphemData'
    messageMap = {}
    for planet in planets:
        messageMap[planet + '_planet_data'] = planet + '_ephemeris_data'
    ephemObject.messageNameMap = ephemeris_converter.map_string_string(messageMap)
    sim.AddModelToTask(unitTaskName, ephemObject)

    # Configure simulation
    sim.ConfigureStopTime(int(simulationTime))
    sim.AddVariableForLogging('EphemData.messagesLinked')
    sim.AddVariableForLogging('EphemData.numOutputBuffers')
    for planet in planets:
        sim.TotalSim.logThisMessage(planet + '_planet_data', 5*samplingTime)
        sim.TotalSim.logThisMessage(planet + '_ephemeris_data', 5*samplingTime)

    # Execute simulation
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

    # Get the link confirmation
    linkMessagesCheck = sim.GetLogVariableData('EphemData.messagesLinked')
    for i in range(len(linkMessagesCheck[:,0])):
        if linkMessagesCheck[i,1] - 1.0 > 1E-12:
            testFailCount += 1
            testMessages.append("FAILED: Messages not linked succesfully")

    # Get the position, velocities and time for the message before and after the copy
    accuracy = 1e-12
    for planet in planets:
        ephemPlanetPosData = sim.pullMessageLogData(planet + '_ephemeris_data' + '.r_BdyZero_N', range(3))
        spicePlanetPosData = sim.pullMessageLogData(planet + '_planet_data' + '.PositionVector', range(3))
        ephemPlanetVelData = sim.pullMessageLogData(planet + '_ephemeris_data' + '.v_BdyZero_N', range(3))
        spicePlanetVelData = sim.pullMessageLogData(planet + '_planet_data' + '.VelocityVector', range(3))
        testFailCount, testMessages = unitTestSupport.compareArrayRelative(spicePlanetPosData[:,1:4], ephemPlanetPosData, accuracy, "Position", testFailCount, testMessages)
        testFailCount, testMessages = unitTestSupport.compareArrayRelative(spicePlanetVelData[:,1:4], ephemPlanetVelData, accuracy, "Velocity", testFailCount, testMessages)

    # print out success message if no error were found
    if testFailCount == 0:
        print   " \n PASSED "

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_ephemConvert(False)
