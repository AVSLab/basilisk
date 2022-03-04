#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
#   Unit Test Script
#   Module Name:        tabularAtmosphere
#   Author:             Mikaela Felix
#   Creation Date:      Feb 11, 2022
#

import pytest
import os, inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import tabularAtmosphere              # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.architecture import bskLogging
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities.readAtmTable import readAtmTable


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.  Note that the order in that you add the parametrize method
# matters for the documentation in that it impacts the order in which the test arguments are shown.
# The first parametrize arguments are shown last in the pytest argument list
@pytest.mark.parametrize("accuracy", [1e-12])
@pytest.mark.parametrize("altitude", [50.0, 33.33333, 10000.0, -10.0]) # exact, interpolate, above, below
@pytest.mark.parametrize("useMinReach", [ True, False])
@pytest.mark.parametrize("useMaxReach", [ True, False])

def test_tabularAtmosphere(show_plots, altitude, accuracy):
    r"""
    **Validation Test Description**

    Compose a general description of what is being tested in this unit test script.  Add enough information so the
    reader understands the purpose and limitations of the test.  As this test script is not parameterized, only one
    version of this script will run.  Note that the ``pytest`` HTML report will list each parameterized test case
    individually.  This way it is clear what set of parameters passed.  But, this also means that this doc-string
    content will be copied into each report so each test description is individually complete.  If there is a
    discussion you want to include that is specific to the a parameterized test case, then include this at the
    end of the file with a conditional print() statement that only executes for that particular parameterized test.
    
 ADDED   
   Calculates the atmospheric density and temperature at the requested altitude of the user. The function readAtmTable
    will be used to to read in the requested atmosphere table and then fetch the column of values for altitude, density, 
    and temperature to insert them into their own respective variables as vectors of doubles. The function also makes 
    necessary conversions to these values before assigning them to their variables so the units are meters for altitude, 
    kilogram per meter cubed for density, and Kelvin for temperature. Will parse the altitude list until either there is
    a match with the requested value and will return the density and temperaure with the same index. If the requested 
    altitude lands between two consecutive values on the list, then will perform linear interpolation. 
    When requesting the table, the inputted string must match directly with the table name but can be all lowercase. 
    The input altitude must be in units of meters. Any input altitude outside of the range on the requested table will
    return 0. Any input value other than type double will throw an error. 

    **Test Parameters**

    As this is a parameterized unit test, note that the test case parameters values are shown automatically in the
    pytest HTML report.  This sample script has the parameters param1 and param 2.  Provide a description of what
    each parameter controls.  This is a convenient location to include the accuracy variable used in the
    validation test.

    Args:
        param1 (int): Dummy test parameter for this parameterized unit test
        param2 (int): Dummy test parameter for this parameterized unit test
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    Here discuss what parameters are being checked.  For example, in this file we are checking the values of the
    variables

    - ``dummy``
    - ``dataVector[3]``

    **Figure Discussion**

    If the test script produces figures you might include a brief discussion on what the simulation results show.
    Discuss why these results validate the operation of the BSK module.

    **General Documentation Comments**

    If the script generates figures, these figures will be automatically pulled from ``matplotlib`` and included below.
    Make sure that the figures have appropriate axes labels and a figure title if needed.  The figures content
    should be understood by just looking at the figure.

    At the end of the script where a print statement says that the script passes.

    Don't use any of the AutoTeX methods we used to use as the goal is to have all the validation reporting
    contained within this HTML ``pytest`` report.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = tabularAtmosphereTestFunction(show_plots, altitude, accuracy)
    assert testResults < 1, testMessage


def tabularAtmosphereTestFunction(show_plots, altitude, accuracy, useMinReach, useMaxReach):
    testFailCount = 0                       # zero unit test result counter
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    module = tabularAtmosphere.TabularAtmosphere()   # update with current values
    module.ModelTag = "tabularAtmosphere"            # update python name of test module
    
    # define constants & load data
    r_eq = 6371*1000.0
    filename = '../../../../../supportData/AtmosphereData/EarthGRAMNominal.txt' # TODO: fix file path syntax
    altList, rhoList, tempList = readAtmTable(filename,'EarthGRAM')
        
    # assign constants & ref. data to module
    module.planetRadius = r_eq
    module.altList = tabularAtmosphere.DoubleVector(altList)    
    module.rhoList = tabularAtmosphere.DoubleVector(rhoList)
    module.tempList = tabularAtmosphere.DoubleVector(tempList)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)
    
    
    # CHECK - env min and max
    minReach = -1.0
    if useMinReach:
        minReach = module.altList[0]    
        module.envMinReach = minReach
        module.planetRadius =  6378136.6 #meters
    maxReach = -1.0
    if useMaxReach:
        maxReach = module.altList[-1]
        module.envMaxReach = maxReach
        module.planetRadius =  6378136.6
    
    #   setup orbit and simulation time
    r0 = r_eq + (altitude * 1000.0)  # meters
    oe = orbitalMotion.ClassicElements()
    mu = 0.3986004415E+15  # meters^3/s^2
    oe.a = r0
    oe.e = 0.0
    oe.i = 45.0 * macros.D2R
    oe.Omega = 30.0 * macros.D2R
    oe.omega = 120.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    r0N, v0N = orbitalMotion.elem2rv(mu, oe)

    # create the input messages
    scStateMsg = messaging.SCStatesMsgPayload()  # Create a structure for the input message
    scStateMsg.r_BN_N = np.array(r0N)
    scInMsg = messaging.SCStatesMsg().write(scStateMsg)
    
    # add spacecraft to environment model
    module.addSpacecraftToModel(scInMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.envOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()
    
    # This pulls the actual data log from the simulation run.
    densData = dataLog.neutralDensity
    tempData = dataLog.localTemp
    
    # define python function to compute truth values
    def tabAtmoComp(val, xList, yList):
        if val < xList[0]:
            out = xList[0]
            return out
        elif val > xList[-1]:
            out = 0
            return out
        else:
            for i, x in enumerate(xList):
                if x > val:
                    x0 = xList[i-1]
                    y0 = yList[i-1]
                    y1 = yList[i]
                    m = (y1 - y0)/(x - x0)
                    out = y0 + (val - x0) * m
                    return out
    
    # compute truth values
    trueDensity = tabAtmoComp(altitude * 1000, altList, rhoList)
    print('\nmodule density: {0:.6e}'.format(densData[0]))
    print('true density: {0:.6e}\n'.format(trueDensity))
    
    trueTemp = tabAtmoComp(altitude * 1000, altList, tempList)
    print('\nmodule temperature: {0:.6e}'.format(tempData[0]))
    print('true temperature: {0:.6e}\n'.format(trueTemp))
    
    # compare truth values to module results
    unitTestSupport.writeTeXSnippet("unitTestToleranceValue", str(accuracy), path)
    if trueDensity != 0:
        testFailCount = not unitTestSupport.isDoubleEqualRelative(densData[0], trueDensity, accuracy)
    else:
        testFailCount = not unitTestSupport.isDoubleEqual(densData[0], trueDensity, accuracy)
    if testFailCount == 0:
        testMessage = "density computed correctly"
    else:
        testMessage = "density computed incorrectly"
        
    # compare truth values to module results for temperature
    if trueTemp != 0 :    # needs checking
        testFailCount = not unitTestSupport.isDoubleEqualRelative(tempData[0], trueTemp, accuracy)
    else:
        testFailCount = not unitTestSupport.isDoubleEqual(tempData[0], trueTemp, accuracy)
    if testFailCount == 0:
        testMessage += "and temperature computed correctly"
    else:
        testMessage += "and temperature computed incorrectly"

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)       

    return [testFailCount, testMessage]         


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_tabularAtmosphere(              # update "module" in function name
                 False,
                 51,          # altitude
                 1e-12        # accuracy
               )
