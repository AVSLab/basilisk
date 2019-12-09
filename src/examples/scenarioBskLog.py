#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        fswModuleTemplateParametrized
#   Author:             (First Name) (Last Name)
#   Creation Date:      Month Day, Year
#

import os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)







# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.fswAlgorithms.fswModuleTemplate import fswModuleTemplate
from Basilisk.utilities import macros
from Basilisk.simulation import bskLogging

def run(case):

    if case == 1:
        # here the verbosity is set globally to WARNING or higher.
        bskLogging.setDefaultLogLevel(bskLogging.WARNING)

    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = fswModuleTemplate.fswModuleTemplateConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "fswModuleTemplate"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.dataInMsgName = "sampleInput"          # update with current values
    moduleConfig.dataOutMsgName = "sampleOutput"        # update with current values
    moduleConfig.dummy = 1                              # update module parameter with required values

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = fswModuleTemplate.FswModuleTemplateFswMsg()  # Create a structure for the input message
    inputMessageData.outputVector = [1.0, 1.0, 0.7]       # Set up a list as a 3-vector
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.dataInMsgName,
                               inputMessageData)

    # setup the bskLog verbosity
    if case == 0:
        # default case = 0 does not set the bskLog verbosity within Python, but uses the default verbosity
        bskLogging.printDefaultLogLevel()
        level = bskLogging.getDefaultLogLevel()
    elif case == 1:
        # here the verbosity was set globally to WARNING or higher at the beginning of the script
        bskLogging.printDefaultLogLevel()
        level = bskLogging.getDefaultLogLevel()

    elif case == 2:
        # here the bskLog verbosity is only changed for this module
        logger = bskLogging.BSKLogger()
        logger.setLogLevel(bskLogging.ERROR)
        print("The verbosity is only changed for this module.")
        logger.printLogLevel()
        moduleConfig.bskLogger = logger
        level = logger.getLogLevel()

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    return level


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
         2,       # case 1 uses global default, case 2 use module specific default, case 0 uses compiler default
       )
