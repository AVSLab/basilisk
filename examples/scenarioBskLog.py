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

r"""
Overview
--------

This scenario demonstrates how to set up run a Basilisk simulation and change the default verbosity
of ``bskLog`` methods.  The default verbosity is set to DEBUG, which means all ``bskLog`` call print
the associated information.  More information about using ``bskLog`` and changing its verbosity
can be found in :ref:`bskLogging`.

"""

#
#   Unit Test Script
#   Module Name:        cModuleTemplateParametrized
#   Author:             (First Name) (Last Name)
#   Creation Date:      Month Day, Year
#

import inspect
import os

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.utilities import macros
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging

def run(case):
    """
        At the end of the python script you can specify the following example parameters.

        Args:
            case (int):

                ======  ========================================
                 Int    Definition
                ======  ========================================
                  0     Uses the BSK default verbosity of DEBUG
                  1     Sets the verbosity globally to WARNING
                  2     Sets the verbosity the a module to ERROR
                ======  ========================================

        """
    if case == 1:
        # here the verbosity is set globally to WARNING or higher.
        # This call must be made at the beginning of the script, certainly before
        # SimulationBaseClass.SimBaseClass() is called.
        bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = messaging.CModuleTemplateMsgPayload()  # Create a structure for the input message
    inputMessageData.dataVector = [1.0, 1.0, 0.7]  # Set up a list as a 3-vector
    dataMsg = messaging.CModuleTemplateMsg().write(inputMessageData)

    # Construct algorithm and associated C++ container
    module = cModuleTemplate.cModuleTemplate()
    module.ModelTag = "cModuleTemplate"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    module.dataInMsg.subscribeTo(dataMsg)
    module.dummy = 1

    # setup the bskLog verbosity
    if case == 0:
        # default case = 0 does not set the bskLog verbosity within Python, but uses the default verbosity
        bskLogging.printDefaultLogLevel()
        level = bskLogging.getDefaultLogLevel()
    elif case == 1:
        # here the verbosity was set globally to WARNING or higher at the beginning of the script
        bskLogging.printDefaultLogLevel()
        print("The verbosity was globally changed.")
        level = bskLogging.getDefaultLogLevel()

    elif case == 2:
        # here the bskLog verbosity is only changed for this module by setting a custom bskLog instance
        logger = bskLogging.BSKLogger()
        logger.setLogLevel(bskLogging.BSK_ERROR)
        print("The verbosity is only changed for this module.")
        logger.printLogLevel()
        module.bskLogger = logger
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
