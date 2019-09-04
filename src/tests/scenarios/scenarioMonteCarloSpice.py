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
# Basilisk Pyswice Module Usage Example
#
# Purpose:  Example of a common incorrect usage of the pyswice module.
#


import inspect
import os

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))
# @endcond

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk import pyswice

from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import sim_model

from Basilisk.utilities.MonteCarlo.Controller import Controller


class MyController(Controller):
    def __init__(self):  # Constructor for EMM Monte Carlo simulations
        Controller.__init__(self)

        # Uncomment the following block to cause this MC scenario to fail
        # due to an incorrect usage of the pyswice module

        # dataPath = bskPath + "/supportData/EphemerisData/"
        # pyswice.furnsh_c(dataPath + 'naif0011.tls')
        # pyswice.furnsh_c(dataPath + 'pck00010.tpc')
        # pyswice.furnsh_c(dataPath + 'de-403-masses.tpc')
        # pyswice.furnsh_c(dataPath + 'de430.bsp')
        # pyswice.furnsh_c(dataPath + 'hst_edited.bsp')


class MySimulation(SimulationBaseClass.SimBaseClass):
    def __init__(self):
        SimulationBaseClass.SimBaseClass.__init__(self)
        # Create simulation variable names
        simTaskName = "simTask"
        simProcessName = "simProcess"

        self.TotalSim.terminateSimulation()

        self.dynProcess = self.CreateNewProcess(simProcessName)

        self.dynProcess.addTask(self.CreateNewTask(simTaskName, macros.sec2nano(.1)))

        scObject = spacecraftPlus.SpacecraftPlus()
        self.AddModelToTask(simTaskName, scObject, None, 1)

        # operate on pyswice
        dataPath = bskPath + "/supportData/EphemerisData/"
        self.scSpiceName = 'HUBBLE SPACE TELESCOPE'
        pyswice.furnsh_c(dataPath + 'naif0011.tls')
        pyswice.furnsh_c(dataPath + 'pck00010.tpc')
        pyswice.furnsh_c(dataPath + 'de-403-masses.tpc')
        pyswice.furnsh_c(dataPath + 'de430.bsp')
        pyswice.furnsh_c(dataPath + 'hst_edited.bsp')

        self.accessSpiceKernel()

        # This is a hack because of a bug in Basilisk... leave this line it keeps
        # variables from going out of scope after this function returns
        self.additionalReferences = [scObject]

    def accessSpiceKernel(self):
        startCalendarTime = '2012 APR 29 15:18:14.907'
        zeroBase = 'Sun'
        integFrame = 'j2000'
        stateOut = pyswice.spkRead(self.scSpiceName, startCalendarTime, integFrame, zeroBase)
        print(stateOut)


def run():
    # First, the `Controller` class is used in order to define the simulation
    monteCarlo = MyController()
    monteCarlo.setSimulationFunction(MySimulation)
    monteCarlo.setExecutionFunction(executeScenario)
    monteCarlo.setExecutionCount(12)
    monteCarlo.setShouldDisperseSeeds(True)
    monteCarlo.setThreadCount(6)
    monteCarlo.setVerbose(False)

    dirName = "montecarlo_test" + str(os.getpid())
    monteCarlo.setArchiveDir(dirName)

    dataPath = bskPath + "/supportData/EphemerisData/"
    pyswice.furnsh_c(dataPath + 'naif0011.tls')
    pyswice.furnsh_c(dataPath + 'pck00010.tpc')
    pyswice.furnsh_c(dataPath + 'de-403-masses.tpc')
    pyswice.furnsh_c(dataPath + 'de430.bsp')

    startCalendarTime = '2012 AUG 05, 21:35:07.496 (UTC)'
    startTimeArray = sim_model.new_doubleArray(1)
    pyswice.str2et_c(startCalendarTime, startTimeArray)
    sim_model.delete_doubleArray(startTimeArray)

    # After the monteCarlo run is configured, it is executed.
    failures = monteCarlo.executeSimulations()

def executeScenario(sim):
    sim.ConfigureStopTime(10.0)
    sim.InitializeSimulation()

    dataPath = bskPath + "/supportData/EphemerisData/"
    pyswice.furnsh_c(dataPath + 'naif0011.tls')
    pyswice.furnsh_c(dataPath + 'pck00010.tpc')
    pyswice.furnsh_c(dataPath + 'de-403-masses.tpc')
    pyswice.furnsh_c(dataPath + 'de430.bsp')

    sim.ExecuteSimulation()


if __name__ == "__main__":
    run()
