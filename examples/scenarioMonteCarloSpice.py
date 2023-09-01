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

This script illustrates how to run a Monte Carlo simulation where the Spice is used within the Python
setup.  Note that the Python Spice setup is separate from the BSK c++ Spice module setup.  In this tutorial
a very simple simulation is shown to showcase how to correctly perform Python-based Spice function calls with a
Basilisk Monte Carlo run.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioMonteCarloSpice.py

The simulation sets up a simple spacecraft and associated initial conditions.  Note that the Basilisk spacecraft
simulation is setup within the class ``MySimulation``.  Here the the code is added to load Spice kernels within
Python to pull the Hubble states from Spice.  Thus, this python Spice call is performed within each Monte Carlo
thread.  In this simple example the Hubble states are then printed to the terminal.

As this Monte Carlo scenario is setup to run 12 times, by running this script the user should see
no errors and the Hubble states printed out 12 times.

In the Controller class `MyController` there is Spice kernel loading code that is commented out.
If the kernels are loaded within the controller class then this results in a Spice kernel loading error.

The user should be careful to load the Spice or use within the Python code within the simulation class.



"""

#
# Basilisk Integrated Test
#
# Purpose:  This Monte Carlo example shows how to properly use Spice in such simulations.
#


import inspect
import os
import shutil

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))
# @endcond

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities.pyswice_spk_utilities import spkRead

from Basilisk.simulation import spacecraft

from Basilisk.utilities.MonteCarlo.Controller import Controller


class MyController(Controller):
    def __init__(self):  # Constructor for Monte Carlo simulations
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


        self.dynProcess = self.CreateNewProcess(simProcessName)

        self.dynProcess.addTask(self.CreateNewTask(simTaskName, macros.sec2nano(10.)))

        scObject = spacecraft.Spacecraft()
        self.AddModelToTask(simTaskName, scObject, 1)
        scObject.hub.r_CN_NInit = [7000000.0, 0.0, 0.0]     # m   - r_CN_N
        scObject.hub.v_CN_NInit = [0.0, 7500.0, 0.0]        # m/s - v_CN_N


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
        startCalendarTime = '2012 APR 29 15:18:14.907 (UTC)'
        zeroBase = 'Sun'
        integFrame = 'j2000'
        stateOut = spkRead(self.scSpiceName, startCalendarTime, integFrame, zeroBase)
        print(stateOut)

def run():
    """
    This is the main function that is called in this script.  It illustrates possible ways
    to include the Python Spice library in a simulation that uses Monte Carlo runs.
    """

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

    # Here is another example where it is allowable to run the python spice routines within a MC simulation setup
    #
    # dataPath = bskPath + "/supportData/EphemerisData/"
    # pyswice.furnsh_c(dataPath + 'naif0011.tls')
    # pyswice.furnsh_c(dataPath + 'pck00010.tpc')
    # pyswice.furnsh_c(dataPath + 'de-403-masses.tpc')
    # pyswice.furnsh_c(dataPath + 'de430.bsp')
    #
    # startCalendarTime = '2012 AUG 05, 21:35:07.496 (UTC)'
    # startTimeArray = sim_model.new_doubleArray(1)
    # pyswice.str2et_c(startCalendarTime, startTimeArray)
    # sim_model.delete_doubleArray(startTimeArray)

    # After the monteCarlo run is configured, it is executed.
    failures = monteCarlo.executeSimulations()

    # Now we clean up data from this test
    shutil.rmtree(dirName)

    return


def executeScenario(sim):
    sim.ConfigureStopTime(macros.sec2nano(100.))
    sim.InitializeSimulation()

    # Here is another example where it is allowable to run the python spice routines within a MC simulation setup
    #
    # dataPath = bskPath + "/supportData/EphemerisData/"
    # pyswice.furnsh_c(dataPath + 'naif0011.tls')
    # pyswice.furnsh_c(dataPath + 'pck00010.tpc')
    # pyswice.furnsh_c(dataPath + 'de-403-masses.tpc')
    # pyswice.furnsh_c(dataPath + 'de430.bsp')

    sim.ExecuteSimulation()


if __name__ == "__main__":
    run()
