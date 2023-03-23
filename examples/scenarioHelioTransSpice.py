#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

The purpose of this simulation is to illustrate how to set a spacecraft's heliocentric translational motion using
custom Spice files. This allows the user to easily visualize a mission trajectory using Vizard.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioHelioTransSpice.py

Configuring Translational Motion Using Custom Spice Files
---------------------------------------------------------

To set up the spacecraft's heliocentric translational motion via custom Spice files, the user should first create a
string list containing the desired file names to upload. This script loads a single Spice file::

    customSpiceFiles = ["spacecraft_21T01.bsp"]

Next, the ``loadSpiceKernel()`` method of class SpiceInterface should be called to load the custom Spice files.
This method accepts a file name and the path to the desired file to load::

    gravFactory.spiceObject.loadSpiceKernel(file, os.path.join(path, "dataForExamples", "Spice/"))

Note that setting up the orbital elements and initial conditions using the ``orbitalMotion`` module is no longer needed.

After the Spice files are loaded, the final step is to connect the configured Spice translational output message to
the spacecraft object's ``transRefInMsg`` input message::

    scObject.transRefInMsg.subscribeTo(gravFactory.spiceObject.transRefStateOutMsgs[0])

Finally, add the Spice object to the simulation task list::

    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)

Ensure to unload the Spice kernels at the end of each simulation::

    gravFactory.spiceObject.unloadSpiceKernel(file, os.path.join(path, "dataForExamples", "Spice/"))

Simulation Visualization In Vizard
----------------------------------

The following image illustrates the expected visualization of this simulation script.

.. image:: /_images/static/scenarioHelioTransSpice.jpg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose: This simulation shows how to specify a spacecraft's heliocentric translational motion through loading custom Spice files.
# Author:   Leah Kiner
# Creation Date: Feb. 4 2022
#

import inspect
import os
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import vizSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

def run():
    """
        Heliocentric mission simulation scenarion.
    """
    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Configure the simulation
    scSim = SimulationBaseClass.SimBaseClass()

    # Shows the simulation progress bar in the terminal
    scSim.SetProgressBar(True)

    # Create the dynamics process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(5 * 60 * 60.)

    # Add the dynamics task to the dynamics process
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Configure the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spiceSat"  # Name of the spacecraft

    # Create gravitational bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createBodies(["earth", "sun", "venus", "mars barycenter"])
    scObject.gravField.setGravBodies(gravityEffector.GravBodyVector(list(gravFactory.gravBodies.values())))

    # Create and configure the default SPICE support module.
    timeInitString = "2028 February 24 5:30:30.0"  # Store the date and time of the start of the simulation.
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)

    # Create a string list of all custom Spice files to upload
    customSpiceFiles = ["spacecraft_21T01.bsp"]

    # Load the custom Spice files using the SpiceInterface class loadSpiceKernel() method
    for file in customSpiceFiles:
        gravFactory.spiceObject.loadSpiceKernel(file, os.path.join(path, "dataForExamples", "Spice/"))

    # Add spacecraft name
    scNames = ["-60000"]
    gravFactory.spiceObject.addSpacecraftNames(messaging.StringVector(scNames))

    # Connect the configured Spice translational output message to spacecraft object's transRefInMsg input message
    scObject.transRefInMsg.subscribeTo(gravFactory.spiceObject.transRefStateOutMsgs[0])

    # Add the Spice and spacecraft objects to the simulation task list.
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)
    scSim.AddModelToTask(simTaskName, scObject)

    # define the spacecraft inertia and other parameters
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.000], [-0.00], [0.00]]  # rad/s - omega_BN_B

    # Configure Vizard settings
    if vizSupport.vizFound:
        colorMsgContent = messaging.ColorMsgPayload()
        colorMsgContent.colorRGBA = vizSupport.toRGBA255("Yellow")
        colorMsg = messaging.ColorMsg().write(colorMsgContent)

        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                  , oscOrbitColorList=[vizSupport.toRGBA255("Magenta")]
                                                  , trueOrbitColorInMsgList=colorMsg.addSubscriber()
                                                  # , saveFile=__file__
                                                  )
        viz.epochInMsg.subscribeTo(gravFactory.epochMsg)
        viz.settings.orbitLinesOn = 1
        viz.settings.spacecraftHelioViewSizeMultiplier = 3
        viz.settings.showSpacecraftLabels = 1
        viz.settings.showCelestialBodyLabels = 1
        viz.settings.mainCameraTarget = "sun"  # Gives heliocentric view
        viz.settings.showMissionTime = 1

    # Initialize and execute simulation
    scSim.InitializeSimulation()
    day = 60 * 60 * 24  # sec
    simulationTime = macros.sec2nano(0.5 * 365 * day)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # change true orbit line color
    colorMsgContent.colorRGBA = vizSupport.toRGBA255("Cyan")
    colorMsg.write(colorMsgContent)
    simulationTime = macros.sec2nano(4.5 * 365 * day)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Unload custom Spice kernels at the end of each simulation
    gravFactory.unloadSpiceKernels()
    for file in customSpiceFiles:
        gravFactory.spiceObject.unloadSpiceKernel(file, os.path.join(path, "Data", "Spice/"))

    return


if __name__ == "__main__":
    run()