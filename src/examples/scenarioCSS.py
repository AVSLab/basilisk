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

This script sets up a 6-DOF spacecraft in deep space without any gravitational bodies.
Only rotational  motion is simulated.  The script illustrates how to setup CSS
sensor units and log their data.  It is possible  to setup individual CSS sensors,
or setup a constellation or array of CSS sensors.

The script is found in the folder ``src/examples`` and executed by using::

    python3 scenarioCSS.py

When the simulation completes a plot is shown for the CSS sensor signal history.

The simulation layout options (A) and (B) are shown in the following illustration.  A single simulation process is created which contains both the spacecraft simulation module, as well as two individual CSS sensor units.  In scenario (A) the CSS units are individually executed by the simulation, while scenario (B) uses a CSS constellation class that executes a list of CSS evaluations at the same time.

.. image:: /_images/static/test_scenarioCSS.svg
   :align: center

The dynamics simulation is setup using a :ref:`SpacecraftPlus` module where a specific
spacecraft location is specified.  Note that both the rotational and translational
degrees of freedom of the spacecraft hub are turned on here to get a 6-DOF simulation.
The position  vector is required when computing the relative heading between the sun
and the spacecraft locations.  The  spacecraft position is held fixed, while the
orientation rotates constantly about the 3rd body axis.

The Field-Of-View variable fov must be specified.  This is the angle between the
sensor bore-sight and the edge of the field of view.  Beyond this angle all sensor
signals are set to zero. The scaleFactor variable scales a normalized CSS response
to this value if facing the sun head on.  The input message name InputSunMsg specifies
an input message that contains the sun's position. If sensor corruptions are to be
modeled, this can be set through the variables::

   CSS1.KellyFactor
   CSS1.SenBias
   CSS1.SenNoiseStd

The Kelly factor has values between 0 (off) and 1 and distorts the nominal cosine
response.  The SenBias  variable determines a normalized bias to be applied to the
CSS model, and SenNoiseStd provides Gaussian noise.

To create additional CSS sensor units, copies of the first CSS unit can be made.
This means only the parameters different in the additional units must be set.

A key parameter that remains is the CSS sensor unit normal vector.  There are
several options to set this vector (in body frame components).  The first
method is to set :math:`\hat{\mathbf n}` or ``nHat_B`` directly.  This is
done with::

   CSS1.nHat_B = np.array([1.0, 0.0, 0.0])
   CSS2.nHat_B = np.array([0.0, -1.0, 0.0])

Another option is to use a frame associated relative to a common CSS platform
:math:`\cal P`.  The bundled CSS units are often symmetrically arranged on a
platform such as in a pyramid configuration.  The the platform frame is  specified through::

   CSS1.setBodyToPlatformDCM(90.*macros.D2R, 0., 0.)

where the three orientation angles are 3-2-1 Euler angles.  These platform angles
are initialized to zero.  Next, the CSS unit direction vectors can be specified
through the azimuth and elevation angles (:math:`\phi`, :math:`\theta`).  These are (3)-(-2) Euler angles. ::

   CSS1.phi = 90.*macros.D2R
   CSS1.theta = 0.*macros.D2R

If no platform orientation is specified, then naturally these azimuth and elevation angles are
measured relative to the body frame :math:`\cal B`.

An optional input message is the solar eclipse message ``sunEclipseInMsgName``.
If this message input name is specified for a CSS unit, then the eclipse
information is taken into account.  If this message name is not set, then
the CSS defaults to the spacecraft always being in the sun.

Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a range of script configurations.

::

    show_plots = True, useCSSConstellation=False, usePlatform=False, useEclipse=False, useKelly=False

This scenario simulates the CSS units being setup individually without any corruption.
The sensor unit normal axes are directly set, and no eclipse is modeled.
The signals of the two CSS units range from a maximum of 2 if the CSS axis is pointing
at the sun to zero.  The limited field of view of 80 degrees causes the sensor signal
to be clipped when the sun light incidence angle gets too small.

.. image:: /_images/Scenarios/scenarioCSS0000.svg
   :align: center

::

   show_plots = True, useCSSConstellation=False, usePlatform=True, useEclipse=False, useKelly=False

The resulting CSS sensor signals should be identical to the first scenario as the
chosen platform orientation and CSS azimuth and elevation angles are chosen to
yield the same senor normal unit axes.

.. image:: /_images/Scenarios/scenarioCSS0100.svg
   :align: center

::

   show_plots = True, useCSSConstellation=False, usePlatform=False, useEclipse=True, useKelly=False

The resulting CSS signals are scaled by a factor of 0.5 and are shown below.

.. image:: /_images/Scenarios/scenarioCSS0010.svg
  :align: center

::

    show_plots = True, useCSSConstellation=False, usePlatform=False, useEclipse=False, useKelly=True

This causes the CSS signals to become slightly warped, and depart from the nominal
cosine  behavior.

.. image:: /_images/Scenarios/scenarioCSS0001.svg
   :align: center

::

    show_plots = True, useCSSConstellation=True, usePlatform=False, useEclipse=False, useKelly=False

The resulting simulation results are shown below to be identical to the first setup as expected.

.. image:: /_images/Scenarios/scenarioCSS1000.svg
   :align: center

"""



#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Demonstrates how to setup CSS sensors on a rigid spacecraft
# Author:   Hanspeter Schaub
# Creation Date:  July 21, 2017
#

import sys
import os
import numpy as np

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.simulation import coarse_sun_sensor
from Basilisk.utilities import orbitalMotion as om

# import simulation related support
from Basilisk.simulation import spacecraftPlus

# import message declarations
from Basilisk.simulation import simMessages

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]


def run(show_plots, useCSSConstellation, usePlatform, useEclipse, useKelly):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        useCSSConstellation (bool): Flag indicating if the CSS cluster/confiugration class should be used.
        usePlatform (bool): Flag specifying if the CSS platform orientation should be set.
        useEclipse (bool): Flag indicating if the eclipse input message is used.
        useKelly (bool): Flag specifying if the Kelly corruption factor is used.

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.sec2nano(120.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0                     # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    #
    # set initial spacecraft states
    #
    scObject.hub.r_CN_NInit = [[-om.AU*1000.0], [0.0], [0.0]]              # m   - r_CN_N
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]                 # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]               # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [1.*macros.D2R]]   # rad/s - omega_BN_B

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # In both CSS simulation scenarios (A) and (B) the CSS modules must
    # first be individuall created and configured.
    # In this simulation each case uses two CSS sensors.  The minimum
    # variables that must be set for each CSS includes
    CSS1 = coarse_sun_sensor.CoarseSunSensor()
    CSS1.ModelTag = "CSS1_sensor"
    CSS1.fov = 80.*macros.D2R
    CSS1.scaleFactor = 2.0
    CSS1.cssDataOutMsgName = "CSS1_output"
    CSS1.sunInMsgName = "sun_message"
    if useKelly:
        CSS1.kellyFactor = 0.2
    if useEclipse:
        CSS1.sunEclipseInMsgName = "eclipse_message"
    if usePlatform:
        CSS1.setBodyToPlatformDCM(90.*macros.D2R, 0., 0.)
        CSS1.theta = -90.*macros.D2R
        CSS1.phi = 0*macros.D2R
        CSS1.setUnitDirectionVectorWithPerturbation(0., 0.)
    else:
        CSS1.nHat_B = np.array([1.0, 0.0, 0.0])

    CSS2 = coarse_sun_sensor.CoarseSunSensor(CSS1)      # make copy of first CSS unit
    CSS2.ModelTag = "CSS2_sensor"
    CSS2.cssDataOutMsgName = "CSS2_output"
    if usePlatform:
        CSS2.theta = 180.*macros.D2R
        CSS2.setUnitDirectionVectorWithPerturbation(0., 0.)
    else:
        CSS2.nHat_B = np.array([0.0, -1.0, 0.0])

    if useCSSConstellation:
        # If instead of individual CSS a cluster of CSS units is to be evaluated as one,
        # then they can be grouped into a list, and added to the Basilisk execution
        # stack as a single entity.  This is done with
        cssList = [CSS1, CSS2]
        cssArray = coarse_sun_sensor.CSSConstellation()
        cssArray.ModelTag = "css_array"
        cssArray.sensorList = coarse_sun_sensor.CSSVector(cssList)
        cssArray.outputConstellationMessage = "CSS_Array_output"
        scSim.AddModelToTask(simTaskName, cssArray)
        # Here the CSSConstellation() module will call the individual CSS
        # update functions, collect all the sensor
        # signals, and store the output in a single output message
        # containing an array of CSS sensor signals.
    else:
        # In this scenario (A) setup the CSS unit are each evaluated separately through
        # This means that each CSS unit creates a individual output messages.
        scSim.AddModelToTask(simTaskName, CSS1)
        scSim.AddModelToTask(simTaskName, CSS2)

    #
    #   Setup data logging before the simulation is initialized
    #
    if useCSSConstellation:
        scSim.TotalSim.logThisMessage(cssArray.outputConstellationMessage, simulationTimeStep)
    else:
        scSim.TotalSim.logThisMessage(CSS1.cssDataOutMsgName, simulationTimeStep)
        scSim.TotalSim.logThisMessage(CSS2.cssDataOutMsgName, simulationTimeStep)

    #
    # create simulation messages
    #
    sunPositionMsg = simMessages.SpicePlanetStateSimMsg()
    sunPositionMsg.PositionVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               CSS1.sunInMsgName,
                               sunPositionMsg)

    if useEclipse:
        eclipseMsg = simMessages.EclipseSimMsg()
        eclipseMsg.shadowFactor = 0.5
        unitTestSupport.setMessage(scSim.TotalSim,
                                   simProcessName,
                                   CSS1.sunEclipseInMsgName,
                                   eclipseMsg)
    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataCSSArray = []
    dataCSS1 = []
    dataCSS2 = []
    if useCSSConstellation:
        dataCSSArray = scSim.pullMessageLogData(cssArray.outputConstellationMessage+".CosValue", list(range(len(cssList))))
    else:
        dataCSS1 = scSim.pullMessageLogData(CSS1.cssDataOutMsgName+".OutputData", list(range(1)))
        dataCSS2 = scSim.pullMessageLogData(CSS2.cssDataOutMsgName+".OutputData", list(range(1)))
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileNameString = os.path.basename(os.path.splitext(__file__)[0])
    plt.close("all")        # clears out plots from earlier test runs
    plt.figure(1)
    if useCSSConstellation:
        for idx in range(1, len(cssList)+1):
            plt.plot(dataCSSArray[:, 0]*macros.NANO2SEC, dataCSSArray[:, idx],
                         color=unitTestSupport.getLineColor(idx,3),
                         label='CSS$_{'+str(idx)+'}$')
    else:
        plt.plot(dataCSS1[:, 0]*macros.NANO2SEC, dataCSS1[:, 1],
                 color=unitTestSupport.getLineColor(1,3),
                 label='CSS$_{1}$')
        plt.plot(dataCSS2[:, 0] * macros.NANO2SEC, dataCSS2[:, 1],
                 color=unitTestSupport.getLineColor(2, 3),
                 label='CSS$_{2}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('CSS Signals ')
    figureList = {}
    pltName = fileNameString+str(int(useCSSConstellation))+str(int(usePlatform))+str(int(useEclipse))+str(int(useKelly))
    figureList[pltName] = plt.figure(1)


    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return dataCSSArray, dataCSS1, dataCSS2, simulationTime, figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
         True,        # show_plots
         False,       # useCSSConstellation
         False,       # usePlatform
         False,       # useEclipse
         False        # useKelly
       )
