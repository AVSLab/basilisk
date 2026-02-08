#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/oV2lPwB1J2g?si=Sw0z6B1D6RXydbTw" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

This scenario demonstrates how to set up and modify QuadMaps within Vizard, as both rectangular/polar
defined regions and camera field-of-view regions.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioQuadMaps.py

.. important:: This scenario requires BSK to be built with ``vizInterface`` enabled.

Creating QuadMaps
-----------------
As shown in this scenario, QuadMaps can be created using the helper functions in :ref:`quadMapSupport`.
This is already imported within :ref:`vizSupport`, and can be accessed using ``vizSupport.qms``.
QuadMaps can also be built by hand.

.. note:: The ordering of the ``vertices`` field is important for QuadMaps to be rendered correctly.
          Each corner must be specified in the fixed-frame of its parent body, and the resulting vector
          is [x1, y1, z1, x2, ..., y4, z4]. If another quadrilateral is to be added to the mesh, its
          vertices append directly after the first set. Thus, the ``vertices`` field should always contain
          a multiple of 12 elements (k quads, each with 4 corners, each with a 3D position).

This scenario demonstrates setting up QuadMaps using latitude/longitude/altitude, as shown by both Colorado (bounded by
[37°N, 41°N] latitude and [-109°02'48"E, -102°02'48"E] longitude) and the Arctic Region (bounded by [66.5°N, 90°N]
latitude and [-180°E, 180°E] longitude). These are computed using ``vizSupport.qms.computeRectMesh()``.

The satellite is capturing images of Earth's surface every 20 minutes, and the camera field-of-view is projected onto
the surface as a red QuadMap. Due to the eccentricity of the orbit, the spacing and scale of these FOV boxes change
over time. There are two separate functions which handle the camera FOV: ``vizSupport.qms.computeCamFOVBox()`` returns
the intersection points between the reference ellipsoid and the camera FOV, while ``vizSupport.qms.subdivideFOVBox()``
interpolates these corners according to produce a square grid that can more easily wrap a convex surface. It is
important to note that, in this scenario, the FOV box is only drawn if all 4 corners find a valid intersection with the
reference ellipsoid. Otherwise, the non-intersection case becomes more complex to handle and is currently the
responsibility of the user to handle.

A custom QuadMap is also created on one of the solar cells of the satellite, which is defined by hand in body-fixed coordinates.

.. image:: /_images/static/scenarioQuadMaps_overview.png
   :align: center

Once half of the simulation time passes, various settings are changed: Arctic Region toggles its ``isHidden`` flag,
Solar Cell switches its label to "NOLABEL", and Colorado changes its label/color.

"""


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the Vizard QuadMap surface highlight visual element.
# Author:   Jack Fox
# Creation Date:  Apr. 15, 2025
#

import os
import numpy as np

# To play with any scenario scripts as tutorials, you should make a copy of them into a custom folder
# outside the Basilisk directory.
#
# To copy them, first find the location of the Basilisk installation.
# After installing, you can find the installed location of Basilisk by opening a python interpreter and
# running the commands:
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])
fileNamePath = os.path.abspath(__file__)

# import simulation related support
from Basilisk.simulation import spacecraft

# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport)
from Basilisk.architecture import messaging


def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots.  This script has no plots to show.

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Set up the simulation tasks/objects
    # Initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"

    # Add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # Set up Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    planet.radiusRatio = 0.9966

    # Finally, the gravitational body must be connected to the spacecraft object.  This is done with
    gravFactory.addBodiesTo(scObject)

    # Setup spice library for Earth ephemeris
    timeInitString = "2000 November 26, 09:30:00.0 TDB"
    spiceObject = gravFactory.createSpiceInterface(time=timeInitString, epochInMsg=True)
    spiceObject.zeroBase = 'Earth'

    scSim.AddModelToTask(simTaskName, spiceObject)

    #
    #   Set up orbit and simulation time
    #
    # Set up the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rMEO = 11260. * 1000      # meters

    # Elliptic MEO case
    oe.a = rMEO
    oe.e = 0.25
    oe.i = 28. * macros.D2R
    oe.Omega = 10.5 * macros.D2R
    oe.omega = 20.5 * macros.D2R
    oe.f = 10. * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    simulationTime = macros.hour2nano(6)

    # Set up data logging before the simulation is initialized
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    # create a logging task object of the spacecraft output message at the desired down sampling ratio
    dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataRec)

    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                                                  # saveFile=__file__
                                                  )

        # Del viz.quadMaps[:] at the start of the sim
        viz.quadMaps.clear()

        # Create Standard Camera
        cam = vizSupport.createStandardCamera(viz,
                                              setMode=0,
                                              bodyTarget="earth",
                                              fieldOfView=np.deg2rad(10)
                                              )

        # In more complex scenarios with pointing modules, a CameraConfigMsg can be used
        # for more precise camera-pointing. See scenarioVizPoint.py for this camera setup demo.

        # cam = messaging.CameraConfigMsgPayload()
        # cam.cameraID = 1
        # cam.sigma_CB = [-0.333333, 0.333333, -0.333333]
        # cam.cameraPos_B = [5., 0., 0.]
        # cam.fieldOfView = 75.*macros.D2R
        # cam.resolution = [2048, 2048]
        # cam.skybox = ""
        # viz.addCamMsgToModule(messaging.CameraConfigMsg().write(cam))

        # Create initial QuadMaps
        vizSupport.addQuadMap(viz,
                              ID=1,
                              parentBodyName="earth",
                              vertices=vizSupport.qms.computeRectMesh(planet,
                                                                      [66.5, 90],
                                                                      [-180, 180],
                                                                      8),
                              color=[0, 0, 255, 100],
                              label="Arctic Region"
                              )

        vizSupport.addQuadMap(viz,
                              ID=2,
                              parentBodyName="earth",
                              vertices=vizSupport.qms.computeRectMesh(planet,
                                                                      [37, 41],
                                                                      [-102.0467, -109.0467],
                                                                      5),
                              color=[0, 255, 0, 100],
                              label="Colorado"
                              )

        vizSupport.addQuadMap(viz,
                              ID=3,
                              parentBodyName="bsk-Sat",
                              vertices=[-2.551, 0.341, 1.01,
                                        -2.804, 0.341, 1.01,
                                        -2.804, 0.111, 1.01,
                                        -2.551, 0.111, 1.01],
                              color=[255, 128, 0, 100],
                              label="Solar Cell")

        viz.settings.mainCameraTarget = "earth"

        # Number of interpolations for camera FOV when generating QuadMaps (helps wrap convex surface)
        fieldOfViewSubdivs = 3

    # ==================== Run simulation loop ==================== #
    #   initialize Simulation:  This function runs the self_init()
    #   and reset() routines on each module.
    scSim.InitializeSimulation()

    camQM_ID = 10

    incrementalStopTime = 0
    imgTimeStep = macros.min2nano(20)
    while incrementalStopTime < simulationTime:
        if vizSupport.vizFound:
            # Add QuadMap showing camera FOV
            FOVBox = vizSupport.qms.computeCamFOVBox(planet, spiceObject, scObject, cam)
            # Only draw if all 4 corners intersect Earth!
            if len(FOVBox) == 12:
                # Subdivide region to wrap onto ellipsoid
                if fieldOfViewSubdivs > 1:
                    FOVBox = vizSupport.qms.subdivideFOVBox(planet, FOVBox, fieldOfViewSubdivs)
                vizSupport.addQuadMap(viz,
                                      ID=camQM_ID,
                                      parentBodyName="earth",
                                      vertices=FOVBox,
                                      color=[255, 0, 0, 60]
                                      )
                camQM_ID += 1

            if incrementalStopTime == simulationTime/2:
                vizSupport.addQuadMap(viz,
                                      ID=1,
                                      parentBodyName="earth",
                                      vertices=vizSupport.qms.computeRectMesh(planet,
                                                                              [66.5, 90],
                                                                              [-180, 180],
                                                                              8),
                                      color=[0, 0, 255, 100],
                                      isHidden=True
                                      )
                vizSupport.addQuadMap(viz,
                                      ID=2,
                                      parentBodyName="earth",
                                      vertices=vizSupport.qms.computeRectMesh(planet,
                                                                              [37, 41],
                                                                              [-102.0467, -109.0467],
                                                                              5),
                                      color=[255, 255, 0, 100],
                                      label="CO (new color!)"
                                      )
                vizSupport.addQuadMap(viz,
                                      ID=3,
                                      parentBodyName="bsk-Sat",
                                      vertices=[-2.551, 0.341, 1.01,
                                                -2.804, 0.341, 1.01,
                                                -2.804, 0.111, 1.01,
                                                -2.551, 0.111, 1.01],
                                      color=[255, 128, 0, 100],
                                      label="NOLABEL")

        incrementalStopTime += imgTimeStep
        scSim.ConfigureStopTime(incrementalStopTime)
        scSim.ExecuteSimulation()

        # Empty out QuadMap container so they are only sent once
        vizSupport.quadMapList = []

    # Unload Spice kernel
    gravFactory.unloadSpiceKernels()

    return {} # no figures to return


if __name__ == "__main__":
    run(False)
