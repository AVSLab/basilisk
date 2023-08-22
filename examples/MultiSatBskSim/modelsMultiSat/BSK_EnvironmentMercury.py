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

import numpy as np
from Basilisk import __path__
from Basilisk.simulation import ephemerisConverter, groundLocation, eclipse
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import macros as mc, simIncludeGravBody

bskPath = __path__[0]


class BSKEnvironmentModel:
    """Defines the Mercury Environment."""
    def __init__(self, SimBase, envRate):
        # Define empty class variables
        self.mu = None
        self.planetRadius = None
        self.sun = None
        self.mercury= None

        # Define process name, task name and task time-step
        self.envTaskName = "EnvironmentTask"
        processTasksTimeStep = mc.sec2nano(envRate)

        # Create task
        SimBase.envProc.addTask(SimBase.CreateNewTask(self.envTaskName, processTasksTimeStep))

        # Instantiate Env modules as objects
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.ephemObject = ephemerisConverter.EphemerisConverter()
        self.eclipseObject = eclipse.Eclipse()
        self.groundStation = groundLocation.GroundLocation()

        # Initialize all modules and write init one-time messages
        self.InitAllEnvObjects()

        # Add modules to environment task
        SimBase.AddModelToTask(self.envTaskName, self.gravFactory.spiceObject, 200)
        SimBase.AddModelToTask(self.envTaskName, self.ephemObject, 200)
        SimBase.AddModelToTask(self.envTaskName, self.eclipseObject, 200)
        SimBase.AddModelToTask(self.envTaskName, self.groundStation, 200)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetGravityBodies(self):
        """
        Specify what gravitational bodies to include in the simulation.
        """
        # Create gravity bodies
        gravBodies = self.gravFactory.createBodies(['sun', 'mercury'])
        gravBodies['mercury'].isCentralBody = True
        self.mu = self.gravFactory.gravBodies['mercury'].mu
        self.planetRadius = self.gravFactory.gravBodies['mercury'].radEquator
        self.sun = 0
        self.mercury = 1

        # Override information with SPICE
        timeInitString = "2012 MAY 1 00:28:30.0"
        self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                              timeInitString,
                                              epochInMsg=True
                                              )
        self.gravFactory.spiceObject.zeroBase = 'mercury'

        # Add pyswice instances
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants

    def SetEpochObject(self):
        """
        Add the ephemeris object to use with the SPICE library.
        """

        # self.epochMsg = self.gravFactory.epochMsg
        self.ephemObject.ModelTag = 'EphemData'
        self.ephemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        self.ephemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.mercury])

    def SetEclipseObject(self):
        """
        Specify what celestial object is causing an eclipse message.
        """
        self.eclipseObject.ModelTag = "eclipseObject"
        self.eclipseObject.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        # add all celestial objects in spiceObjects except for the sun (0th object)
        for item in range(1, len(self.gravFactory.spiceObject.planetStateOutMsgs)):
            self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[item])

    def SetGroundLocations(self):
        """
        Specify which ground locations are of interest.
        """
        self.groundStation.ModelTag = "GroundStation"
        self.groundStation.planetRadius = self.planetRadius
        self.groundStation.specifyLocation(np.radians(40.009971), np.radians(-105.243895), 1624)
        self.groundStation.minimumElevation = np.radians(10.)
        self.groundStation.maximumRange = 1e9

    # Global call to initialize every module
    def InitAllEnvObjects(self):
        self.SetGravityBodies()
        self.SetEpochObject()
        self.SetEclipseObject()
        self.SetGroundLocations()
