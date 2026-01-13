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
from Basilisk.simulation import ephemerisConverter, groundLocation, eclipse, magneticFieldWMM, exponentialAtmosphere #msisAtmosphere
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import macros as mc, simIncludeGravBody, unitTestSupport

bskPath = __path__[0]


class BSKEnvironmentModel:
    """Defines the Earth Environment."""
    def __init__(self, SimBase, envRate):
        # Define empty class variables
        self.mu = None
        self.planetRadius = None

        # Define process name, task name and task time-step
        self.envTaskName = "EnvironmentTask"
        processTasksTimeStep = mc.sec2nano(envRate)
        epochTimeStr = "2025 OCTOBER 08 15:00:00.000 (UTC)"

        # Create task
        SimBase.envProc.addTask(SimBase.CreateNewTask(self.envTaskName, processTasksTimeStep))

        # Instantiate Env modules as objects
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.ephemObject = ephemerisConverter.EphemerisConverter()
        self.eclipseObject = eclipse.Eclipse()
        self.groundStationBar = groundLocation.GroundLocation()
        self.groundStationSval = groundLocation.GroundLocation()
        self.magneticField = magneticFieldWMM.MagneticFieldWMM()
        self.atmosphere = exponentialAtmosphere.ExponentialAtmosphere()

        # Initialize all modules and write init one-time messages
        self.InitAllEnvObjects(epochTimeStr)

        # Add modules to environment task
        SimBase.AddModelToTask(self.envTaskName, self.gravFactory.spiceObject, 200)
        SimBase.AddModelToTask(self.envTaskName, self.ephemObject, 200)
        SimBase.AddModelToTask(self.envTaskName, self.eclipseObject, 200)
        SimBase.AddModelToTask(self.envTaskName, self.groundStationBar, 200)
        SimBase.AddModelToTask(self.envTaskName, self.groundStationSval, 200)
        SimBase.AddModelToTask(self.envTaskName, self.magneticField, 199)
        SimBase.AddModelToTask(self.envTaskName, self.atmosphere, 199)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetGravityBodies(self, epochTimeStr):
        """
        Specify what gravitational bodies to include in the simulation.
        """
        # Create gravity bodies
        self.gravBodyList = ['sun', 'earth', 'moon'] # Sun must be the first element (sun is excluded in: SetEclipseObject()))
        gravBodies = self.gravFactory.createBodies(self.gravBodyList)
        gravBodies['earth'].isCentralBody = True
        self.mu = self.gravFactory.gravBodies['earth'].mu
        self.planetRadius = self.gravFactory.gravBodies['earth'].radEquator

        # Override information with SPICE
        self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                              epochTimeStr,
                                              epochInMsg=True)
        self.gravFactory.spiceObject.zeroBase = 'Earth'

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
        self.ephemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.gravBodyList.index('sun')])
        self.ephemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.gravBodyList.index('earth')])
        self.ephemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.gravBodyList.index('moon')])

    def SetEclipseObject(self):
        """
        Specify what celestial object is causing an eclipse message.
        """
        self.eclipseObject.ModelTag = "eclipseObject"
        self.eclipseObject.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.gravBodyList.index('sun')])
        # add all celestial objects in spiceObjects except for the sun (0th object)
        for item in range(1, len(self.gravFactory.spiceObject.planetStateOutMsgs)):
            self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[item])

    def SetGroundLocations(self):
        """
        Specify which ground locations are of interest.
        """
        self.groundStationBar.ModelTag = "BarentsSea"  # TODO change this to mapping function from Anaïs C. to scann the Barents Sea
        self.groundStationBar.planetRadius = self.planetRadius
        self.groundStationBar.specifyLocation(np.radians(73.0), np.radians(20.0), 0.0)  # Gloeshaugen, Trondheim, Norway
        self.groundStationBar.minimumElevation = np.radians(15.)
        self.groundStationBar.maximumRange = 1e9

        self.groundStationSval.ModelTag = "KsatSvalbard"
        self.groundStationSval.planetRadius = self.planetRadius
        self.groundStationSval.specifyLocation(np.radians(78.929056), np.radians(11.870703), 32)  # Svalbard, Norway
        self.groundStationSval.minimumElevation = np.radians(10.)
        self.groundStationSval.maximumRange = 1e9

    def SetMagneticFieldObject(self, epochTimeStr):
        """
        Specify the magnetic field model to use.
        """
        self.magneticField.ModelTag = "WMM"
        self.magneticField.wmmDataFullPath = bskPath + '/supportData/MagneticField/WMM.COF'
        # set epoch date/time message
        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg(epochTimeStr)
        self.magneticField.epochInMsg.subscribeTo(epochMsg)
        # set the minReach and maxReach values for the WMM
#        self.magneticField.minReach = -1 # TODO confirm infinit reach? (feasable??)
#        self.magneticField.maxReach = -1 # TODO confirm infinit reach? (feasable??)

    def SetAtmosphereObject(self):
        """
        Specify the atmosphere model to use.
        """
        self.atmosphere.ModelTag = "ExpAtmo"

    def InitAllEnvObjects(self, epochTimeStr):
        self.SetGravityBodies(epochTimeStr)
        self.SetEpochObject()
        self.SetEclipseObject()
        self.SetGroundLocations()
        self.SetMagneticFieldObject(epochTimeStr)
        self.SetAtmosphereObject()
