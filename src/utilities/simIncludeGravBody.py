
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.



from Basilisk.simulation import gravityEffector
from Basilisk.simulation import spiceInterface
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation.gravityEffector import loadGravFromFile as loadGravFromFile_python
from Basilisk.architecture import messaging

try:
    from collections.abc import OrderedDict
except ImportError:
    from collections import OrderedDict


class gravBodyFactory(object):
    """Factory cass to create gravitational bodies."""
    def __init__(self, bodyNames=None):
        self.spicePlanetNames = []
        self.spicePlanetFrames = []
        self.gravBodies = OrderedDict()
        self.spiceObject = None
        self.spiceKernelFileNames = []
        self.epochMsg = None
        if bodyNames:
            self.createBodies(bodyNames)

    def createBodies(self, bodyNames):
        """
            A convenience function to create multiple typical solar system bodies.

            Parameters
            ----------
            bodyNames : array_like
                Planet name strings. Each planet name must be a valid SPICE celestial body string.

            Returns
            -------
            gravBodies : array_like
                A list of gravity body objects held by the gravity factory.
        """
        for name in bodyNames:
            if name == 'mercury':
                self.createMercury()
            elif name == 'venus':
                self.createVenus()
            elif name == "earth":
                self.createEarth()
            elif name == "moon":
                self.createMoon()
            elif name == "mars":
                self.createMars()
            elif name == "mars barycenter":
                self.createMarsBarycenter()
            elif name == "jupiter barycenter":
                self.createJupiter()
            elif name == "saturn":
                self.createSaturn()
            elif name == 'uranus':
                self.createUranus()
            elif name == 'neptune':
                self.createNeptune()
            elif name == "sun":
                self.createSun()
            else:
                print("gravBody " + name + " not found in gravBodyUtilities.py")
        return self.gravBodies


    def createSun(self):
        sun = gravityEffector.GravBodyData()
        sun.planetName = "sun_planet_data"
        sun.mu = 1.32712440018E20  # meters^3/s^2
        sun.radEquator = 695508000.0  # meters
        sun.isCentralBody = False
        sun.useSphericalHarmParams = False
        self.gravBodies['sun'] = sun
        sun.this.disown()
        return sun

    def createMercury(self):
        mercury = gravityEffector.GravBodyData()
        mercury.planetName = "mercury_planet_data"
        mercury.mu = 4.28283100e13  # meters^3/s^2
        mercury.radEquator = 2439700.0  # meters
        mercury.isCentralBody = False
        mercury.useSphericalHarmParams = False
        self.gravBodies['mercury'] = mercury
        mercury.this.disown()
        return mercury

    def createVenus(self):
        venus = gravityEffector.GravBodyData()
        venus.planetName = "venus_planet_data"
        venus.mu = 3.24858599e14  # meters^3/s^2
        venus.radEquator = 6051800.0  # meters
        venus.isCentralBody = False
        venus.useSphericalHarmParams = False
        self.gravBodies['venus'] = venus
        venus.this.disown()
        return venus

    def createEarth(self):
        earth = gravityEffector.GravBodyData()
        earth.planetName = "earth_planet_data"
        earth.mu = 0.3986004415E+15  # meters^3/s^2
        earth.radEquator = 6378136.6  # meters
        earth.isCentralBody = False
        earth.useSphericalHarmParams = False
        self.gravBodies['earth'] = earth
        earth.this.disown()
        return earth

    def createMoon(self):
        moon = gravityEffector.GravBodyData()
        moon.planetName = "moon_planet_data"
        moon.mu = 4.902799E12  # meters^3/s^2
        moon.radEquator = 1738100.0  # meters
        moon.isCentralBody = False
        moon.useSphericalHarmParams = False
        self.gravBodies['moon'] = moon
        moon.this.disown()
        return moon

    def createMars(self):
        mars = gravityEffector.GravBodyData()
        mars.planetName = "mars_planet_data"
        mars.mu = 4.28283100e13  # meters^3/s^2
        mars.radEquator = 3396190  # meters
        mars.isCentralBody = False
        mars.useSphericalHarmParams = False
        self.gravBodies['mars'] = mars
        mars.this.disown()
        return mars

    def createMarsBarycenter(self):
        mars_barycenter = gravityEffector.GravBodyData()
        mars_barycenter.planetName = "mars barycenter_planet_data"
        mars_barycenter.mu = 4.28283100e13  # meters^3/s^2
        mars_barycenter.radEquator = 3396190  # meters
        mars_barycenter.isCentralBody = False
        mars_barycenter.useSphericalHarmParams = False
        self.gravBodies['mars barycenter'] = mars_barycenter
        mars_barycenter.this.disown()
        return mars_barycenter

    def createJupiter(self):
        jupiter = gravityEffector.GravBodyData()
        jupiter.planetName = "jupiter barycenter_planet_data"
        jupiter.mu = 1.266865349093058E17  # meters^3/s^2
        jupiter.radEquator = 71492000.0  # meters
        jupiter.isCentralBody = False
        jupiter.useSphericalHarmParams = False
        self.gravBodies['jupiter barycenter'] = jupiter
        jupiter.this.disown()
        return jupiter

    def createSaturn(self):
        saturn = gravityEffector.GravBodyData()
        saturn.planetName = "saturn barycenter_planet_data"
        saturn.mu = 3.79395000E16  # meters^3/s^2
        saturn.radEquator = 60268000.0  # meters
        saturn.isCentralBody = False
        saturn.useSphericalHarmParams = False
        self.gravBodies['saturn'] = saturn
        saturn.this.disown()
        return saturn

    def createUranus(self):
        uranus = gravityEffector.GravBodyData()
        uranus.planetName = "uranus barycenter_planet_data"
        uranus.mu = 5.79396566E15  # meters^3/s^2
        uranus.radEquator = 25559000.0  # meters
        uranus.isCentralBody = False
        uranus.useSphericalHarmParams = False
        self.gravBodies['uranus'] = uranus
        uranus.this.disown()
        return uranus

    def createNeptune(self):
        neptune = gravityEffector.GravBodyData()
        neptune.planetName = "neptune barycenter_planet_data"
        neptune.mu = 6.83509920E15  # meters^3/s^2
        neptune.radEquator = 24764000.0  # meters
        neptune.isCentralBody = False
        neptune.useSphericalHarmParams = False
        self.gravBodies['neptune'] = neptune
        neptune.this.disown()
        return neptune

    def createSpiceInterface(self, path, time, **kwargs):
        """
            A convenience function to configure a NAIF Spice module for the simulation.
            It connect the gravBodyData objects to the spice planet state messages.  Thus,
            it must be run after the gravBodyData objects are created.

            Parameters
            ----------
            path : string
                The absolute path to the Basilisk source directory (default '').
            time : string
                The time string.

            Other Parameters
            ----------------
            kwargs :
                spiceKernalFileNames : array_like
                    A list of spice kernel file names including file extension.
                spicePlanetNames : array_like
                    A list of planet names whose Spice data is loaded, overriding the gravBodies list.
                spicePlanetFrames : array_like
                    A list of strings for the planet frame names.  If left empty for a planet, then
                    "IAU_" + planetName is assumed for the planet frame.
                epochInMsg: bool
                    Flag to set an epoch input message for the spice interface

            Returns
            -------
            spiceObject : Basilisk spice module
                A configured Basilisk spice module.
        """

        if 'spiceKernalFileNames' in kwargs:
            try:
                for fileName in kwargs['spiceKernalFileNames']:
                    self.spiceKernelFileNames.append(fileName)
            except TypeError:
                raise TypeError('spiceKernalFileNames expects a list')
        else:
            self.spiceKernelFileNames.extend(['de430.bsp', 'naif0012.tls', 'de-403-masses.tpc', 'pck00010.tpc'])

        self.spicePlanetNames = []
        if 'spicePlanetNames' in kwargs:
            try:
                for planetName in kwargs['spicePlanetNames']:
                    self.spicePlanetNames.append(planetName)
            except TypeError:
                raise TypeError('spicePlanetNames expects a list')
        else:
            self.spicePlanetNames = list(self.gravBodies.keys())

        self.spicePlanetFrames = []
        if 'spicePlanetFrames' in kwargs:
            try:
                for planetFrame in kwargs['spicePlanetFrames']:
                    self.spicePlanetFrames.append(planetFrame)
            except TypeError:
                raise TypeError('spicePlanetFrames expects a list')

        self.spiceObject = spiceInterface.SpiceInterface()
        self.spiceObject.ModelTag = "SpiceInterfaceData"
        self.spiceObject.SPICEDataPath = path
        self.spiceObject.addPlanetNames(spiceInterface.StringVector(self.spicePlanetNames))
        self.spiceObject.UTCCalInit = time
        if len(self.spicePlanetFrames) > 0:
            if len(self.spicePlanetFrames) != len(self.spicePlanetNames):
                print("List arguments spicePlanetFrames and spicePlanetNames must contain the same number of strings.")
                exit(0)
            self.spiceObject.planetFrames = spiceInterface.StringVector(self.spicePlanetFrames)

        for fileName in self.spiceKernelFileNames:
            self.spiceObject.loadSpiceKernel(fileName, path)
        self.spiceObject.SPICELoaded = True

        # subscribe Grav Body data to the spice state message
        c = 0
        for key, gravBodyDataItem in self.gravBodies.items():
            gravBodyDataItem.planetBodyInMsg.subscribeTo(self.spiceObject.planetStateOutMsgs[c])
            c += 1

        # create and connect to an epoch input message
        if 'epochInMsg' in kwargs:
            if kwargs['epochInMsg']:
                self.epochMsg = unitTestSupport.timeStringToGregorianUTCMsg(time, dataPath=path)
                self.spiceObject.epochInMsg.subscribeTo(self.epochMsg)

        return

    def unloadSpiceKernels(self):
        for fileName in self.spiceKernelFileNames:
            self.spiceObject.unloadSpiceKernel(fileName, self.spiceObject.SPICEDataPath)
        return



def loadGravFromFile(fileName, spherHarm, maxDeg=2):
    """
            Load the gravitational body spherical harmonics coefficients from a file.

            Parameters
            ----------
            fileName : string
                The full path to the specified data file.
            spherHarm:
                The spherical harmonics container of the gravity body.
            maxDeg : integer
                maximum degree of spherical harmonics to load


            Notes
            -----
            This function is a convenience utility for loading in the spherical harmonics
            coefficients from a data file.  The default harmonic degree is 2 unless specified.
            Note that this function calls the gravityEffector function loadGravFromFile().
    """
    loadGravFromFile_python(fileName, spherHarm, maxDeg)
