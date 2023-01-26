
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



from collections import OrderedDict

from Basilisk.simulation import gravityEffector
from Basilisk.simulation import spiceInterface
from Basilisk.simulation.gravityEffector import loadGravFromFile as loadGravFromFile_python
from Basilisk.simulation.gravityEffector import loadPolyFromFile as loadPolyFromFile_python
from Basilisk.utilities import unitTestSupport


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

    # Note, in the `create` functions below the `isCentralBody` and `useSphericalHarmParams` are
    # all set to False in the `GravGodyData()` constructor.

    def createSun(self):
        """Create gravity body with sun mass properties."""
        sun = gravityEffector.GravBodyData()
        sun.planetName = "sun_planet_data"
        sun.displayName = "sun"
        sun.modelDictionaryKey = ""
        sun.mu = 1.32712440018E20  # meters^3/s^2
        sun.radEquator = 695508000.0  # meters
        self.gravBodies['sun'] = sun
        self.spicePlanetFrames.append("IAU_sun")
        sun.this.disown()
        return sun

    def createMercury(self):
        """Create gravity body with Mercury mass properties."""
        mercury = gravityEffector.GravBodyData()
        mercury.planetName = "mercury_planet_data"
        mercury.displayName = "mercury"
        mercury.modelDictionaryKey = ""
        mercury.mu = 4.28283100e13  # meters^3/s^2
        mercury.radEquator = 2439700.0  # meters
        self.gravBodies['mercury'] = mercury
        self.spicePlanetFrames.append("IAU_mercury")
        mercury.this.disown()
        return mercury

    def createVenus(self):
        """Create gravity body with Venus mass properties."""
        venus = gravityEffector.GravBodyData()
        venus.planetName = "venus_planet_data"
        venus.displayName = "venus"
        venus.modelDictionaryKey = ""
        venus.mu = 3.24858599e14  # meters^3/s^2
        venus.radEquator = 6051800.0  # meters
        self.gravBodies['venus'] = venus
        self.spicePlanetFrames.append("IAU_venus")
        venus.this.disown()
        return venus

    def createEarth(self):
        """Create gravity body with Earth mass properties."""
        earth = gravityEffector.GravBodyData()
        earth.planetName = "earth_planet_data"
        earth.displayName = "earth"
        earth.modelDictionaryKey = ""
        earth.mu = 0.3986004415E+15  # meters^3/s^2
        earth.radEquator = 6378136.6  # meters
        self.gravBodies['earth'] = earth
        self.spicePlanetFrames.append("IAU_earth")
        earth.this.disown()
        return earth

    def createMoon(self):
        """Create gravity body with Moon mass properties."""
        moon = gravityEffector.GravBodyData()
        moon.planetName = "moon_planet_data"
        moon.displayName = "moon"
        moon.modelDictionaryKey = ""
        moon.mu = 4.902799E12  # meters^3/s^2
        moon.radEquator = 1738100.0  # meters
        self.gravBodies['moon'] = moon
        self.spicePlanetFrames.append("IAU_moon")
        moon.this.disown()
        return moon

    def createMars(self):
        """Create gravity body with Mars mass properties."""
        mars = gravityEffector.GravBodyData()
        mars.planetName = "mars_planet_data"
        mars.displayName = "mars"
        mars.modelDictionaryKey = ""
        mars.mu = 4.28283100e13  # meters^3/s^2
        mars.radEquator = 3396190  # meters
        self.gravBodies['mars'] = mars
        self.spicePlanetFrames.append("IAU_mars")
        mars.this.disown()
        return mars

    def createMarsBarycenter(self):
        """Create gravity body with Mars mass properties."""
        mars_barycenter = gravityEffector.GravBodyData()
        mars_barycenter.planetName = "mars barycenter_planet_data"
        mars_barycenter.displayName = "mars"
        mars_barycenter.modelDictionaryKey = ""
        mars_barycenter.mu = 4.28283100e13  # meters^3/s^2
        mars_barycenter.radEquator = 3396190  # meters
        self.gravBodies['mars barycenter'] = mars_barycenter
        self.spicePlanetFrames.append("IAU_mars")
        mars_barycenter.this.disown()
        return mars_barycenter

    def createJupiter(self):
        """Create gravity body with Jupiter mass properties."""
        jupiter = gravityEffector.GravBodyData()
        jupiter.planetName = "jupiter barycenter_planet_data"
        jupiter.displayName = "jupiter"
        jupiter.modelDictionaryKey = ""
        jupiter.mu = 1.266865349093058E17  # meters^3/s^2
        jupiter.radEquator = 71492000.0  # meters
        self.gravBodies['jupiter barycenter'] = jupiter
        self.spicePlanetFrames.append("IAU_jupiter")
        jupiter.this.disown()
        return jupiter

    def createSaturn(self):
        """Create gravity body with Saturn mass properties."""
        saturn = gravityEffector.GravBodyData()
        saturn.planetName = "saturn barycenter_planet_data"
        saturn.displayName = "saturn"
        saturn.modelDictionaryKey = ""
        saturn.mu = 3.79395000E16  # meters^3/s^2
        saturn.radEquator = 60268000.0  # meters
        self.gravBodies['saturn'] = saturn
        self.spicePlanetFrames.append("IAU_saturn")
        saturn.this.disown()
        return saturn

    def createUranus(self):
        """Create gravity body with Uranus mass properties."""
        uranus = gravityEffector.GravBodyData()
        uranus.planetName = "uranus barycenter_planet_data"
        uranus.displayName = "uranus"
        uranus.modelDictionaryKey = ""
        uranus.mu = 5.79396566E15  # meters^3/s^2
        uranus.radEquator = 25559000.0  # meters
        self.gravBodies['uranus'] = uranus
        self.spicePlanetFrames.append("IAU_uranus")
        uranus.this.disown()
        return uranus

    def createNeptune(self):
        """Create gravity body with Neptune mass properties."""
        neptune = gravityEffector.GravBodyData()
        neptune.planetName = "neptune barycenter_planet_data"
        neptune.displayName = "neptune"
        neptune.modelDictionaryKey = ""
        neptune.mu = 6.83509920E15  # meters^3/s^2
        neptune.radEquator = 24764000.0  # meters
        self.gravBodies['neptune'] = neptune
        self.spicePlanetFrames.append("IAU_neptune")
        neptune.this.disown()
        return neptune

    def createCustomGravObject(self, label, mu, **kwargs):
        """
            Create a custom gravity body object.

            Parameters
            ----------
            label : string
                Gravity body name
            mu : double
                Gravity constant

            Other Parameters
            ----------------
            kwargs :
                radEquator : double
                    Equatorial radius in meters
                radiusRatio : double
                    Ratio of the polar radius to the equatorial radius.
                planetFrame : string
                    Name of the spice planet frame
                displayName: string
                    Vizard celestial body name, if not provided then planetFrame becomes the Vizard name
                modelDictionaryKey: string
                    Vizard model key name.  if not set, then either the displayName or planetName is used to set the model

        """
        unitTestSupport.checkMethodKeyword(
            ['radEquator', 'radiusRatio', 'planetFrame', 'displayName', 'modelDictionaryKey'],
            kwargs)

        if not isinstance(label, str):
            print('ERROR: label must be a string')
            exit(1)

        gravBody = gravityEffector.GravBodyData()
        gravBody.planetName = label
        gravBody.mu = mu
        if 'radEquator' in kwargs:
            gravBody.radEquator = kwargs['radEquator']
        if 'radiusRatio' in kwargs:
            gravBody.radiusRatio = kwargs['radiusRatio']
        if 'displayName' in kwargs:
            gravBody.displayName = kwargs['displayName']
        gravBody.modelDictionaryKey = ""
        if 'modelDictionaryKey' in kwargs:
            gravBody.modelDictionaryKey = kwargs['modelDictionaryKey']
        self.gravBodies[label] = gravBody
        planetFrame = ""
        if 'planetFrame' in kwargs:
            planetFrame = kwargs['planetFrame']
        self.spicePlanetFrames.append(planetFrame)
        gravBody.this.disown()
        return gravBody

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
                spiceKernalFileNames :
                    A list of spice kernel file names including file extension.
                spicePlanetNames :
                    A list of planet names whose Spice data is loaded, overriding the gravBodies list.
                spicePlanetFrames :
                    A list of strings for the planet frame names.  If left empty for a planet, then
                    ``IAU_`` + planetName is assumed for the planet frame.
                epochInMsg: bool
                    Flag to set an epoch input message for the spice interface

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

        if 'spicePlanetFrames' in kwargs:
            try:
                self.spicePlanetFrames = []
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
        """Method to unload spice kernals at the end of a simulation."""
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

def loadPolyFromFile(fileName, poly):
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
    loadPolyFromFile_python(fileName, poly)
