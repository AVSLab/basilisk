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

from Basilisk.simulation import gravityEffector
from Basilisk.simulation import spice_interface
from Basilisk.simulation import simMessages


class gravBodyFactory(object):
    def __init__(self, bodyNames=None):
        self.spicePlanetNames = []
        self.gravBodies = {}
        self.spiceObject = None
        self.spiceKernelFileNames = []
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
                print "gravBody " + name + " not found in gravBodyUtilities.py"
        return self.gravBodies


    def createSun(self):
        sun = gravityEffector.GravBodyData()
        sun.bodyInMsgName = "sun_planet_data"
        sun.outputMsgName = "sun_display_frame_data"
        sun.mu = 1.32712440018E20  # meters^3/s^2
        sun.radEquator = 695508000.0  # meters
        sun.isCentralBody = False
        sun.useSphericalHarmParams = False
        self.gravBodies['sun'] = sun
        sun.this.disown()
        return sun

    def createMercury(self):
        mercury = gravityEffector.GravBodyData()
        mercury.bodyInMsgName = "mercury_planet_data"
        mercury.outputMsgName = "mercury_display_frame_data"
        mercury.mu = 4.28283100e13  # meters^3/s^2
        mercury.radEquator = 2439700.0  # meters
        mercury.isCentralBody = False
        mercury.useSphericalHarmParams = False
        self.gravBodies['mercury'] = mercury
        mercury.this.disown()
        return mercury

    def createVenus(self):
        venus = gravityEffector.GravBodyData()
        venus.bodyInMsgName = "venus_planet_data"
        venus.outputMsgName = "venus_display_frame_data"
        venus.mu = 3.24858599e14  # meters^3/s^2
        venus.radEquator = 6051800.0  # meters
        venus.isCentralBody = False
        venus.useSphericalHarmParams = False
        self.gravBodies['venus'] = venus
        venus.this.disown()
        return venus

    def createEarth(self):
        earth = gravityEffector.GravBodyData()
        earth.bodyInMsgName = "earth_planet_data"
        earth.outputMsgName = "earth_display_frame_data"
        earth.mu = 0.3986004415E+15  # meters^3/s^2
        earth.radEquator = 6378136.6  # meters
        earth.isCentralBody = False
        earth.useSphericalHarmParams = False
        self.gravBodies['earth'] = earth
        earth.this.disown()
        return earth

    def createMoon(self):
        moon = gravityEffector.GravBodyData()
        moon.bodyInMsgName = "moon_planet_data"
        moon.outputMsgName = "moon_display_frame_data"
        moon.mu = 4.902799E12  # meters^3/s^2
        moon.radEquator = 1738100.0  # meters
        moon.isCentralBody = False
        moon.useSphericalHarmParams = False
        self.gravBodies['moon'] = moon
        moon.this.disown()
        return moon

    def createMars(self):
        mars = gravityEffector.GravBodyData()
        mars.bodyInMsgName = "mars_planet_data"
        mars.outputMsgName = "mars_display_frame_data"
        mars.mu = 4.28283100e13  # meters^3/s^2
        mars.radEquator = 3396190  # meters
        mars.isCentralBody = False
        mars.useSphericalHarmParams = False
        self.gravBodies['mars'] = mars
        mars.this.disown()
        return mars

    def createMarsBarycenter(self):
        mars_barycenter = gravityEffector.GravBodyData()
        mars_barycenter.bodyInMsgName = "mars barycenter_planet_data"
        mars_barycenter.outputMsgName = "mars_barycenter_display_frame_data"
        mars_barycenter.mu = 4.28283100e13  # meters^3/s^2
        mars_barycenter.radEquator = 3396190  # meters
        mars_barycenter.isCentralBody = False
        mars_barycenter.useSphericalHarmParams = False
        self.gravBodies['mars barycenter'] = mars_barycenter
        mars_barycenter.this.disown()
        return mars_barycenter

    def createJupiter(self):
        jupiter = gravityEffector.GravBodyData()
        jupiter.bodyInMsgName = "jupiter barycenter_planet_data"
        jupiter.outputMsgName = "jupiter_display_frame_data"
        jupiter.mu = 1.266865349093058E17  # meters^3/s^2
        jupiter.radEquator = 71492000.0  # meters
        jupiter.isCentralBody = False
        jupiter.useSphericalHarmParams = False
        self.gravBodies['jupiter barycenter'] = jupiter
        jupiter.this.disown()
        return jupiter

    def createSaturn(self):
        saturn = gravityEffector.GravBodyData()
        saturn.bodyInMsgName = "saturn barycenter_planet_data"
        saturn.outputMsgName = "saturn_display_frame_data"
        saturn.mu = 3.79395000E16  # meters^3/s^2
        saturn.radEquator = 60268000.0  # meters
        saturn.isCentralBody = False
        saturn.useSphericalHarmParams = False
        self.gravBodies['saturn'] = saturn
        saturn.this.disown()
        return saturn

    def createUranus(self):
        uranus = gravityEffector.GravBodyData()
        uranus.bodyInMsgName = "uranus barycenter_planet_data"
        uranus.outputMsgName = "uranus_display_frame_data"
        uranus.mu = 5.79396566E15  # meters^3/s^2
        uranus.radEquator = 25559000.0  # meters
        uranus.isCentralBody = False
        uranus.useSphericalHarmParams = False
        self.gravBodies['uranus'] = uranus
        uranus.this.disown()
        return uranus

    def createNeptune(self):
        neptune = gravityEffector.GravBodyData()
        neptune.bodyInMsgName = "neptune barycenter_planet_data"
        neptune.outputMsgName = "neptune_display_frame_data"
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

            Returns
            -------
            spiceObject : Basilisk spice module
                A configured Basilisk spice module.
        """

        if kwargs.has_key('spiceKernalFileNames'):
            try:
                for fileName in kwargs['spiceKernalFileNames']:
                    self.spiceKernelFileNames.append(fileName)
            except(TypeError):
                raise TypeError('spiceKernalFileNames expects a list')
        else:
            self.spiceKernelFileNames.extend(['de430.bsp', 'naif0012.tls', 'de-403-masses.tpc', 'pck00010.tpc'])

        self.spicePlanetNames = []
        if kwargs.has_key('spicePlanetNames'):
            try:
                for planetName in kwargs['spicePlanetNames']:
                    self.spicePlanetNames.append(planetName)
            except(TypeError):
                raise TypeError('spicePlanetNames expects a list')
        else:
            self.spicePlanetNames = self.gravBodies.keys()

        self.spiceObject = spice_interface.SpiceInterface()
        self.spiceObject.ModelTag = "SpiceInterfaceData"
        self.spiceObject.SPICEDataPath = path
        self.spiceObject.outputBufferCount = 10000
        self.spiceObject.planetNames = spice_interface.StringVector(self.spicePlanetNames)
        self.spiceObject.UTCCalInit = time

        for fileName in self.spiceKernelFileNames:
            self.spiceObject.loadSpiceKernel(fileName, path)
        self.spiceObject.SPICELoaded = True

        return self.spiceObject

    def unloadSpiceKernels(self):
        for fileName in self.spiceKernelFileNames:
            self.spiceObject.unloadSpiceKernel(self.spiceObject.SPICEDataPath, fileName)
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
    gravityEffector.loadGravFromFile(fileName, spherHarm, maxDeg)

