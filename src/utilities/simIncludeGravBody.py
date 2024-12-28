# ISC License
#
# Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

from typing import Optional
from typing import Dict
from typing import Iterable
from typing import Union
from typing import overload
from typing import Sequence
from typing import List
from typing import Protocol
from typing import runtime_checkable
from typing import Any
from dataclasses import dataclass

from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.architecture import astroConstants
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import spiceInterface
from Basilisk.simulation.gravityEffector import (
    loadGravFromFile as loadGravFromFile_python,
)
from Basilisk.simulation.gravityEffector import (
    loadPolyFromFile as loadPolyFromFile_python,
)
from Basilisk.utilities import unitTestSupport

from Basilisk.utilities.deprecated import deprecationWarn

# this statement is needed to enable Windows to print ANSI codes in the Terminal
# see https://stackoverflow.com/questions/287871/how-to-print-colored-text-in-terminal-in-python/3332860#3332860
import os
os.system("")

@dataclass
class BodyData:
    """A class that contains information about a body in the simulation.

    Args:
        identifier (str): The SPICE identifier of the body.
        planetName (str): The name that identifies the body within the simulation.
        displayName (str): The name for the body in the Vizard display.
        modelDictionaryKey (str): Vizard model key name.
        mu (float): Gravitational parameter in m^3/s^2.
        radEquator (float): Equatorial radius of the body in meters.
        spicePlanetFrame (str): The name of the SPICE frame (attitude provider).
        radiusRatio (float, optional): Used to compute ellipticity. It is
            provided for bodies in the basic Vizard body dictionary. Defaults to 1.
    """

    identifier: str
    planetName: str
    displayName: str
    modelDictionaryKey: str
    mu: float
    radEquator: float
    spicePlanetFrame: str
    radiusRatio: float = 1


BODY_DATA = {
    "sun": BodyData(
        identifier="sun",
        planetName="sun_planet_data",
        displayName="sun",
        modelDictionaryKey="",
        mu=astroConstants.MU_SUN*1e9,
        radEquator=astroConstants.REQ_SUN*1e3,
        spicePlanetFrame="IAU_sun",
    ),
    "mercury": BodyData(
        identifier="mercury",
        planetName="mercury_planet_data",
        displayName="mercury",
        modelDictionaryKey="",
        mu=astroConstants.MU_MERCURY*1e9,
        radEquator=astroConstants.REQ_MERCURY*1e3,
        spicePlanetFrame="IAU_mercury",
    ),
    "venus": BodyData(
        identifier="venus",
        planetName="venus_planet_data",
        displayName="venus",
        modelDictionaryKey="",
        mu=astroConstants.MU_VENUS*1e9,
        radEquator=astroConstants.REQ_VENUS*1e3,
        spicePlanetFrame="IAU_venus",
    ),
    "earth": BodyData(
        identifier="earth",
        planetName="earth_planet_data",
        displayName="earth",
        modelDictionaryKey="",
        mu=astroConstants.MU_EARTH*1e9,
        radEquator=astroConstants.REQ_EARTH*1e3,
        spicePlanetFrame="IAU_earth",
    ),
    "moon": BodyData(
        identifier="moon",
        planetName="moon_planet_data",
        displayName="moon",
        modelDictionaryKey="",
        mu=astroConstants.MU_MOON*1e9,
        radEquator=astroConstants.REQ_MOON*1e3,
        spicePlanetFrame="IAU_moon",
    ),
    "mars": BodyData(
        identifier="mars",
        planetName="mars_planet_data",
        displayName="mars",
        modelDictionaryKey="",
        mu=astroConstants.MU_MARS*1e9,
        radEquator=astroConstants.REQ_MARS*1e3,
        spicePlanetFrame="IAU_mars",
    ),
    "mars barycenter": BodyData(
        identifier="mars barycenter",
        planetName="mars barycenter_planet_data",
        displayName="mars barycenter",
        modelDictionaryKey="",
        mu=astroConstants.MU_MARS*1e9,
        radEquator=astroConstants.REQ_MARS*1e3,
        spicePlanetFrame="IAU_mars",
    ),
    "jupiter barycenter": BodyData(
        identifier="jupiter barycenter",
        planetName="jupiter barycenter_planet_data",
        displayName="jupiter",
        modelDictionaryKey="",
        mu=astroConstants.MU_JUPITER*1e9,
        radEquator=astroConstants.REQ_JUPITER*1e3,
        spicePlanetFrame="IAU_jupiter",
    ),
    "saturn": BodyData(
        identifier="saturn",
        planetName="saturn barycenter_planet_data",
        displayName="saturn",
        modelDictionaryKey="",
        mu=astroConstants.MU_SATURN*1e9,
        radEquator=astroConstants.REQ_SATURN*1e3,
        spicePlanetFrame="IAU_saturn",
    ),
    "uranus": BodyData(
        identifier="uranus",
        planetName="uranus barycenter_planet_data",
        displayName="uranus",
        modelDictionaryKey="",
        mu=astroConstants.MU_URANUS*1e9,
        radEquator=astroConstants.REQ_URANUS*1e3,
        spicePlanetFrame="IAU_uranus",
    ),
    "neptune": BodyData(
        identifier="neptune",
        planetName="neptune barycenter_planet_data",
        displayName="neptune",
        modelDictionaryKey="",
        mu=astroConstants.MU_NEPTUNE*1e9,
        radEquator=astroConstants.REQ_NEPTUNE*1e3,
        spicePlanetFrame="IAU_neptune",
    ),
}


@runtime_checkable
class WithGravField(Protocol):
    gravField: Any  # cannot be GravityEffector because SWIG classes dont give type info


class gravBodyFactory:
    """Class to create gravitational bodies."""

    def __init__(self, bodyNames: Iterable[str] = []):
        self.spicePlanetFrames: List[str] = []
        self.gravBodies: Dict[str, gravityEffector.GravBodyData] = {}
        self.spiceObject: Optional[spiceInterface.SpiceInterface] = None
        self.spiceKernelFileNames: List[str] = []
        self.epochMsg: Optional[messaging.EpochMsg] = None
        if bodyNames:
            self.createBodies(bodyNames)

    def addBodiesTo(
        self,
        objectToAddTheBodies: Union[gravityEffector.GravityEffector, WithGravField],
    ):
        """Can be called with a GravityEffector or an object that has a gravField
        variable to set the gravity bodies used in the object.
        """
        bodies = gravityEffector.GravBodyVector(list(self.gravBodies.values()))
        if isinstance(objectToAddTheBodies, WithGravField):
            objectToAddTheBodies = objectToAddTheBodies.gravField
        objectToAddTheBodies.setGravBodies(bodies)  # type: ignore

    # Note, in the `create` functions below the `isCentralBody` and `useSphericalHarmParams` are
    # all set to False in the `GravGodyData()` constructor.

    def createBody(
        self, bodyData: Union[str, BodyData]
    ) -> gravityEffector.GravBodyData:
        """Convenience function to create a body given its name.

        Args:
            bodyData (Union[str, BodyData]):
                A valid SPICE celestial body string or a BodyData class with
                the relevant data.

        Returns:
            gravityEffector.GravBodyData: The body object with corresponding data.
        """
        if not isinstance(bodyData, BodyData):
            key = bodyData.lower().replace("_", " ")
            if key not in BODY_DATA:
                raise ValueError(
                    f"Body {bodyData} is unkown. "
                    f"Valid options are: {', '.join(BODY_DATA)}"
                )

            bodyData = BODY_DATA[key]

        body = gravityEffector.GravBodyData()
        body.planetName = bodyData.planetName
        body.displayName = bodyData.displayName
        body.modelDictionaryKey = bodyData.modelDictionaryKey
        body.mu = bodyData.mu
        body.radEquator = bodyData.radEquator
        body.radiusRatio = bodyData.radiusRatio
        self.gravBodies[bodyData.identifier] = body
        self.spicePlanetFrames.append(bodyData.spicePlanetFrame)
        return body

    def createBodies(
        self, *bodyNames: Union[str, Iterable[str]]
    ) -> Dict[str, gravityEffector.GravBodyData]:
        """A convenience function to create multiple typical solar system bodies.

        Args:
            bodyNames (Union[str, Iterable[str]]):
                Planet name strings. Each planet name must be a valid SPICE celestial body string.

        Returns:
            Dict[str, gravityEffector.GravBodyData]:
                A dictionary of gravity body objects held by the gravity factory.
        """
        for nameOrIterableOfNames in bodyNames:
            if isinstance(nameOrIterableOfNames, str):
                self.createBody(nameOrIterableOfNames)
            else:
                for name in nameOrIterableOfNames:
                    self.createBody(name)
        return self.gravBodies

    def createSun(self):
        return self.createBody("sun")

    def createMercury(self):
        return self.createBody("mercury")

    def createVenus(self):
        return self.createBody("venus")

    def createEarth(self):
        return self.createBody("earth")

    def createMoon(self):
        return self.createBody("moon")

    def createMars(self):
        return self.createBody("mars")

    def createMarsBarycenter(self):
        return self.createBody("mars barycenter")

    def createJupiter(self):
        return self.createBody("jupiter barycenter")

    def createSaturn(self):
        return self.createBody("saturn")

    def createUranus(self):
        return self.createBody("uranus")

    def createNeptune(self):
        return self.createBody("neptune")

    def createCustomGravObject(
        self,
        label: str,
        mu: float,
        displayName: Optional[str] = None,
        modelDictionaryKey: Optional[str] = None,
        radEquator: Optional[float] = None,
        radiusRatio: Optional[float] = None,
        planetFrame: Optional[str] = None,
    ) -> gravityEffector.GravBodyData:
        """Create a custom gravity body object.

        Args:
            label (str): Gravity body name
            mu (float): Gravity constant in m^3/s^2
            displayName (Optional[str], optional): Vizard celestial body name, if not
                provided then planetFrame becomes the Vizard name. Defaults to None.
            modelDictionaryKey (Optional[str], optional): Vizard model key name.
                If not set, then either the displayName or planetName is used to
                set the model. Defaults to None.
            radEquator (Optional[float], optional): Equatorial radius in meters.
                Defaults to None.
            radiusRatio (Optional[float], optional): Ratio of the polar radius to
                the equatorial radius. Defaults to None.
            planetFrame (Optional[str], optional): Name of the spice planet
                frame. Defaults to None.

        Returns:
            gravityEffector.GravBodyData: The body object with the given data.
        """
        gravBody = gravityEffector.GravBodyData()
        gravBody.planetName = label
        gravBody.mu = mu
        if radEquator is not None:
            gravBody.radEquator = radEquator
        if radiusRatio is not None:
            gravBody.radiusRatio = radiusRatio
        if displayName is not None:
            gravBody.displayName = displayName
        if modelDictionaryKey is None:
            gravBody.modelDictionaryKey = ""
        else:
            gravBody.modelDictionaryKey = modelDictionaryKey
        self.gravBodies[label] = gravBody
        if planetFrame is not None:
            self.spicePlanetFrames.append(planetFrame)
        return gravBody

    @overload
    def createSpiceInterface(
        self,
        path: str,
        time: str,
        spiceKernelFileNames: Iterable[str] = [
                                              "de430.bsp",
                                              "naif0012.tls",
                                              "de-403-masses.tpc",
                                              "pck00010.tpc",
                                              ],
        spicePlanetNames: Optional[Sequence[str]] = None,
        spicePlanetFrames: Optional[Sequence[str]] = None,
        epochInMsg: bool = False,
    ) -> spiceInterface.SpiceInterface:
        """A convenience function to configure a NAIF Spice module for the simulation.
        It connects the gravBodyData objects to the spice planet state messages.  Thus,
        it must be run after the gravBodyData objects are created.

        Args:
            path (str): The absolute path to the folder that contains the
                kernels to be loaded.
            time (str): The time string in a format that SPICE recognizes.
            spiceKernelFileNames (Iterable[str], optional): A list of spice kernel file
                names including file extension. Defaults to
                `['de430.bsp', 'naif0012.tls', 'de-403-masses.tpc', 'pck00010.tpc']`.
            spicePlanetNames (Optional[Sequence[str]], optional): A list of planet names
                whose Spice data is loaded. If this is not provided, Spice data is
                loaded for the bodies created with this factory object. Defaults to None.
            spicePlanetFrames (Optional[Sequence[str]], optional): A list of planet
                frame names to load in Spice. If this is not provided, frames are loaded
                for the bodies created with this factory function. Defaults to None.
            epochInMsg (bool, optional): Flag to set an epoch input message for the
                spice interface. Defaults to False.

        Returns:
            spiceInterface.SpiceInterface: The generated SpiceInterface, which is the
            same as the stored `self.spiceObject`
        """

    @overload
    def createSpiceInterface(
        self,
        *,
        path: str = "%BSK_PATH%/supportData/EphemerisData/",
        time: str,
        spiceKernelFileNames: Iterable[str] = [
                                              "de430.bsp",
                                              "naif0012.tls",
                                              "de-403-masses.tpc",
                                              "pck00010.tpc",
                                              ],
        spicePlanetNames: Optional[Sequence[str]] = None,
        spicePlanetFrames: Optional[Sequence[str]] = None,
        epochInMsg: bool = False,
    ) -> spiceInterface.SpiceInterface:
        """A convenience function to configure a NAIF Spice module for the simulation.
        It connect the gravBodyData objects to the spice planet state messages.  Thus,
        it must be run after the gravBodyData objects are created.

        Unless the 'path' input is provided, the kernels are loaded from the folder:
        `%BSK_PATH%/supportData/EphemerisData/`, where `%BSK_PATH%` is replaced by
        the Basilisk directory.

        Args:
            path (str): The absolute path to the folder that contains the
                kernels to be loaded.
            time (str): The time string in a format that SPICE recognizes.
            spiceKernelFileNames (Iterable[str], optional): A list of spice kernel file
                names including file extension. Defaults to
                `['de430.bsp', 'naif0012.tls', 'de-403-masses.tpc', 'pck00010.tpc']`.
            spicePlanetNames (Optional[Sequence[str]], optional): A list of planet names
                whose Spice data is loaded. If this is not provided, Spice data is
                loaded for the bodies created with this factory object. Defaults to None.
            spicePlanetFrames (Optional[Sequence[str]], optional): A list of planet
                frame names to load in Spice. If this is not provided, frames are loaded
                for the bodies created with this factory function. Defaults to None.
            epochInMsg (bool, optional): Flag to set an epoch input message for the
                spice interface. Defaults to False.

        Returns:
            spiceInterface.SpiceInterface: The generated SpiceInterface, which is the
            same as the stored `self.spiceObject`
        """

    def createSpiceInterface(
        self,
        path: str = "%BSK_PATH%/supportData/EphemerisData/",
        time: str = "",
        spiceKernelFileNames: Iterable[str] = [
                                              "de430.bsp",
                                              "naif0012.tls",
                                              "de-403-masses.tpc",
                                              "pck00010.tpc",
                                              ],
        spicePlanetNames: Optional[Sequence[str]] = None,
        spicePlanetFrames: Optional[Sequence[str]] = None,
        epochInMsg: bool = False,
        spiceKernalFileNames = None,
    ) -> spiceInterface.SpiceInterface:
        if time == "":
            raise ValueError(
                "'time' argument must be provided and a valid SPICE time string"
            )

        if spiceKernalFileNames is not None:
            spiceKernelFileNames = spiceKernalFileNames
            deprecationWarn(
                f"{gravBodyFactory.createSpiceInterface.__qualname__}.spiceKernalFileNames"
                "2024/11/24",
                "The argument 'spiceKernalFileNames' is deprecated, as it is a "
                "misspelling of 'spiceKernelFileNames'"
            )

        path = path.replace("%BSK_PATH%", list(__path__)[0])

        self.spiceKernelFileNames.extend(spiceKernelFileNames)
        self.spicePlanetNames = list(spicePlanetNames or self.gravBodies)
        if spicePlanetFrames is not None:
            self.spicePlanetFrames = list(spicePlanetFrames)

        self.spiceObject = spiceInterface.SpiceInterface()
        self.spiceObject.ModelTag = "SpiceInterfaceData"
        self.spiceObject.SPICEDataPath = path
        self.spiceObject.addPlanetNames(self.spicePlanetNames)
        self.spiceObject.UTCCalInit = time
        if len(self.spicePlanetFrames) > 0:
            self.spiceObject.planetFrames = list(self.spicePlanetFrames)

        self.spiceObject.SPICELoaded = True
        for fileName in set(self.spiceKernelFileNames):
            if self.spiceObject.loadSpiceKernel(fileName, path):
                # error occured loading spice kernel
                self.spiceObject.SPICELoaded = False
                if fileName == "de430.bsp":
                    print("\033[91mERROR loading the large file de430.bsp:\033[0m  If BSK was "
                          "installed via a wheel, try running bskLargeData from the console to "
                          "install the large BSK data files.\n")

        # subscribe Grav Body data to the spice state message
        for c, gravBodyDataItem in enumerate(self.gravBodies.values()):
            gravBodyDataItem.planetBodyInMsg.subscribeTo(
                self.spiceObject.planetStateOutMsgs[c]
            )

        # create and connect to an epoch input message
        if epochInMsg:
            self.epochMsg = unitTestSupport.timeStringToGregorianUTCMsg(
                time, dataPath=path
            )
            self.spiceObject.epochInMsg.subscribeTo(self.epochMsg)

        return self.spiceObject

    def unloadSpiceKernels(self):
        """Method to unload spice kernals at the end of a simulation."""
        if self.spiceObject is None:
            return
        for fileName in set(self.spiceKernelFileNames):
            self.spiceObject.unloadSpiceKernel(fileName, self.spiceObject.SPICEDataPath)


def loadGravFromFile(
    fileName: str,
    spherHarm: gravityEffector.SphericalHarmonicsGravityModel,
    maxDeg: int = 2,
):
    """Load the gravitational body spherical harmonics coefficients from a file.

    Note that this function calls the `gravityEffector` function `loadGravFromFile()`.

    Args:
        fileName (str): The full path to the specified data file.
        spherHarm (gravityEffector.SphericalHarmonicsGravityModel):
            The spherical harmonics container of the gravity body.
        maxDeg (int, optional): Maximum degree of spherical harmonics to load.
            Defaults to 2.
    """
    loadGravFromFile_python(fileName, spherHarm, maxDeg)


def loadPolyFromFile(fileName: str, poly: gravityEffector.PolyhedralGravityModel):
    """Load the gravitational body polyhedral coefficients from a file.

    Args:
        fileName (str): The full path to the specified data file.
        poly (gravityEffector.PolyhedralGravityModel):
            The polyhedarl gravity model container of the body.
    """
    loadPolyFromFile_python(fileName, poly)
