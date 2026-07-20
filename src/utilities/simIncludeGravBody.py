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

"""Create gravity-body descriptors and connect them to Basilisk dynamics.

The usual workflow is to create the bodies first, mark one body as central when
appropriate, and then attach the factory bodies to a dynamics object:

.. code-block:: python

    from Basilisk.utilities import simIncludeGravBody

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    gravFactory.addBodiesTo(spacecraftObject)

The same ``addBodiesTo()`` call accepts a MuJoCo ``MJScene``. In that case the
factory creates the intermediate ``NBodyGravity`` model and connects every
MuJoCo body as a gravity target.

Call ``createSpiceInterface()`` after creating the gravity bodies. The factory
then connects each body's ephemeris input to the corresponding SPICE output.
"""

import os
from dataclasses import dataclass
from pathlib import Path
from typing import (
    Any,
    Dict,
    Iterable,
    List,
    Optional,
    Protocol,
    Sequence,
    Union,
    overload,
    runtime_checkable,
)

from Basilisk import __path__
from Basilisk.architecture import astroConstants
from Basilisk.architecture import messaging
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import spiceInterface
from Basilisk.simulation.gravityEffector import (
    loadGravFromFile as loadGravFromFile_python,
)
from Basilisk.simulation.gravityEffector import (
    loadPolyFromFile as loadPolyFromFile_python,
)
from Basilisk.utilities import simHelpers

from Basilisk.utilities.deprecated import deprecationWarn
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, POOCH, get_path

# Enable ANSI color codes in the Windows terminal.
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


# ``astroConstants`` stores gravitational parameters in km^3/s^2 and radii in
# km. The multipliers below convert them to the SI units used by the dynamics.
BODY_DATA = {
    "sun": BodyData(
        identifier="sun",
        planetName="sun_planet_data",
        displayName="sun",
        modelDictionaryKey="",
        mu=astroConstants.MU_SUN * 1e9,
        radEquator=astroConstants.REQ_SUN * 1e3,
        spicePlanetFrame="IAU_sun",
    ),
    "mercury": BodyData(
        identifier="mercury",
        planetName="mercury_planet_data",
        displayName="mercury",
        modelDictionaryKey="",
        mu=astroConstants.MU_MERCURY * 1e9,
        radEquator=astroConstants.REQ_MERCURY * 1e3,
        spicePlanetFrame="IAU_mercury",
    ),
    "venus": BodyData(
        identifier="venus",
        planetName="venus_planet_data",
        displayName="venus",
        modelDictionaryKey="",
        mu=astroConstants.MU_VENUS * 1e9,
        radEquator=astroConstants.REQ_VENUS * 1e3,
        spicePlanetFrame="IAU_venus",
    ),
    "earth": BodyData(
        identifier="earth",
        planetName="earth_planet_data",
        displayName="earth",
        modelDictionaryKey="",
        mu=astroConstants.MU_EARTH * 1e9,
        radEquator=astroConstants.REQ_EARTH * 1e3,
        spicePlanetFrame="IAU_earth",
    ),
    "moon": BodyData(
        identifier="moon",
        planetName="moon_planet_data",
        displayName="moon",
        modelDictionaryKey="",
        mu=astroConstants.MU_MOON * 1e9,
        radEquator=astroConstants.REQ_MOON * 1e3,
        spicePlanetFrame="IAU_moon",
    ),
    "mars": BodyData(
        identifier="mars",
        planetName="mars_planet_data",
        displayName="mars",
        modelDictionaryKey="",
        mu=astroConstants.MU_MARS * 1e9,
        radEquator=astroConstants.REQ_MARS * 1e3,
        spicePlanetFrame="IAU_mars",
    ),
    "mars barycenter": BodyData(
        identifier="mars barycenter",
        planetName="mars barycenter_planet_data",
        displayName="mars barycenter",
        modelDictionaryKey="",
        mu=astroConstants.MU_MARS * 1e9,
        radEquator=astroConstants.REQ_MARS * 1e3,
        spicePlanetFrame="IAU_mars",
    ),
    "jupiter barycenter": BodyData(
        identifier="jupiter barycenter",
        planetName="jupiter barycenter_planet_data",
        displayName="jupiter",
        modelDictionaryKey="",
        mu=astroConstants.MU_JUPITER * 1e9,
        radEquator=astroConstants.REQ_JUPITER * 1e3,
        spicePlanetFrame="IAU_jupiter",
    ),
    "saturn": BodyData(
        identifier="saturn",
        planetName="saturn barycenter_planet_data",
        displayName="saturn",
        modelDictionaryKey="",
        mu=astroConstants.MU_SATURN * 1e9,
        radEquator=astroConstants.REQ_SATURN * 1e3,
        spicePlanetFrame="IAU_saturn",
    ),
    "uranus": BodyData(
        identifier="uranus",
        planetName="uranus barycenter_planet_data",
        displayName="uranus",
        modelDictionaryKey="",
        mu=astroConstants.MU_URANUS * 1e9,
        radEquator=astroConstants.REQ_URANUS * 1e3,
        spicePlanetFrame="IAU_uranus",
    ),
    "neptune": BodyData(
        identifier="neptune",
        planetName="neptune barycenter_planet_data",
        displayName="neptune",
        modelDictionaryKey="",
        mu=astroConstants.MU_NEPTUNE * 1e9,
        radEquator=astroConstants.REQ_NEPTUNE * 1e3,
        spicePlanetFrame="IAU_neptune",
    ),
}


@runtime_checkable
class WithGravField(Protocol):
    # SWIG classes do not expose enough static type information to name the
    # concrete gravity-effector type here. Runtime protocol checking lets the
    # factory recognize objects such as ``spacecraft.Spacecraft`` by interface.
    gravField: Any


@runtime_checkable
class WithSetGravBodies(Protocol):
    def setGravBodies(self, gravBodies: Any) -> None:
        ...


def _ensure_trailing_sep(path: str) -> str:
    """Return a directory path in the format expected by the SPICE C API."""
    path = str(path)
    return path if path.endswith(os.sep) else path + os.sep


class gravBodyFactory:
    """Create and retain the gravity bodies used by a simulation.

    ``gravBodies`` is the factory's canonical dictionary of
    ``GravBodyData`` descriptors. A descriptor can be shared by conventional
    spacecraft dynamics, MuJoCo dynamics, SPICE, and Vizard without copying its
    gravity model or configuration.
    """

    def __init__(self, bodyNames: Iterable[str] = []):
        # Body names and any supplied body-fixed frames are kept in insertion
        # order because SPICE output messages are connected in that same order.
        self.spicePlanetFrames: List[str] = []
        self.gravBodies: Dict[str, gravityEffector.GravBodyData] = {}
        self.spiceObject: Optional[spiceInterface.SpiceInterface] = None
        self.spiceKernelFileNames: List[str] = []
        self.epochMsg: Optional[messaging.EpochMsg] = None
        if bodyNames:
            self.createBodies(bodyNames)

    def addBodiesTo(
        self,
        objectToAddTheBodies: Any,
    ) -> Optional[Any]:
        """Attach the factory's gravity bodies to a dynamics object.

        A conventional dynamics object may be a ``GravityEffector`` or expose a
        ``gravField`` attribute. When an ``MJScene`` is provided, this method
        creates one ``NBodyGravity`` model, adds every factory body as a gravity
        source, and adds every MuJoCo body as a gravity target.

        Args:
            objectToAddTheBodies (Any): Dynamics object that should receive the
                factory's gravity bodies.

        Returns:
            Optional[Any]: The created ``NBodyGravity`` model for an ``MJScene``;
                otherwise, ``None``.

        Raises:
            TypeError: If the supplied object is not a supported dynamics object.
            ValueError: If gravity was already attached to the ``MJScene``.

        .. note::

           When factory bodies use SPICE ephemerides, add the factory's
           ``spiceObject`` to the ``MJScene`` dynamics task before calling this
           method. This keeps planet states current at each MuJoCo integrator
           substep.
        """
        # Standard dynamics objects such as ``Spacecraft`` own a gravity
        # effector in ``gravField``. Accepting either the owner or the effector
        # keeps the long-standing ``addBodiesTo()`` calling convention.
        if isinstance(objectToAddTheBodies, WithGravField):
            objectToAddTheBodies = objectToAddTheBodies.gravField

        # GravityEffectors expose ``setGravBodies``. Checking for that protocol
        # first preserves the conventional path without importing MuJoCo.
        if isinstance(objectToAddTheBodies, WithSetGravBodies):
            bodies = gravityEffector.GravBodyVector(list(self.gravBodies.values()))
            objectToAddTheBodies.setGravBodies(bodies)
            return None

        return self._addBodiesToMJScene(objectToAddTheBodies)

    def _addBodiesToMJScene(self, scene: Any) -> Any:
        """Create and attach an ``NBodyGravity`` model for a MuJoCo scene."""
        # MuJoCo is an optional Basilisk build feature. Import it only after the
        # conventional gravity-effector path has been ruled out.
        try:
            from Basilisk.simulation import NBodyGravity
            from Basilisk.simulation import mujoco
        except ImportError as error:
            raise TypeError(
                "The supplied object does not support gravity-body attachment, "
                "and this Basilisk build does not include MuJoCo."
            ) from error

        if not isinstance(scene, mujoco.MJScene):
            raise TypeError(
                "addBodiesTo() requires a GravityEffector, an object with a "
                "gravField attribute, or an MJScene."
            )

        # Attaching twice would duplicate every gravity force actuator. Keep a
        # scene-side marker so that the error is caught before modifying it.
        cacheAttribute = "_gravBodyFactoryNBodyGravity"
        if getattr(scene, cacheAttribute, None) is not None:
            raise ValueError(
                f"Gravity bodies have already been added to MJScene '{scene.ModelTag}'."
            )

        gravity = NBodyGravity.NBodyGravity()
        gravity.ModelTag = (
            f"{scene.ModelTag}_gravity" if scene.ModelTag else "mujocoScene_gravity"
        )

        # A factory body is a gravity *source*. The descriptor overload lets
        # NBodyGravity read its model, central-body flag, and SPICE input
        # directly from the same GravBodyData used by standard spacecraft.
        for name, gravBody in self.gravBodies.items():
            gravity.addGravitySourceFromBody(name, gravBody)

        # Gravity is applied independently at each MuJoCo body's center of
        # mass. This preserves gravity-gradient forces across articulated or
        # otherwise extended systems.
        for name in scene.getBodyNames():
            gravity.addGravityTarget(name, scene.getBody(name))

        scene.AddModelToDynamicsTask(gravity)

        # MJScene stores raw model pointers in its dynamics task, so retain the
        # Python model for at least as long as the scene.
        setattr(scene, cacheAttribute, gravity)

        # Match the standard spacecraft path, where Vizard discovers the
        # attached gravity bodies through the dynamics object.
        scene._vizGravBodies = self.gravBodies

        return gravity

    # ``GravBodyData`` starts with a point-mass gravity model and is not a
    # central body. Users should set ``isCentralBody = True`` on the appropriate
    # body and may replace the point-mass model before simulation initialization.

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
            # Accept both ``"mars barycenter"`` and ``"mars_barycenter"`` to
            # match common SPICE-name spelling in user scripts.
            key = bodyData.lower().replace("_", " ")
            if key not in BODY_DATA:
                raise ValueError(
                    f"Body {bodyData} is unknown. "
                    f"Valid options are: {', '.join(BODY_DATA)}"
                )

            bodyData = BODY_DATA[key]

        # The descriptor owns the gravity configuration; dynamics objects only
        # retain a shared reference to it when ``addBodiesTo()`` is called.
        body = gravityEffector.GravBodyData()
        body.planetName = bodyData.planetName
        body.displayName = bodyData.displayName
        body.modelDictionaryKey = bodyData.modelDictionaryKey
        body.mu = bodyData.mu
        body.radEquator = bodyData.radEquator
        body.radiusRatio = bodyData.radiusRatio

        # Dictionary insertion order becomes the default SPICE planet-output
        # order. ``spicePlanetFrames`` is populated in the same order.
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
        # Support either ``createBodies("earth", "moon")`` or
        # ``createBodies(["earth", "moon"])`` without making users reshape
        # their existing collection.
        for nameOrIterableOfNames in bodyNames:
            if isinstance(nameOrIterableOfNames, str):
                self.createBody(nameOrIterableOfNames)
            else:
                for name in nameOrIterableOfNames:
                    self.createBody(name)
        return self.gravBodies

    # Named helpers below are intentionally thin wrappers. They provide
    # discoverable IDE completion while keeping all initialization in
    # ``createBody()``.
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
            label (str): Gravity body name.
            mu (float): Gravitational parameter in m^3/s^2.
            displayName (Optional[str], optional): Vizard celestial-body name. If
                omitted, Vizard uses the gravity body's ``planetName``. Defaults
                to None.
            modelDictionaryKey (Optional[str], optional): Vizard model key name.
                If not set, then either the displayName or planetName is used to
                set the model. Defaults to None.
            radEquator (Optional[float], optional): Equatorial radius in meters.
                Defaults to None.
            radiusRatio (Optional[float], optional): Ratio of the polar radius to
                the equatorial radius. Defaults to None.
            planetFrame (Optional[str], optional): Name of the SPICE planet
                frame. Defaults to None.

        Returns:
            gravityEffector.GravBodyData: The body object with the given data.
        """
        # The GravBodyData constructor supplies the default point-mass gravity
        # model. This method fills in the independent parameters used to
        # initialize that model when the simulation starts.
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

        # Retaining the descriptor in ``gravBodies`` makes this custom body
        # participate in later ``addBodiesTo()`` and SPICE setup calls.
        self.gravBodies[label] = gravBody
        if planetFrame is not None:
            self.spicePlanetFrames.append(planetFrame)
        return gravBody

    # The two overloads document the supported positional and keyword-only
    # calling styles for type checkers. The third definition contains the
    # runtime implementation shared by both styles.
    @overload
    def createSpiceInterface(
        self,
        path: str,
        time: str,
        spiceKernelFileNames: Iterable[DataFile.EphemerisData] = (
            DataFile.EphemerisData.de430,
            DataFile.EphemerisData.naif0012,
            DataFile.EphemerisData.de_403_masses,
            DataFile.EphemerisData.pck00010,
        ),
        spicePlanetNames: Optional[Sequence[str]] = None,
        spicePlanetFrames: Optional[Sequence[str]] = None,
        epochInMsg: bool = False,
    ) -> spiceInterface.SpiceInterface:
        """Configure a NAIF SPICE module for the simulation.

        This method connects the factory's ``GravBodyData`` objects to the
        corresponding SPICE planet-state messages. It must therefore be called
        after the gravity bodies are created.

        .. note::

           ``spicePlanetFrames`` specifies the output frames used to compute
           ``J20002Pfix`` for each planet state output message. If this argument is not
           set, the method uses the frames gathered when bodies were created (typically
           ``IAU_*`` frame names).

           Alternatives to ``IAU_*`` can be provided as long as SPICE can resolve the
           frame IDs from loaded kernels. For example, ``"ITRF93"`` can be used for
           Earth-fixed outputs when Earth high precision kernels are available, and
           ``"J2000"`` can be used to request an inertial-aligned output frame.

           Earth frame-association FK kernels (for example ``earth_assoc_itrf93.tf``)
           do not override ``spicePlanetFrames``.

        Args:
            path (str): The absolute path to the folder that contains the kernels to be loaded.
            time (str): The time string in a format that SPICE recognizes.
            spiceKernelFileNames (Iterable[DataFile.EphemerisData], optional):
                Support-data entries for the SPICE kernels to load. Defaults to
                ``de430.bsp``, ``naif0012.tls``, ``de-403-masses.tpc``, and
                ``pck00010.tpc``.
            spicePlanetNames (Optional[Sequence[str]], optional):
                Planet names whose SPICE data is loaded. If omitted, the method
                uses the bodies created with this factory, in insertion order.
                Defaults to None.
            spicePlanetFrames (Optional[Sequence[str]], optional):
                Planet frame names to load in SPICE. If omitted, the method uses
                the frames recorded when the factory bodies were created.
                Defaults to None.
            epochInMsg (bool, optional):
                If True, create and connect an epoch input message. Defaults to
                False.

        Returns:
            spiceInterface.SpiceInterface: The generated ``SpiceInterface``.
                The same object is also available as ``self.spiceObject``.
        """

    @overload
    def createSpiceInterface(
        self,
        *,
        path: str = "%BSK_PATH%/supportData/EphemerisData/",
        time: str,
        spiceKernelFileNames: Iterable[DataFile.EphemerisData] = (
            DataFile.EphemerisData.de430,
            DataFile.EphemerisData.naif0012,
            DataFile.EphemerisData.de_403_masses,
            DataFile.EphemerisData.pck00010,
        ),
        spicePlanetNames: Optional[Sequence[str]] = None,
        spicePlanetFrames: Optional[Sequence[str]] = None,
        epochInMsg: bool = False,
    ) -> spiceInterface.SpiceInterface:
        """Configure a NAIF SPICE module for the simulation.

        This method connects the factory's ``GravBodyData`` objects to the
        corresponding SPICE planet-state messages. It must therefore be called
        after the gravity bodies are created.

        Unless the ``path`` input is provided, the kernels are loaded from the folder:
        `%BSK_PATH%/supportData/EphemerisData/`, where `%BSK_PATH%` is replaced by
        the Basilisk directory.

        .. note::

           ``spicePlanetFrames`` specifies the output frames used to compute
           ``J20002Pfix`` for each planet state output message. If this argument is not
           set, the method uses the frames gathered when bodies were created (typically
           ``IAU_*`` frame names).

           Alternatives to ``IAU_*`` can be provided as long as SPICE can resolve the
           frame IDs from loaded kernels. For example, ``"ITRF93"`` can be used for
           Earth-fixed outputs when Earth high precision kernels are available, and
           ``"J2000"`` can be used to request an inertial-aligned output frame.

           Earth frame-association FK kernels (for example ``earth_assoc_itrf93.tf``)
           do not override ``spicePlanetFrames``.

        Args:
            path (str): The absolute path to the folder that contains the kernels to be loaded.
            time (str): The time string in a format that SPICE recognizes.
            spiceKernelFileNames (Iterable[DataFile.EphemerisData], optional):
                Support-data entries for the SPICE kernels to load. Defaults to
                ``de430.bsp``, ``naif0012.tls``, ``de-403-masses.tpc``, and
                ``pck00010.tpc``.
            spicePlanetNames (Optional[Sequence[str]], optional):
                Planet names whose SPICE data is loaded. If omitted, the method
                uses the bodies created with this factory, in insertion order.
                Defaults to None.
            spicePlanetFrames (Optional[Sequence[str]], optional):
                Planet frame names to load in SPICE. If omitted, the method uses
                the frames recorded when the factory bodies were created.
                Defaults to None.
            epochInMsg (bool, optional):
                If True, create and connect an epoch input message. Defaults to
                False.

        Returns:
            spiceInterface.SpiceInterface: The generated ``SpiceInterface``.
                The same object is also available as ``self.spiceObject``.
        """

    def createSpiceInterface(
        self,
        path: str = "%BSK_PATH%/supportData/EphemerisData/",
        time: str = "",
        spiceKernelFileNames: Iterable[DataFile.EphemerisData] = (
            DataFile.EphemerisData.de430,
            DataFile.EphemerisData.naif0012,
            DataFile.EphemerisData.de_403_masses,
            DataFile.EphemerisData.pck00010,
        ),
        spicePlanetNames: Optional[Sequence[str]] = None,
        spicePlanetFrames: Optional[Sequence[str]] = None,
        epochInMsg: bool = False,
        spiceKernalFileNames=None,
    ) -> spiceInterface.SpiceInterface:
        """Configure a NAIF SPICE module for the simulation.

        This method connects the factory's ``GravBodyData`` objects to the
        corresponding SPICE planet-state messages. It must therefore be called
        after the gravity bodies are created.

        Unless the ``path`` input is provided, the kernels are loaded from the folder:
        `%BSK_PATH%/supportData/EphemerisData/`, where `%BSK_PATH%` is replaced by
        the Basilisk directory.

        .. note::

           ``spicePlanetFrames`` specifies the output frames used to compute
           ``J20002Pfix`` for each planet state output message. If this argument is not
           set, the method uses the frames gathered when bodies were created (typically
           ``IAU_*`` frame names).

           Alternatives to ``IAU_*`` can be provided as long as SPICE can resolve the
           frame IDs from loaded kernels. For example, ``"ITRF93"`` can be used for
           Earth-fixed outputs when Earth high precision kernels are available, and
           ``"J2000"`` can be used to request an inertial-aligned output frame.

           Earth frame-association FK kernels (for example ``earth_assoc_itrf93.tf``)
           do not override ``spicePlanetFrames``.

        Args:
            path (str): The absolute path to the folder that contains the kernels to be loaded.
            time (str): The time string in a format that SPICE recognizes.
            spiceKernelFileNames (Iterable[DataFile.EphemerisData], optional):
                Support-data entries for the SPICE kernels to load. Defaults to
                ``de430.bsp``, ``naif0012.tls``, ``de-403-masses.tpc``, and
                ``pck00010.tpc``.
            spicePlanetNames (Optional[Sequence[str]], optional):
                Planet names whose SPICE data is loaded. If omitted, the method
                uses the bodies created with this factory, in insertion order.
                Defaults to None.
            spicePlanetFrames (Optional[Sequence[str]], optional):
                Planet frame names to load in SPICE. If omitted, the method uses
                the frames recorded when the factory bodies were created.
                Defaults to None.
            epochInMsg (bool, optional):
                If True, create and connect an epoch input message. Defaults to
                False.
            spiceKernalFileNames: Deprecated misspelling of
                ``spiceKernelFileNames``.

        Returns:
            spiceInterface.SpiceInterface: The generated ``SpiceInterface``.
                The same object is also available as ``self.spiceObject``.
        """
        if time == "":
            raise ValueError(
                "'time' must be provided as a valid SPICE time string"
            )

        # Keep the historical misspelling working while directing users to the
        # correctly spelled keyword.
        if spiceKernalFileNames is not None:
            spiceKernelFileNames = spiceKernalFileNames
            deprecationWarn(
                f"{gravBodyFactory.createSpiceInterface.__qualname__}.spiceKernalFileNames",
                "2024/11/24",
                "The argument 'spiceKernalFileNames' is deprecated. Use 'spiceKernelFileNames'",
            )

        # ``%BSK_PATH%`` is retained for compatibility with older scripts. The
        # resolved path is also passed to the optional epoch-message helper.
        if isinstance(path, str) and "%BSK_PATH%" in path:
            path = path.replace("%BSK_PATH%", list(__path__)[0])

        path = Path(path).expanduser().resolve()

        # Keep the interface on the factory so callers can schedule it and later
        # unload its kernels through this same factory instance.
        self.spiceObject = spiceInterface.SpiceInterface()
        self.spiceObject.ModelTag = "SpiceInterfaceData"
        self.spiceObject.SPICEDataPath = (
            str(Path(POOCH.abspath) / "supportData" / "EphemerisData") + os.sep
        )
        self.spiceKernelFileNames.extend(spiceKernelFileNames)

        # Python dictionaries preserve insertion order. By default, SPICE
        # outputs therefore follow the same order as ``gravBodies`` below.
        # Callers who provide ``spicePlanetNames`` must keep that ordering
        # consistent with the factory bodies they intend to connect.
        self.spicePlanetNames = list(spicePlanetNames or self.gravBodies)
        if spicePlanetFrames is not None:
            self.spicePlanetFrames = list(spicePlanetFrames)

        self.spiceObject.addPlanetNames(self.spicePlanetNames)
        self.spiceObject.UTCCalInit = time
        if len(self.spicePlanetFrames) > 0:
            self.spiceObject.planetFrames = list(self.spicePlanetFrames)

        self.spiceObject.SPICELoaded = True
        # A set avoids loading the same kernel more than once if this factory
        # accumulated duplicate entries.
        for file_enum in set(self.spiceKernelFileNames):
            # Prefer the support-data registry. It selects a local checkout
            # when available and otherwise retrieves the cached data file.
            try:
                resolved_path = Path(get_path(file_enum)).resolve()
            except Exception:
                # User-supplied filenames may not be registry entries. Fall
                # back to the interface's SPICE data directory for those.
                fname = getattr(file_enum, "value", str(file_enum))
                resolved_path = (Path(self.spiceObject.SPICEDataPath) / fname).resolve()

            # ``loadSpiceKernel`` accepts the filename and containing directory
            # separately, so split the resolved absolute path at this boundary.
            full_kernel_path = str(resolved_path)
            resolved_dir = str(resolved_path.parent) + os.sep
            resolved_name = resolved_path.name
            load_failed = self.spiceObject.loadSpiceKernel(resolved_name, resolved_dir)

            if load_failed:
                self.spiceObject.SPICELoaded = False
                print(f"\033[91mERROR loading kernel:\033[0m {full_kernel_path}")

        # Connect each shared descriptor to the matching SPICE output. Both
        # conventional GravityEffector and MuJoCo NBodyGravity later read this
        # same ``planetBodyInMsg`` connection.
        for c, gravBodyDataItem in enumerate(self.gravBodies.values()):
            gravBodyDataItem.planetBodyInMsg.subscribeTo(
                self.spiceObject.planetStateOutMsgs[c]
            )

        if epochInMsg:
            # Store the message on the factory so its Python owner remains alive
            # for as long as the SPICE interface is in use.
            self.epochMsg = simHelpers.timeStringToGregorianUTCMsg(
                time, dataPath=str(path)
            )
            self.spiceObject.epochInMsg.subscribeTo(self.epochMsg)

        return self.spiceObject

    def unloadSpiceKernels(self):
        """Unload this factory's SPICE kernels at the end of a simulation."""
        if self.spiceObject is None:
            return
        for file_enum in set(self.spiceKernelFileNames):
            # Kernel lists normally contain DataFile enum members, but retain
            # support for callers that supplied filename strings.
            fname = getattr(file_enum, "value", str(file_enum))

            d = _ensure_trailing_sep(self.spiceObject.SPICEDataPath)
            self.spiceObject.unloadSpiceKernel(fname, d)


def loadGravFromFile(
    fileName: str,
    spherHarm: gravityEffector.SphericalHarmonicsGravityModel,
    maxDeg: int = 2,
):
    """Load the gravitational body spherical harmonics coefficients from a file.

    This wrapper delegates to ``gravityEffector.loadGravFromFile()``.

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
            The polyhedral gravity model container of the body.
    """
    loadPolyFromFile_python(fileName, poly)
