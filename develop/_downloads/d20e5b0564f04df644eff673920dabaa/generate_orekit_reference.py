#
#  ISC License
#
#  Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder
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
Generate the Orekit reference ephemerides used by the accuracy comparison.

This is a maintainer tool and is not run by CI. It reads ``cases.json``, propagates
each case with Orekit's numerical propagator, and stores the state in
``<output-dir>/orekit_<case>.csv`` (SI units, default ``data/``).

Usage::

    python generate_orekit_reference.py /path/to/orekit-data.zip [case ...] [--output-dir DIR]
                                        [--oblate-shadow] [--max-step SECONDS]

It requires the ``orekit_jpype`` package, version 13 or newer, and a Java runtime. Every case is propagated and
written in the GCRF frame, which has the ICRF axes of the DE430 SPICE kernels that Basilisk uses. Cases without
Earth rotation evaluate the gravity field in the GCRF frame, so the Earth pole is along inertial +Z.
Cases with Earth rotation evaluate it in ITRF (IERS 2010 conventions). The orekit-data folder
can be fetched with ``orekit_jpype.pyhelpers.download_orekit_data_curdir()``.

The radiation pressure uses a conical Earth shadow. By default the occulting body is a sphere of the equatorial radius, as
in Basilisk and GMAT, and ``--oblate-shadow`` uses Orekit's oblate Earth instead. The shadow transitions and the facets
of a box that turn in and out of view make the force non-smooth, which the adaptive integrator resolves only with a small
maximum step; the cases set it with ``orekit_max_step_s`` (``--max-step`` overrides it, default 300 s).

The drag uses Orekit's ``SimpleExponentialAtmosphere`` with the density at zero altitude and the scale height of the
``exponential_atmosphere`` entry of ``cases.json``, evaluated at the altitude above the oblate Earth. The generator also
writes ``orekit_density_probe.csv``: the altitude and the density that Orekit computes at the points of ``density_probe``,
which ``compare_with_basilisk.py`` compares with Basilisk and GMAT before it propagates anything.

Next to the ephemerides, ``<output-dir>/orekit_manifest.json`` records for every case the effective configuration and
its hash, the frame, the generator options, the tool version, the checksums of the external data and of the ephemeris,
and a ``variant`` label. A reference generated with ``--oblate-shadow`` or ``--max-step`` is labeled as such, and
``compare_with_basilisk.py`` refuses it unless it is asked for explicitly.

Cases with ``"body": "box"`` use Orekit's ``BoxAndSolarArraySpacecraft`` without solar array and a ``FixedRate`` attitude, so the
drag and the radiation pressure depend on the attitude.
"""

import argparse
import csv
import tempfile
from importlib import metadata
from pathlib import Path

from comparisonCommon import (DEFAULT_VARIANT, HERE, caseDuration, caseEpoch, fileSha256, gravityCoefficientsRecord,
                              loadSpec, mrp2dcm, probePoints, requireIcrf, writeManifestEntry, writeOrekitGfc, writeProbe)

SPEED_OF_LIGHT = 299792458.0  # [m/s]
MIN_OREKIT_MAJOR = 13  # [-] oldest Orekit major release with the API used here (e.g. SpacecraftState.withMass)


def requireOrekit(version):
    """Raise if the installed ``orekit_jpype`` is older than the Orekit release that this generator supports.

    The ``orekit_jpype`` version starts with the version of Orekit that it packages.

    Args:
        version (str): version of the installed ``orekit_jpype`` package, for example ``"13.1.8.0"``.
    """
    if int(version.split(".")[0]) < MIN_OREKIT_MAJOR:
        raise RuntimeError(f"Orekit {MIN_OREKIT_MAJOR} or newer is required, but orekit_jpype {version} is installed.")


def jarPath(location, fileClass):
    """Return the filesystem path of a jar from the URL of its code source.

    ``URL.getPath()`` keeps the URL escaping and does not give a native Windows path, so the URL is
    converted through ``java.io.File``, whose path is then explicitly converted to a Python string.

    Args:
        location (java.net.URL): code source location of the jar.
        fileClass (type): ``java.io.File`` class.
    """
    return Path(str(fileClass(location.toURI()).getPath()))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("orekitData", type=Path, help="orekit-data.zip or folder")
    parser.add_argument("cases", nargs="*", help="case names to generate (default: all)")
    parser.add_argument("--output-dir", type=Path, default=HERE / "data",
                        help="folder where the reference ephemerides are written (default: data next to this script)")
    parser.add_argument("--oblate-shadow", action="store_true",
                        help="use the oblate Earth as the occulting body of the radiation pressure (default: a sphere "
                        "of the equatorial radius, as in Basilisk and GMAT)")
    parser.add_argument("--max-step", type=float,
                        help="maximum integrator step in seconds, overriding the case value (default: 300 s)")
    args = parser.parse_args()

    requireOrekit(metadata.version("orekit_jpype"))
    import orekit_jpype  # imported here so that the helper functions of this module need no Java
    orekit_jpype.initVM()
    from orekit_jpype.pyhelpers import setup_orekit_curdir
    setup_orekit_curdir(str(args.orekitData))

    from jpype import JArray, JClass, JDouble
    from java.io import File
    from org.hipparchus.geometry.euclidean.threed import Vector3D
    from org.hipparchus.ode.nonstiff import DormandPrince853Integrator
    from org.orekit.bodies import CelestialBodyFactory, OneAxisEllipsoid
    from org.orekit.data import DataContext, DirectoryCrawler
    from org.orekit.attitudes import Attitude, FixedRate
    from org.orekit.forces import BoxAndSolarArraySpacecraft
    from org.orekit.forces.drag import DragForce, IsotropicDrag
    from org.orekit.forces.gravity import (HolmesFeatherstoneAttractionModel, NewtonianAttraction,
                                           ThirdBodyAttraction)
    from org.orekit.forces.gravity.potential import GravityFieldFactory, ICGEMFormatReader
    from org.orekit.forces.radiation import IsotropicRadiationSingleCoefficient, SolarRadiationPressure
    from org.orekit.frames import FramesFactory
    from org.hipparchus.geometry.euclidean.threed import Rotation
    from org.orekit.models.earth.atmosphere import SimpleExponentialAtmosphere
    from org.orekit.orbits import CartesianOrbit, OrbitType
    from org.orekit.propagation import SpacecraftState
    from org.orekit.propagation.numerical import NumericalPropagator
    from org.orekit.time import AbsoluteDate, TimeScalesFactory
    from org.orekit.utils import IERSConventions, PVCoordinates

    spec = loadSpec()
    requireIcrf(spec, "orekit")
    sc = spec["spacecraft"]
    outDir = args.output_dir
    outDir.mkdir(parents=True, exist_ok=True)
    mu = spec["mu_m3_s2"]  # [m^3/s^2]
    ae = spec["equatorial_radius_m"]  # [m]
    gcrf = FramesFactory.getGCRF()  # ICRF axes, as the J2000 frame of the DE430 SPICE kernels used by Basilisk
    itrf = FramesFactory.getITRF(IERSConventions.IERS_2010, True)
    utc = TimeScalesFactory.getUTC()

    def utcDate(e):
        """Return the Orekit date of the UTC string ``"YYYY-MM-DDTHH:MM:SS.sss"``."""
        return AbsoluteDate(int(e[0:4]), int(e[5:7]), int(e[8:10]), int(e[11:13]), int(e[14:16]),
                            float(e[17:]), utc)

    earthShape = OneAxisEllipsoid(ae, spec["earth_flattening"], itrf)
    shadowShape = OneAxisEllipsoid(ae, 0.0, itrf)  # spherical occulting body, as in Basilisk and GMAT
    sun = CelestialBodyFactory.getSun()
    bodies = {"sun": sun, "moon": CelestialBodyFactory.getMoon()}

    expAtm = spec["exponential_atmosphere"]
    def jarRecord(javaClass):
        """Return the file name and SHA-256 hash of the jar that provides a Java class."""
        jar = jarPath(JClass(javaClass).class_.getProtectionDomain().getCodeSource().getLocation(), File)
        return {"id": jar.name, "sha256": fileSha256(jar)}

    orekitJar = jarRecord("org.orekit.frames.FramesFactory")
    hipparchusJar = jarRecord("org.hipparchus.ode.nonstiff.DormandPrince853Integrator")
    toolVersion = (f"Orekit {orekitJar['id']}, Hipparchus {hipparchusJar['id']}, "
                   f"orekit_jpype {metadata.version('orekit_jpype')}")
    externalData = {
        "orekit_data": {"id": args.orekitData.name, "sha256": fileSha256(args.orekitData)},
        "orekit_jar": orekitJar,
        "hipparchus_jar": hipparchusJar,
        "gravity_coefficients": gravityCoefficientsRecord(spec),
    }

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        DataContext.getDefault().getDataProvidersManager().addProvider(
            DirectoryCrawler(File(str(tmp))))

        probeEpoch = utcDate(spec["epoch_utc"])
        probeAtmosphere = SimpleExponentialAtmosphere(
            earthShape, expAtm["density_at_zero_altitude_kg_m3"], 0.0, expAtm["scale_height_m"])
        probeRows = []
        for x, y, z, _ in probePoints(spec):  # Earth-fixed positions
            position = Vector3D(float(x), float(y), float(z))
            altitude = earthShape.transform(position, itrf, probeEpoch).getAltitude()  # [m]
            probeRows.append((x, y, z, altitude, probeAtmosphere.getDensity(probeEpoch, position, itrf)))
        writeProbe(outDir, "orekit", spec, probeRows, toolVersion, {"atmosphere": "SimpleExponentialAtmosphere"})
        print(f"density probe: {len(probeRows)} points")

        for name, case in spec["cases"].items():
            if args.cases and name not in args.cases:
                continue
            degree, order = case["gravity"]["degree"], case["gravity"]["order"]
            epoch = utcDate(caseEpoch(spec, case))
            steps = int(round(caseDuration(spec, case) / spec["sample_period_s"]))
            box = None
            if case.get("body", "cannonball") == "box":
                size, coeffs = spec["box"]["size_m"], spec["box"]
                box = BoxAndSolarArraySpacecraft(
                    float(size[0]), float(size[1]), float(size[2]), sun, 0.0, Vector3D(0.0, 1.0, 0.0),
                    float(coeffs["drag_cd"]), 0.0, float(coeffs["absorption"]), float(coeffs["specular_reflection"]))
            gravityFrame = itrf if case["earth_rotation"] else gcrf

            pv = PVCoordinates(Vector3D(*case["r0_m"]), Vector3D(*case["v0_m_s"]))
            orbit = CartesianOrbit(pv, gcrf, epoch, mu)
            maxStep = args.max_step or case.get("orekit_max_step_s", 300.0)  # [s]
            integrator = DormandPrince853Integrator(1.0e-3, maxStep, 1.0e-10, 1.0e-13)
            integrator.setInitialStepSize(10.0)  # [s]
            propagator = NumericalPropagator(integrator)
            propagator.setOrbitType(OrbitType.CARTESIAN)

            if degree > 0:
                gfc = f"{name}.gfc"
                writeOrekitGfc(tmp / gfc, spec, degree, order)
                GravityFieldFactory.clearPotentialCoefficientsReaders()
                GravityFieldFactory.addPotentialCoefficientsReader(ICGEMFormatReader(gfc, True))
                provider = GravityFieldFactory.getNormalizedProvider(degree, order)
                propagator.addForceModel(HolmesFeatherstoneAttractionModel(gravityFrame, provider))
            else:
                propagator.addForceModel(NewtonianAttraction(mu))
            for bodyName in case["third_bodies"]:
                propagator.addForceModel(ThirdBodyAttraction(bodies[bodyName]))
            if case["srp"]:
                pRef = spec["srp"]["solar_flux_w_m2"] / SPEED_OF_LIGHT  # [N/m^2]
                dRef = spec["srp"]["astronomical_unit_km"] * 1000.0  # [m]
                propagator.addForceModel(SolarRadiationPressure(
                    dRef, pRef, sun, earthShape if args.oblate_shadow else shadowShape,
                    box if box is not None else
                    IsotropicRadiationSingleCoefficient(sc["srp_area_m2"], sc["srp_cr"])))
            if case["drag"]:
                atmosphere = SimpleExponentialAtmosphere(
                    earthShape, expAtm["density_at_zero_altitude_kg_m3"], 0.0, expAtm["scale_height_m"])
                propagator.addForceModel(DragForce(
                    atmosphere, box if box is not None else IsotropicDrag(sc["drag_area_m2"], sc["drag_cd"])))
            if box is not None:
                att = case["attitude"]
                dcm = mrp2dcm(att["sigma_BN"])  # [BN], maps inertial components to body components
                rotation = Rotation(JArray(JArray(JDouble))([[float(x) for x in row] for row in dcm]), 1.0e-10)
                spin = Vector3D(*[float(w) for w in att["omega_BN_B_rad_s"]])  # [rad/s] in the body frame
                propagator.setAttitudeProvider(FixedRate(Attitude(epoch, gcrf, rotation, spin, Vector3D.ZERO)))

            state = SpacecraftState(orbit).withMass(sc["mass_kg"])
            propagator.setInitialState(state)
            csvPath = outDir / f"orekit_{name}.csv"
            with open(csvPath, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s"])
                for k in range(steps + 1):
                    t = k * spec["sample_period_s"]  # [s]
                    if k > 0:
                        # restart from the previous sample to avoid re-propagating from t = 0
                        propagator.resetInitialState(state)
                        state = propagator.propagate(epoch.shiftedBy(float(t)))
                    pvOut = state.getPVCoordinates(gcrf)
                    p, v = pvOut.getPosition(), pvOut.getVelocity()
                    writer.writerow([f"{t:.1f}"] + [f"{c:.9f}" for c in
                                    (p.getX(), p.getY(), p.getZ(), v.getX(), v.getY(), v.getZ())])
            options = {"oblate_shadow": args.oblate_shadow, "max_step_s": maxStep,
                       "max_step_overridden": args.max_step is not None,
                       "integrator": {"type": "DormandPrince853", "min_step_s": 1.0e-3,
                                      "abs_tolerance_m": 1.0e-10, "rel_tolerance": 1.0e-13,
                                      "initial_step_s": 10.0}}
            variants = ((["oblate_shadow"] if args.oblate_shadow else [])
                        + ([f"max_step_{args.max_step:g}s"] if args.max_step is not None else []))
            writeManifestEntry(outDir, "orekit", spec, name, csvPath, spec["inertial_frame"], toolVersion, options,
                               externalData, "+".join(variants) if variants else DEFAULT_VARIANT)
            print(f"{name}: {steps + 1} samples")


if __name__ == "__main__":
    main()
