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

It requires the ``orekit_jpype`` package and a Java runtime. Cases without Earth rotation
evaluate the gravity field in the EME2000 frame, so the Earth pole is along inertial +Z.
Cases with Earth rotation evaluate it in ITRF (IERS 2010 conventions). The orekit-data folder
can be fetched with ``orekit_jpype.pyhelpers.download_orekit_data_curdir()``.
"""

import argparse
import csv
import tempfile
from pathlib import Path

import orekit_jpype

from comparisonCommon import HERE, loadSpec, writeOrekitGfc

SPEED_OF_LIGHT = 299792458.0  # [m/s]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("orekitData", type=Path, help="orekit-data.zip or folder")
    parser.add_argument("cases", nargs="*", help="case names to generate (default: all)")
    parser.add_argument("--output-dir", type=Path, default=HERE / "data",
                        help="folder where the reference ephemerides are written (default: data next to this script)")
    args = parser.parse_args()

    orekit_jpype.initVM()
    from orekit_jpype.pyhelpers import setup_orekit_curdir
    setup_orekit_curdir(str(args.orekitData))

    from jpype import JArray, JDouble, JImplements, JOverride
    from java.io import File
    from org.hipparchus.geometry.euclidean.threed import Vector3D
    from org.hipparchus.ode.nonstiff import DormandPrince853Integrator
    from org.orekit.bodies import CelestialBodyFactory, OneAxisEllipsoid
    from org.orekit.data import DataContext, DirectoryCrawler
    from org.orekit.forces.drag import DragForce, IsotropicDrag
    from org.orekit.forces.gravity import (HolmesFeatherstoneAttractionModel, NewtonianAttraction,
                                           ThirdBodyAttraction)
    from org.orekit.forces.gravity.potential import GravityFieldFactory, ICGEMFormatReader
    from org.orekit.forces.radiation import IsotropicRadiationSingleCoefficient, SolarRadiationPressure
    from org.orekit.frames import FramesFactory
    from org.orekit.models.earth.atmosphere import NRLMSISE00, NRLMSISE00InputParameters
    from org.orekit.orbits import CartesianOrbit, OrbitType
    from org.orekit.propagation import SpacecraftState
    from org.orekit.propagation.numerical import NumericalPropagator
    from org.orekit.time import AbsoluteDate, TimeScalesFactory
    from org.orekit.utils import IERSConventions, PVCoordinates

    spec = loadSpec()
    sc = spec["spacecraft"]
    outDir = args.output_dir
    outDir.mkdir(parents=True, exist_ok=True)
    mu = spec["mu_m3_s2"]  # [m^3/s^2]
    ae = spec["equatorial_radius_m"]  # [m]
    eme2000 = FramesFactory.getEME2000()
    itrf = FramesFactory.getITRF(IERSConventions.IERS_2010, True)
    utc = TimeScalesFactory.getUTC()
    e = spec["epoch_utc"]  # "YYYY-MM-DDTHH:MM:SS.sss"
    epoch = AbsoluteDate(int(e[0:4]), int(e[5:7]), int(e[8:10]), int(e[11:13]), int(e[14:16]),
                         float(e[17:]), utc)
    steps = int(round(spec["duration_s"] / spec["sample_period_s"]))
    earthShape = OneAxisEllipsoid(ae, spec["earth_flattening"], itrf)
    sun = CelestialBodyFactory.getSun()
    bodies = {"sun": sun, "moon": CelestialBodyFactory.getMoon()}

    @JImplements(NRLMSISE00InputParameters)
    class ConstantWeather:
        """Constant solar flux and geomagnetic activity."""

        @JOverride
        def getMinDate(self):
            return AbsoluteDate.PAST_INFINITY

        @JOverride
        def getMaxDate(self):
            return AbsoluteDate.FUTURE_INFINITY

        @JOverride
        def getAverageFlux(self, date):
            return float(spec["space_weather"]["f107"])

        @JOverride
        def getDailyFlux(self, date):
            return float(spec["space_weather"]["f107"])

        @JOverride
        def getAp(self, date):
            return JArray(JDouble)([float(spec["space_weather"]["ap"])] * 7)

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        DataContext.getDefault().getDataProvidersManager().addProvider(
            DirectoryCrawler(File(str(tmp))))

        for name, case in spec["cases"].items():
            if args.cases and name not in args.cases:
                continue
            degree, order = case["gravity"]["degree"], case["gravity"]["order"]
            gravityFrame = itrf if case["earth_rotation"] else eme2000

            pv = PVCoordinates(Vector3D(*case["r0_m"]), Vector3D(*case["v0_m_s"]))
            orbit = CartesianOrbit(pv, eme2000, epoch, mu)
            integrator = DormandPrince853Integrator(1.0e-3, 300.0, 1.0e-10, 1.0e-13)
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
                    dRef, pRef, sun, earthShape,
                    IsotropicRadiationSingleCoefficient(sc["srp_area_m2"], sc["srp_cr"])))
            if case["drag"]:
                atmosphere = NRLMSISE00(ConstantWeather(), sun, earthShape)
                propagator.addForceModel(DragForce(
                    atmosphere, IsotropicDrag(sc["drag_area_m2"], sc["drag_cd"])))

            state = SpacecraftState(orbit).withMass(sc["mass_kg"])
            propagator.setInitialState(state)
            with open(outDir / f"orekit_{name}.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s"])
                for k in range(steps + 1):
                    t = k * spec["sample_period_s"]  # [s]
                    if k > 0:
                        # restart from the previous sample to avoid re-propagating from t = 0
                        propagator.resetInitialState(state)
                        state = propagator.propagate(epoch.shiftedBy(float(t)))
                    pvOut = state.getPVCoordinates(eme2000)
                    p, v = pvOut.getPosition(), pvOut.getVelocity()
                    writer.writerow([f"{t:.1f}"] + [f"{c:.9f}" for c in
                                    (p.getX(), p.getY(), p.getZ(), v.getX(), v.getY(), v.getZ())])
            print(f"{name}: {steps + 1} samples")


if __name__ == "__main__":
    main()
