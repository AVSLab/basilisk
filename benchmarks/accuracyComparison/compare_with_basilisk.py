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
Compare Basilisk's orbit propagation with the GMAT and Orekit reference ephemerides.

This is a rough comparison. The three tools differ in their features and in how a force model is set up, the results
are not regenerated automatically, and they go out of date as the tools are released.

The cases are defined in ``cases.json``. The reference ephemerides are produced beforehand with
``generate_gmat_reference.py`` and ``generate_orekit_reference.py`` (GMAT and Orekit are needed only for that step) and
are read from ``data/`` or the folder given with ``--data-dir``. Each generator also writes a manifest with the effective
case configuration, the generator options, the frame, the tool version and the checksums of the data; it is validated
before Basilisk is run, so a reference that was generated for another epoch, configuration or frame is rejected. A
reference generated with an alternative configuration (for example ``--oblate-shadow``) is only accepted with
``--orekit-variant`` naming it, and the results then carry that variant. Basilisk is run with the same initial
conditions, inertial frame (ICRF), gravity field, third bodies, solar radiation pressure, and exponential-atmosphere drag
(for a cannonball or a box spacecraft) as the reference tools. The maximum position and velocity differences to each
reference, and between the references, are printed. The methodology and results are described in
:ref:`accuracyComparison`.

Usage::

    python compare_with_basilisk.py [--cases leo_all geo_all] [--duration-days 30] [--kernel-dir DIR]
                                    [--data-dir DIR] [--figures-dir DIR] [--orekit-variant NAME]

The cases with a rotating Earth use the high-precision Earth orientation kernel and the ITRF93 frame association
kernel (``earth_000101_260711_260415.bpc`` and ``earth_assoc_itrf93.tf``), which are not part of Basilisk's
support data. They are searched in ``--kernel-dir``, in ``data/spice`` next to this script, and in the Basilisk
support-data cache.

The atmosphere altitude is computed above the same ellipsoid as in the other tools. Before it propagates a case with
drag, the script compares the altitude and the density that the three tools compute at the points of ``density_probe``
in ``cases.json`` (the probe files written by the generators), and stops if the altitude definition or the density
model differ beyond the tolerances given there.
"""

import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np
from Basilisk.architecture import bskLogging, messaging
from Basilisk.simulation import (dragDynamicEffector, eclipse, exponentialAtmosphere, facetDragDynamicEffector,
                                 facetSRPDynamicEffector, radiationPressure, spacecraft, svIntegrators,
                                 zeroWindModel)
from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile

try:
    from matplotlib import pyplot as plt
except ImportError:
    plt = None

DATA_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(DATA_DIR))
from comparisonCommon import (DEFAULT_VARIANT, TOOLS, boxFacets, caseDuration, caseEpoch, caseReferences, checkProbe,
                              loadProbe, loadSpec, validateManifestEntry, writeBasiliskGravity)

DEFAULT_KERNELS = (DataFile.EphemerisData.de430, DataFile.EphemerisData.naif0012,
                   DataFile.EphemerisData.de_403_masses, DataFile.EphemerisData.pck00010)
ITRF_KERNELS = ["earth_000101_260711_260415.bpc", "earth_assoc_itrf93.tf"]
SPICE_FRAMES = {"earth": "ITRF93", "sun": "IAU_SUN", "moon": "IAU_MOON"}
SAMPLE_TIME_TOLERANCE = 1.0e-3  # [s] allowed difference between reference and Basilisk sample times
RKF78_REL_TOL = 1.0e-4  # [-] relative tolerance of the Basilisk RKF78 integrator (the library default)
RKF78_ABS_TOL = 1.0e-8  # [m, m/s, ...] absolute tolerance of the Basilisk RKF78 integrator (the library default)
MONTHS = ["JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"]


def loadReference(tool, caseName, spec, dataDir=None, variant=DEFAULT_VARIANT):
    """Load a reference ephemeris as an array with columns (t, r_xyz, v_xyz) in SI units, after validating its manifest.

    Args:
        tool (str): ``"gmat"`` or ``"orekit"``.
        caseName (str): case name in ``cases.json``.
        spec (dict): parsed ``cases.json`` content.
        dataDir (Path): folder with the reference ephemerides; ``data`` next to this script by default.
        variant (str): variant of the reference that is accepted, ``"default"`` unless an alternative configuration
            is compared on purpose.
    """
    folder = Path(dataDir or DATA_DIR / "data")
    path = folder / f"{tool}_{caseName}.csv"
    if not path.exists():
        raise FileNotFoundError(
            f"The {tool} reference ephemeris {path} does not exist. Generate it with "
            f"generate_{tool}_reference.py first (see Reproducing the Results in the accuracy comparison "
            f"documentation, ``accuracyComparison``), or give the folder that contains it with --data-dir.")
    validateManifestEntry(folder, tool, spec, caseName, path, variant)
    return np.loadtxt(path, delimiter=",", skiprows=1)


def validateReference(tool, caseName, reference, bsk):
    """Return the part of a reference ephemeris that matches the Basilisk samples, after checking it.

    The reference must have the same columns, at least as many samples as Basilisk, and the same sample times.
    Otherwise states of different epochs would be compared and the reported differences would be misleading.

    Args:
        tool (str): ``"gmat"`` or ``"orekit"``.
        caseName (str): case name in ``cases.json``.
        reference (ndarray): reference ephemeris with columns (t, r_xyz, v_xyz).
        bsk (ndarray): Basilisk ephemeris with columns (t, r_xyz, v_xyz).
    """
    where = f"The {tool} reference ephemeris of case {caseName}"
    if reference.ndim != 2 or reference.shape[1] != bsk.shape[1]:
        raise ValueError(f"{where} has shape {reference.shape}; {bsk.shape[1]} columns (t, r_xyz, v_xyz) are expected.")
    if len(reference) < len(bsk):
        raise ValueError(f"{where} has {len(reference)} samples but Basilisk produced {len(bsk)}. Regenerate it with "
                         f"generate_{tool}_reference.py using the same cases.json, or use a shorter --duration-days.")
    reference = reference[:len(bsk)]
    timeError = np.abs(reference[:, 0] - bsk[:, 0]).max()  # [s]
    if timeError > SAMPLE_TIME_TOLERANCE:
        raise ValueError(f"The sample times of the {tool} reference ephemeris of case {caseName} differ from the Basilisk "
                         f"sample times by up to {timeError:.3g} s. Regenerate the reference with the sample period of "
                         f"cases.json.")
    return reference


def findKernelDir(kernelDir=None):
    """Return the folder that contains the high-precision Earth orientation kernels.

    Args:
        kernelDir (Path): folder given by the user, searched first.
    """
    candidates = [Path(kernelDir)] if kernelDir else []
    candidates += [DATA_DIR / "data" / "spice",
                   Path.home() / ".cache" / "bsk_support_data" / "supportData" / "EphemerisData"]
    for folder in candidates:
        if all((folder / name).exists() for name in ITRF_KERNELS):
            return folder
    raise FileNotFoundError(f"The Earth orientation kernels {ITRF_KERNELS} were not found in "
                            f"{[str(d) for d in candidates]}. Use --kernel-dir.")


def spiceTime(spec, case):
    """Return the case epoch as a SPICE UTC time string.

    Args:
        spec (dict): parsed ``cases.json`` content.
        case (dict): the case entry.
    """
    e = caseEpoch(spec, case)  # "YYYY-MM-DDTHH:MM:SS.sss"
    return f"{e[0:4]} {MONTHS[int(e[5:7]) - 1]} {e[8:10]} {e[11:]} (UTC)"


def configureExponentialAtmosphere(spec, atmo):
    """Configure a Basilisk exponential atmosphere with the shared model of ``cases.json``.

    The density at zero altitude and the scale height are passed as given, and the altitude is computed above the
    ellipsoid of the Earth. The propagation and the density probe use this same function.

    Args:
        spec (dict): parsed ``cases.json`` content.
        atmo (ExponentialAtmosphere): the module to configure.
    """
    expAtm = spec["exponential_atmosphere"]
    atmo.planetRadius = spec["equatorial_radius_m"]  # [m]
    atmo.setPlanetPolarRadius(spec["equatorial_radius_m"] * (1.0 - spec["earth_flattening"]))  # [m]
    atmo.baseDensity = expAtm["density_at_zero_altitude_kg_m3"]  # [kg/m^3]
    atmo.scaleHeight = expAtm["scale_height_m"]  # [m]


def probeBasilisk(spec, positions):
    """Return the density [kg/m^3] of Basilisk's exponential atmosphere at Earth-fixed positions.

    One update of the module with one spacecraft per position and a planet at the origin with identity orientation, so
    that the planet-fixed and the inertial frame coincide.

    Args:
        spec (dict): parsed ``cases.json`` content.
        positions (ndarray): [m] Earth-fixed positions, one row per point.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("probe")
    process.addTask(scSim.CreateNewTask("probeTask", macros.sec2nano(1.0)))
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.ModelTag = "probeAtmosphere"
    configureExponentialAtmosphere(spec, atmo)
    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.J20002Pfix = np.eye(3).tolist()  # [-]
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload)
    atmo.planetPosInMsg.subscribeTo(planetMsg)
    messages = []
    for position in positions:
        payload = messaging.SCStatesMsgPayload()
        payload.r_BN_N = [float(c) for c in position]
        messages.append(messaging.SCStatesMsg().write(payload))
        atmo.addSpacecraftToModel(messages[-1])
    scSim.AddModelToTask("probeTask", atmo)
    recorders = [msg.recorder() for msg in atmo.envOutMsgs]
    for recorder in recorders:
        scSim.AddModelToTask("probeTask", recorder)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(0)
    scSim.ExecuteSimulation()
    return np.array([recorder.neutralDensity[-1] for recorder in recorders])


def compareDensityProbe(spec, dataDir=None):
    """Check that Basilisk, GMAT and Orekit evaluate the same altitude and density at the probe points.

    Args:
        spec (dict): parsed ``cases.json`` content.
        dataDir (Path): folder with the probe files; ``data`` next to this script by default.

    Raises:
        ValueError: if a probe file is missing or stale, or a tolerance of ``density_probe`` is exceeded.
    """
    folder = Path(dataDir or DATA_DIR / "data")
    print("density probe: altitude error [m] / density difference to Basilisk [-]")
    for tool in TOOLS:
        rows = loadProbe(folder, tool, spec)
        results = checkProbe(spec, tool, rows, probeBasilisk(spec, rows[:, :3]))
        print(f"  {tool}: max |dh| = {max(abs(r[1]) for r in results):.3e} m, "
              f"max |drho/rho| = {max(abs(r[2]) for r in results):.3e}")


def propagateBasilisk(spec, case, gravityFile, kernelDir=None, durationSeconds=None):
    """Propagate one case with Basilisk and return an array with columns (t, r_xyz, v_xyz).

    Args:
        spec (dict): parsed ``cases.json`` content.
        case (dict): the case entry.
        gravityFile (Path): spherical-harmonics file used when the gravity degree is positive.
        kernelDir (Path): folder with the Earth orientation kernels, used by the cases with a rotating Earth.
        durationSeconds (float): [s] propagation time; the duration of the case by default.
    """
    sc = spec["spacecraft"]
    isBox = case.get("body", "cannonball") == "box"
    stepSeconds = case["basilisk_step_s"]  # [s]
    thirdBodies = case["third_bodies"]
    needSpice = case["earth_rotation"] or thirdBodies or case["srp"] or case["drag"]

    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("dynamicsProcess")
    proc.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(stepSeconds)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraft"
    integrator = svIntegrators.svIntegratorRKF78(scObject)
    integrator.relTol = RKF78_REL_TOL  # [-]
    integrator.absTol = RKF78_ABS_TOL  # [m, m/s, ...]
    scObject.setIntegrator(integrator)
    scObject.hub.mHub = sc["mass_kg"]  # [kg]
    scObject.hub.r_CN_NInit = case["r0_m"]
    scObject.hub.v_CN_NInit = case["v0_m_s"]
    if isBox:
        # spherical inertia and facets with the center of pressure at the center of mass: no torque changes the spin
        scObject.hub.IHubPntBc_B = [[500.0, 0.0, 0.0], [0.0, 500.0, 0.0], [0.0, 0.0, 500.0]]  # [kg*m^2]
        scObject.hub.sigma_BNInit = case["attitude"]["sigma_BN"]
        scObject.hub.omega_BN_BInit = case["attitude"]["omega_BN_B_rad_s"]  # [rad/s]
    scSim.AddModelToTask("dynamicsTask", scObject)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.mu = spec["mu_m3_s2"]
    earth.radEquator = spec["equatorial_radius_m"]
    earth.isCentralBody = True
    if case["gravity"]["degree"] > 0:
        earth.useSphericalHarmonicsGravityModel(str(gravityFile), case["gravity"]["degree"])
    effectiveBodies = [earth]
    if thirdBodies:
        extra = {"sun": gravFactory.createSun, "moon": gravFactory.createMoon}
        effectiveBodies += [extra[b]() for b in thirdBodies]
    if case["srp"] and "sun" not in thirdBodies:
        gravFactory.createSun()  # needed for the SPICE sun state message only

    spiceObject = None
    if needSpice:
        kernels = list(DEFAULT_KERNELS) + ITRF_KERNELS
        folder = findKernelDir(kernelDir)
        spiceObject = gravFactory.createSpiceInterface(
            path=str(folder) + "/", time=spiceTime(spec, case), epochInMsg=True, spiceKernelFileNames=kernels,
            spicePlanetFrames=[SPICE_FRAMES[name] for name in gravFactory.gravBodies])
        spiceObject.zeroBase = "Earth"
        scSim.AddModelToTask("dynamicsTask", spiceObject, 100)
    else:
        # No planet-orientation message is connected on purpose: the gravity effector then uses an identity
        # orientation, which keeps the pole along inertial +Z as in GMAT and Orekit. Silence the expected warning.
        quietLogger = bskLogging.BSKLogger()
        quietLogger.setLogLevel(bskLogging.BSK_ERROR)
        scObject.gravField.bskLogger = quietLogger
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(effectiveBodies)

    bodyNames = list(gravFactory.gravBodies)
    if case["srp"]:
        eclipseObject = eclipse.Eclipse()
        eclipseObject.setExtrapolateScStateToStepMidpoint(True)  # same task rate as the spacecraft
        eclipseObject.addSpacecraftToModel(scObject.scStateOutMsg)
        eclipseObject.addPlanetToModel(spiceObject.planetStateOutMsgs[bodyNames.index("earth")])
        eclipseObject.sunInMsg.subscribeTo(spiceObject.planetStateOutMsgs[bodyNames.index("sun")])
        scSim.AddModelToTask("dynamicsTask", eclipseObject, 90)

        if isBox:
            box = spec["box"]
            srp = facetSRPDynamicEffector.FacetSRPDynamicEffector()
            srp.ModelTag = "facetSrp"
            srp.setNumFacets(6)
            srp.setNumArticulatedFacets(0)
            for area, normal in boxFacets(spec):
                srp.addFacet(area, np.eye(3), normal, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                             box["diffuse_reflection"], box["specular_reflection"])
        else:
            srp = radiationPressure.RadiationPressure()  # default model is the cannonball model
            srp.area = sc["srp_area_m2"]  # [m^2]
            srp.coefficientReflection = sc["srp_cr"]  # [-]
        sunMsg = spiceObject.planetStateOutMsgs[bodyNames.index("sun")]
        if isBox:
            srp.sunInMsg.subscribeTo(sunMsg)
        else:
            srp.sunEphmInMsg.subscribeTo(sunMsg)
        srp.sunEclipseInMsg.subscribeTo(eclipseObject.eclipseOutMsgs[0])
        scObject.addDynamicEffector(srp)
        scSim.AddModelToTask("dynamicsTask", srp, 80)

    if case["drag"]:
        earthPlanetMsg = spiceObject.planetStateOutMsgs[bodyNames.index("earth")]
        atmo = exponentialAtmosphere.ExponentialAtmosphere()
        atmo.setExtrapolateScStateToStepMidpoint(True)  # same task period as the spacecraft
        atmo.ModelTag = "expAtmosphere"
        configureExponentialAtmosphere(spec, atmo)
        atmo.addSpacecraftToModel(scObject.scStateOutMsg)
        atmo.planetPosInMsg.subscribeTo(earthPlanetMsg)
        scSim.AddModelToTask("dynamicsTask", atmo, 90)

        wind = zeroWindModel.ZeroWindModel()  # atmosphere co-rotating with the planet
        wind.ModelTag = "zeroWind"
        wind.setExtrapolateScStateToStepMidpoint(True)  # same task period as the spacecraft, as for the atmosphere
        wind.planetPosInMsg.subscribeTo(earthPlanetMsg)
        wind.addSpacecraftToModel(scObject.scStateOutMsg)
        scSim.AddModelToTask("dynamicsTask", wind, 85)

        if isBox:
            drag = facetDragDynamicEffector.FacetDragDynamicEffector()
            drag.ModelTag = "facetDrag"
            for area, normal in boxFacets(spec):
                drag.addFacet(area, spec["box"]["drag_cd"], normal, [0.0, 0.0, 0.0])
        else:
            drag = dragDynamicEffector.DragDynamicEffector()
            drag.ModelTag = "drag"
            drag.coreParams.projectedArea = sc["drag_area_m2"]  # [m^2]
            drag.coreParams.dragCoeff = sc["drag_cd"]  # [-]
        drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])
        drag.windVelInMsg.subscribeTo(wind.envOutMsgs[0])
        scObject.addDynamicEffector(drag)
        scSim.AddModelToTask("dynamicsTask", drag, 80)

    recorder = scObject.scStateOutMsg.recorder(macros.sec2nano(spec["sample_period_s"]))
    scSim.AddModelToTask("dynamicsTask", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(durationSeconds or caseDuration(spec, case)))
    scSim.ExecuteSimulation()
    if spiceObject is not None:
        gravFactory.unloadSpiceKernels()

    return np.column_stack([np.array(recorder.times()) * macros.NANO2SEC,
                            np.array(recorder.r_BN_N), np.array(recorder.v_BN_N)])


def maxErrors(a, b):
    """Return the max position [m] and velocity [m/s] difference between two ephemerides."""
    return (np.linalg.norm(a[:, 1:4] - b[:, 1:4], axis=1).max(),
            np.linalg.norm(a[:, 4:7] - b[:, 4:7], axis=1).max())


def plotCase(name, bsk, gmat, orekit, variants=None):
    """Return a figure with the position differences between the tools versus time.

    Args:
        name (str): case name used as the title.
        bsk, gmat, orekit (ndarray): ephemerides with columns (t, r_xyz, v_xyz). ``gmat`` is None for the cases that GMAT
            does not model.
        variants (dict): variant of each reference tool that is not the default one, shown in the title.
    """
    def difference(a, b):
        return np.maximum(np.linalg.norm(a[:, 1:4] - b[:, 1:4], axis=1), 1e-6)  # [m]

    fig = plt.figure(figsize=(6.0, 3.6))
    tDays = bsk[:, 0] / 86400.0  # [days]
    if gmat is not None:
        plt.semilogy(tDays, difference(bsk, gmat), "-", label="Basilisk - GMAT")
    plt.semilogy(tDays, difference(bsk, orekit), "-", label="Basilisk - Orekit")
    if gmat is not None:
        plt.semilogy(tDays, difference(gmat, orekit), "--", label="GMAT - Orekit")
    plt.xlabel("time [days]")
    plt.ylabel("position difference [m]")
    plt.title(name + "".join(f" ({tool} variant: {v})" for tool, v in (variants or {}).items()))
    plt.grid(True)
    plt.legend(fontsize="small")
    plt.tight_layout()
    return fig


def run(caseNames=None, durationDays=None, kernelDir=None, figuresDir=None, showPlots=False, dataDir=None,
        variants=None):
    """Propagate the comparison cases with Basilisk and compare to GMAT and Orekit.

    Returns a dictionary keyed by case name with the maximum position [m] and velocity [m/s] differences and, under
    ``variants``, the variant of each reference that is not the default configuration.

    Args:
        caseNames (list): subset of cases to run; all cases by default.
        durationDays (float): propagation time in days, compared with the start of the reference ephemerides;
            by default the full duration of each case. A case shorter than this is not extended.
        kernelDir (Path): folder with the Earth orientation kernels.
        figuresDir (Path): folder where the position-difference figures are saved as SVG files.
        showPlots (bool): show the matplotlib plots.
        dataDir (Path): folder with the reference ephemerides; ``data`` next to this script by default.
        variants (dict): variant of the reference of each tool that is accepted, e.g. ``{"orekit": "oblate_shadow"}``.
            The references must otherwise be the default configuration.
    """
    spec = loadSpec()
    caseNames = caseNames or list(spec["cases"])
    variants = variants or {}
    results = {}
    if any(spec["cases"][name]["drag"] for name in caseNames):
        compareDensityProbe(spec, dataDir)

    with tempfile.TemporaryDirectory() as tmp:
        for name in caseNames:
            case = spec["cases"][name]
            duration = caseDuration(spec, case)  # [s]
            if durationDays is not None:
                duration = min(duration, durationDays * 86400.0)  # [s]
            # validate the references before the (long) Basilisk run
            loaded = {tool: loadReference(tool, name, spec, dataDir, variants.get(tool, DEFAULT_VARIANT))
                      for tool in caseReferences(case)}
            gravityFile = Path(tmp) / f"{name}.txt"
            if case["gravity"]["degree"] > 0:
                writeBasiliskGravity(gravityFile, spec, case["gravity"]["degree"], case["gravity"]["order"])
            bsk = propagateBasilisk(spec, case, gravityFile, kernelDir, duration)
            references = {tool: validateReference(tool, name, ref, bsk) for tool, ref in loaded.items()}
            gmat, orekit = references.get("gmat"), references["orekit"]
            results[name] = {}
            if gmat is not None:
                results[name]["bsk_vs_gmat"] = maxErrors(bsk, gmat)
            results[name]["bsk_vs_orekit"] = maxErrors(bsk, orekit)
            if gmat is not None:
                results[name]["gmat_vs_orekit"] = maxErrors(gmat, orekit)
            caseVariants = {tool: v for tool, v in variants.items() if tool in references and v != DEFAULT_VARIANT}
            results[name]["variants"] = caseVariants
            print(f"{name}: max |dr| [m] (|dv| [m/s]) "
                  + ", ".join(f"{k}={v[0]:.3e} ({v[1]:.2e})" for k, v in results[name].items() if k != "variants")
                  + "".join(f"; {tool} reference variant: {v}" for tool, v in caseVariants.items()))

            if plt is not None and (figuresDir or showPlots):
                fig = plotCase(name, bsk, gmat, orekit, caseVariants)
                if figuresDir:
                    Path(figuresDir).mkdir(parents=True, exist_ok=True)
                    suffix = "".join(f"_{tool}-{v}" for tool, v in caseVariants.items())
                    fig.savefig(Path(figuresDir) / f"accuracyComparison_{name}{suffix}.svg")
                if not showPlots:
                    plt.close(fig)

    if plt is not None and showPlots:
        plt.show()
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--cases", nargs="*", help="case names to run (default: all)")
    parser.add_argument("--duration-days", type=float,
                        help="maximum propagation time in days (default: the full duration of each case)")
    parser.add_argument("--kernel-dir", type=Path, help="folder with the high-precision Earth orientation kernels")
    parser.add_argument("--data-dir", type=Path, help="folder with the GMAT and Orekit reference ephemerides "
                        "(default: data next to this script)")
    parser.add_argument("--figures-dir", type=Path, help="folder where the SVG figures are saved")
    for tool in ("gmat", "orekit"):
        parser.add_argument(f"--{tool}-variant", default=DEFAULT_VARIANT,
                            help=f"variant of the {tool} reference to accept: an alternative configuration such as "
                            "oblate_shadow is rejected unless it is named here (default: %(default)s)")
    parser.add_argument("--show-plots", action="store_true", help="show the plots")
    args = parser.parse_args()
    run(args.cases, args.duration_days, args.kernel_dir, args.figures_dir, args.show_plots, args.data_dir,
        {"gmat": args.gmat_variant, "orekit": args.orekit_variant})
