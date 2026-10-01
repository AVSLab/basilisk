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

The cases are defined in ``cases.json``. The reference ephemerides are produced beforehand with
``generate_gmat_reference.py`` and ``generate_orekit_reference.py`` (GMAT and Orekit are needed only for that step) and
are read from ``data/`` or the folder given with ``--data-dir``. Basilisk is run with the same initial conditions, gravity field, third bodies,
solar radiation pressure and NRLMSISE-00 drag as the reference tools, and the maximum position and velocity
differences to each reference, and between the references, are printed. The methodology and results are described in
:ref:`accuracyComparison`.

Usage::

    python compare_with_basilisk.py [--cases leo_all geo_all] [--duration-days 30] [--kernel-dir DIR]
                                    [--data-dir DIR] [--figures-dir DIR] [--apparent-solar-time]

The cases with a rotating Earth use the high-precision Earth orientation kernel and the ITRF93 frame association
kernel (``earth_000101_260711_260415.bpc`` and ``earth_assoc_itrf93.tf``), which are not part of Basilisk's
support data. They are searched in ``--kernel-dir``, in ``data/spice`` next to this script, and in the Basilisk
support-data cache. The 30-day run of all cases takes several minutes.

The atmosphere altitude is computed above the same ellipsoid as in the other tools. By default the local solar time of
NRLMSISE-00 is the mean solar time, as in GMAT; ``--apparent-solar-time`` adds the equation of time, as in Orekit. The cases
with drag are also run with the other convention, which shows that the convention explains the difference to Orekit.
"""

import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np
from Basilisk.architecture import bskLogging, messaging
from Basilisk.simulation import (dragDynamicEffector, eclipse, msisAtmosphere, radiationPressure,
                                 spacecraft, svIntegrators, zeroWindModel)
from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile

try:
    from matplotlib import pyplot as plt
except ImportError:
    plt = None

DATA_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(DATA_DIR))
from comparisonCommon import loadSpec, writeBasiliskGravity

DEFAULT_KERNELS = (DataFile.EphemerisData.de430, DataFile.EphemerisData.naif0012,
                   DataFile.EphemerisData.de_403_masses, DataFile.EphemerisData.pck00010)
ITRF_KERNELS = ["earth_000101_260711_260415.bpc", "earth_assoc_itrf93.tf"]
SPICE_FRAMES = {"earth": "ITRF93", "sun": "IAU_SUN", "moon": "IAU_MOON"}
SAMPLE_TIME_TOLERANCE = 1.0e-3  # [s] allowed difference between reference and Basilisk sample times
MONTHS = ["JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"]


def loadReference(tool, caseName, dataDir=None):
    """Load a reference ephemeris as an array with columns (t, r_xyz, v_xyz) in SI units.

    Args:
        tool (str): ``"gmat"`` or ``"orekit"``.
        caseName (str): case name in ``cases.json``.
        dataDir (Path): folder with the reference ephemerides; ``data`` next to this script by default.
    """
    path = Path(dataDir or DATA_DIR / "data") / f"{tool}_{caseName}.csv"
    if not path.exists():
        raise FileNotFoundError(
            f"The {tool} reference ephemeris {path} does not exist. Generate it with "
            f"generate_{tool}_reference.py first (see Reproducing the Results in the accuracy comparison "
            f"documentation, ``accuracyComparison``), or give the folder that contains it with --data-dir.")
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


def spiceTime(spec):
    """Return the case epoch as a SPICE UTC time string."""
    e = spec["epoch_utc"]  # "YYYY-MM-DDTHH:MM:SS.sss"
    return f"{e[0:4]} {MONTHS[int(e[5:7]) - 1]} {e[8:10]} {e[11:]} (UTC)"


def propagateBasilisk(spec, case, gravityFile, kernelDir=None, apparentSolarTime=False):
    """Propagate one case with Basilisk and return an array with columns (t, r_xyz, v_xyz).

    Args:
        spec (dict): parsed ``cases.json`` content.
        case (dict): the case entry.
        gravityFile (Path): spherical-harmonics file used when the gravity degree is positive.
        kernelDir (Path): folder with the Earth orientation kernels, used by the cases with a rotating Earth.
        apparentSolarTime (bool): add the equation of time to the local solar time of NRLMSISE-00.
    """
    sc = spec["spacecraft"]
    stepSeconds = case["basilisk_step_s"]  # [s]
    thirdBodies = case["third_bodies"]
    needSpice = case["earth_rotation"] or thirdBodies or case["srp"] or case["drag"]

    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("dynamicsProcess")
    proc.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(stepSeconds)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraft"
    integrator = svIntegrators.svIntegratorRKF78(scObject)
    scObject.setIntegrator(integrator)
    scObject.hub.mHub = sc["mass_kg"]  # [kg]
    scObject.hub.r_CN_NInit = case["r0_m"]
    scObject.hub.v_CN_NInit = case["v0_m_s"]
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
            path=str(folder) + "/", time=spiceTime(spec), epochInMsg=True, spiceKernelFileNames=kernels,
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

        srp = radiationPressure.RadiationPressure()  # default model is the cannonball model
        srp.area = sc["srp_area_m2"]  # [m^2]
        srp.coefficientReflection = sc["srp_cr"]  # [-]
        srp.sunEphmInMsg.subscribeTo(spiceObject.planetStateOutMsgs[bodyNames.index("sun")])
        srp.sunEclipseInMsg.subscribeTo(eclipseObject.eclipseOutMsgs[0])
        scObject.addDynamicEffector(srp)
        scSim.AddModelToTask("dynamicsTask", srp, 80)

    if case["drag"]:
        weather = spec["space_weather"]
        earthPlanetMsg = spiceObject.planetStateOutMsgs[bodyNames.index("earth")]
        atmo = msisAtmosphere.MsisAtmosphere()
        atmo.setExtrapolateScStateToStepMidpoint(True)  # same task rate as the spacecraft
        atmo.ModelTag = "msis"
        atmo.planetRadius = spec["equatorial_radius_m"]  # [m]
        atmo.setPlanetPolarRadius(spec["equatorial_radius_m"] * (1.0 - spec["earth_flattening"]))  # [m]
        atmo.setUseApparentSolarTime(apparentSolarTime)
        atmo.addSpacecraftToModel(scObject.scStateOutMsg)
        atmo.planetPosInMsg.subscribeTo(earthPlanetMsg)
        atmo.epochInMsg.subscribeTo(gravFactory.epochMsg)
        swKeys = (["ap_24_0"] + [f"ap_3_{-3 * k}" for k in range(20)] + ["f107_1944_0", "f107_24_-24"])
        swMsgs = []
        for c, key in enumerate(swKeys):
            value = weather["f107"] if key.startswith("f107") else weather["ap"]
            swMsgs.append(messaging.SwDataMsg().write(messaging.SwDataMsgPayload(dataValue=value)))
            atmo.swDataInMsgs[c].subscribeTo(swMsgs[-1])
        scSim.AddModelToTask("dynamicsTask", atmo, 90)

        wind = zeroWindModel.ZeroWindModel()  # atmosphere co-rotating with the planet
        wind.ModelTag = "zeroWind"
        wind.planetPosInMsg.subscribeTo(earthPlanetMsg)
        wind.addSpacecraftToModel(scObject.scStateOutMsg)
        scSim.AddModelToTask("dynamicsTask", wind, 85)

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
    scSim.ConfigureStopTime(macros.sec2nano(spec["duration_s"]))
    scSim.ExecuteSimulation()
    if spiceObject is not None:
        gravFactory.unloadSpiceKernels()

    return np.column_stack([np.array(recorder.times()) * macros.NANO2SEC,
                            np.array(recorder.r_BN_N), np.array(recorder.v_BN_N)])


def maxErrors(a, b):
    """Return the max position [m] and velocity [m/s] difference between two ephemerides."""
    return (np.linalg.norm(a[:, 1:4] - b[:, 1:4], axis=1).max(),
            np.linalg.norm(a[:, 4:7] - b[:, 4:7], axis=1).max())


def plotCase(name, bsk, gmat, orekit, bskOther=None, solarTime="mean", otherSolarTime="apparent"):
    """Return a figure with the position differences between the tools versus time.

    Args:
        name (str): case name used as the title.
        bsk, gmat, orekit (ndarray): ephemerides with columns (t, r_xyz, v_xyz).
        bskOther (ndarray): Basilisk ephemeris of the same case with the other NRLMSISE-00 solar time convention. If
            given (cases with drag), the Basilisk curves are labeled with their convention and a fourth curve shows
            this ephemeris against Orekit.
        solarTime (str): solar time convention of ``bsk``, ``"mean"`` or ``"apparent"``.
        otherSolarTime (str): solar time convention of ``bskOther``.
    """
    def difference(a, b):
        return np.maximum(np.linalg.norm(a[:, 1:4] - b[:, 1:4], axis=1), 1e-6)  # [m]

    fig = plt.figure(figsize=(6.0, 3.6))
    tDays = bsk[:, 0] / 86400.0  # [days]
    bskName = "Basilisk" if bskOther is None else f"Basilisk ({solarTime} solar time)"
    plt.semilogy(tDays, difference(bsk, gmat), "-", label=f"{bskName} - GMAT")
    plt.semilogy(tDays, difference(bsk, orekit), "-", label=f"{bskName} - Orekit")
    plt.semilogy(tDays, difference(gmat, orekit), "--", label="GMAT - Orekit")
    if bskOther is not None:
        plt.semilogy(tDays, difference(bskOther, orekit), "-", color="tab:red",
                     label=f"Basilisk ({otherSolarTime} solar time) - Orekit")
    plt.xlabel("time [days]")
    plt.ylabel("position difference [m]")
    plt.title(name)
    plt.grid(True)
    plt.legend(fontsize="small")
    plt.tight_layout()
    return fig


def run(caseNames=None, durationDays=None, kernelDir=None, figuresDir=None, showPlots=False,
        apparentSolarTime=False, dataDir=None):
    """Propagate the comparison cases with Basilisk and compare to GMAT and Orekit.

    Returns a dictionary keyed by case name with the maximum position [m] and velocity [m/s] differences.

    Args:
        caseNames (list): subset of cases to run; all cases by default.
        durationDays (float): propagation time in days, compared with the start of the reference ephemerides;
            by default the full duration of ``cases.json``.
        kernelDir (Path): folder with the Earth orientation kernels.
        figuresDir (Path): folder where the position-difference figures are saved as SVG files.
        showPlots (bool): show the matplotlib plots.
        apparentSolarTime (bool): add the equation of time to the local solar time of NRLMSISE-00. The cases with drag
            are run a second time with the other convention, which is reported and drawn as a fourth curve.
        dataDir (Path): folder with the reference ephemerides; ``data`` next to this script by default.
    """
    spec = loadSpec()
    if durationDays is not None:
        spec["duration_s"] = durationDays * 86400.0
    caseNames = caseNames or list(spec["cases"])
    results = {}

    with tempfile.TemporaryDirectory() as tmp:
        for name in caseNames:
            case = spec["cases"][name]
            gravityFile = Path(tmp) / f"{name}.txt"
            if case["gravity"]["degree"] > 0:
                writeBasiliskGravity(gravityFile, spec, case["gravity"]["degree"], case["gravity"]["order"])
            bsk = propagateBasilisk(spec, case, gravityFile, kernelDir, apparentSolarTime)
            gmat = validateReference("gmat", name, loadReference("gmat", name, dataDir), bsk)
            orekit = validateReference("orekit", name, loadReference("orekit", name, dataDir), bsk)
            results[name] = {"bsk_vs_gmat": maxErrors(bsk, gmat), "bsk_vs_orekit": maxErrors(bsk, orekit),
                             "gmat_vs_orekit": maxErrors(gmat, orekit)}
            bskOther = None
            if case["drag"]:
                # the same case with the other solar time convention shows that it explains the Orekit difference
                bskOther = propagateBasilisk(spec, case, gravityFile, kernelDir, not apparentSolarTime)
                key = "bsk_mean_solar_time_vs_orekit" if apparentSolarTime else "bsk_apparent_solar_time_vs_orekit"
                results[name][key] = maxErrors(bskOther, orekit)
            print(f"{name}: max |dr| [m] (|dv| [m/s]) "
                  + ", ".join(f"{k}={v[0]:.3e} ({v[1]:.2e})" for k, v in results[name].items()))

            if plt is not None and (figuresDir or showPlots):
                fig = plotCase(name, bsk, gmat, orekit, bskOther,
                               "apparent" if apparentSolarTime else "mean",
                               "mean" if apparentSolarTime else "apparent")
                if figuresDir:
                    Path(figuresDir).mkdir(parents=True, exist_ok=True)
                    fig.savefig(Path(figuresDir) / f"accuracyComparison_{name}.svg")
                if not showPlots:
                    plt.close(fig)

    if plt is not None and showPlots:
        plt.show()
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--cases", nargs="*", help="case names to run (default: all)")
    parser.add_argument("--duration-days", type=float, help="propagation time in days (default: full duration)")
    parser.add_argument("--kernel-dir", type=Path, help="folder with the high-precision Earth orientation kernels")
    parser.add_argument("--data-dir", type=Path, help="folder with the GMAT and Orekit reference ephemerides "
                        "(default: data next to this script)")
    parser.add_argument("--figures-dir", type=Path, help="folder where the SVG figures are saved")
    parser.add_argument("--apparent-solar-time", action="store_true",
                        help="add the equation of time to the NRLMSISE-00 local solar time")
    parser.add_argument("--show-plots", action="store_true", help="show the plots")
    args = parser.parse_args()
    run(args.cases, args.duration_days, args.kernel_dir, args.figures_dir, args.show_plots,
        args.apparent_solar_time, args.data_dir)
