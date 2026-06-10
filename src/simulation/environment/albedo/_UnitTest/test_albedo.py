# ISC License
#
# Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

#
#   Unit Test Script
#   Module Name:   Planet's Albedo
#   Author:        Demet Cilden-Guler
#   Creation Date: May 28, 2020
#

import os

import numpy as np
import pytest
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import albedo
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion as om
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import bskLogging
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path

bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))


# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
@pytest.mark.parametrize("planetCase", ["earth", "mars"])
@pytest.mark.parametrize(
    "modelType", ["ALBEDO_AVG_IMPLICIT", "ALBEDO_AVG_EXPLICIT", "ALBEDO_DATA"]
)
@pytest.mark.parametrize("useEclipse", [True, False])
def test_unitAlbedo(planetCase, modelType, useEclipse):
    """
    **Validation Test Description**

    This section describes the specific unit tests conducted on this module.
    The test contains 4 tests and is located at ``test_albedo.py``.
    The success criteria is to match the outputs with the generated truth.

    Args:

        planetCase (string): Defines which planet to use.  Options include "earth" and "mars".
        modelType (string):  Defines which albedo model to use. Options include "ALBEDO_AVG_EXPLICIT", "ALBEDO_AVG_IMPLICIT" and "ALBEDO_DATA".
        useEclipse (bool):  Defines if the eclipse is considered for this parameterized unit test.

    **Description of Variables Being Tested**

    In this file, we are checking the values of the variable:

    ``albedoAtInstrument``

    which are pulled from the log data to see if they match with the expected truth values.

    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitAlbedo(
        planetCase, modelType, useEclipse
    )
    assert testResults < 1, testMessage


def unitAlbedo(planetCase, modelType, useEclipse):
    __tracebackhide__ = True
    testFailCount = 0
    testMessages = []
    testTaskName = "unitTestTask"
    testProcessName = "unitTestProcess"
    testTaskRate = macros.sec2nano(1.0)

    # Create a simulation container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))

    # create planet input message
    planetInMsg = messaging.SpicePlanetStateMsg()

    # Albedo A1
    albModule = albedo.Albedo()
    albModule.ModelTag = "Albedo_0"
    if modelType == "ALBEDO_DATA":
        dataFile = (
            DataFile.AlbedoData.Earth_ALB_2018_CERES_All_10x10
            if planetCase == "earth"
            else DataFile.AlbedoData.Mars_ALB_TES_10x10
        )
        data = get_path(dataFile)
        fileName = data.name
        dataPath = str(data.parent)
        albModule.addPlanetandAlbedoDataModel(planetInMsg, dataPath, fileName)
    else:
        ALB_avg = 0.5
        numLat = 200
        numLon = 400
        if modelType == "ALBEDO_AVG_EXPLICIT":
            albModule.addPlanetandAlbedoAverageModel(
                planetInMsg, ALB_avg, numLat, numLon
            )
        else:
            albModule.addPlanetandAlbedoAverageModel(planetInMsg)

    if useEclipse:
        albModule.bskLogger.setLogLevel(bskLogging.BSK_ERROR)
        albModule.setEclipseCase(True)
    # Create dummy sun message
    sunPositionMsg = messaging.SpicePlanetStateMsgPayload()

    # Create dummy planet message
    planetPositionMsg = messaging.SpicePlanetStateMsgPayload()
    planetPositionMsg.PositionVector = [0.0, 0.0, 0.0]

    gravFactory = simIncludeGravBody.gravBodyFactory()
    if planetCase == "earth":
        planet = gravFactory.createEarth()
        sunPositionMsg.PositionVector = [-om.AU * 1000.0, 0.0, 0.0]
    elif planetCase == "mars":
        planet = gravFactory.createMars()
        sunPositionMsg.PositionVector = [-1.5 * om.AU * 1000.0, 0.0, 0.0]
    planetPositionMsg.PlanetName = planetCase
    planetPositionMsg.J20002Pfix = np.identity(3)

    req = planet.radEquator
    sunMessage = "sun_message"
    # Create dummy spacecraft message
    scStateMsg = messaging.SCStatesMsgPayload()
    rSC = req + 6000 * 1000  # meters
    alpha = 71.0 * macros.D2R
    scStateMsg.r_BN_N = np.dot(rSC, [np.cos(alpha), np.sin(alpha), 0.0])
    scStateMsg.sigma_BN = [0.0, 0.0, 0.0]

    # Albedo instrument configuration
    config1 = albedo.instConfig_t()
    config1.fov = 80.0 * macros.D2R
    config1.nHat_B = np.array([-np.cos(alpha), -np.sin(alpha), 0.0])
    config1.r_IB_B = np.array([0.0, 0.0, 0.0])
    albModule.addInstrumentConfig(config1)

    sunInMsg = messaging.SpicePlanetStateMsg().write(sunPositionMsg)
    albModule.sunPositionInMsg.subscribeTo(sunInMsg)

    planetInMsg.write(planetPositionMsg)

    scInMsg = messaging.SCStatesMsg().write(scStateMsg)
    albModule.spacecraftStateInMsg.subscribeTo(scInMsg)

    unitTestSim.AddModelToTask(testTaskName, albModule)

    # setup logging
    dataLog = albModule.albOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(testTaskName, dataLog)

    # Initialize and run simulation one step at a time
    unitTestSim.InitializeSimulation()
    # Execute the simulation for one time step
    unitTestSim.TotalSim.SingleStepProcesses()
    # This pulls the actual data log from the simulation run.
    dataAlb0 = dataLog.albedoAtInstrument
    errTol = 1e-12
    if planetCase == "earth":
        if modelType == "ALBEDO_DATA":
            if useEclipse:
                truthAlb = 0.0022055492477917
            else:
                truthAlb = 0.0022055492477917
        else:
            if modelType == "ALBEDO_AVG_EXPLICIT":
                if useEclipse:
                    truthAlb = 0.0041742091531996
                else:
                    truthAlb = 0.004174209177079
            else:
                if useEclipse:
                    truthAlb = 0.002421222716229847
                else:
                    truthAlb = 0.002421222716229847
    else:
        if modelType == "ALBEDO_DATA":
            if useEclipse:
                truthAlb = 0.0014001432717662
            else:
                truthAlb = 0.0014001432717662
        else:
            if modelType == "ALBEDO_AVG_EXPLICIT":
                if useEclipse:
                    truthAlb = 0.0035681407388827
                else:
                    truthAlb = 0.0035681407390035
            else:
                if useEclipse:
                    truthAlb = 0.0011418311186365906
                else:
                    truthAlb = 0.0011418311186365906

    if not unitTestSupport.isDoubleEqual(dataAlb0[0], truthAlb, errTol):
        testFailCount += 1
    #   print out success or failure message
    if testFailCount == 0:
        print("PASSED: " + albModule.ModelTag)
    else:
        print("Failed: " + albModule.ModelTag)
    print(
        "This test uses a relative accuracy value of " + str(errTol * 100) + " percent"
    )

    return [testFailCount, "".join(testMessages)]


def test_albedo_invalid_file(tmp_path):
    """Verify that Albedo model returns gracefully when file cannot be loaded.

    Regression test for BSK-428 where model would segfault when invalid file
    was specified.

    .. note:: The model is not in a usable state if this initialization fails.
        Ideally an exception would be thrown, but the SWIG infrastructure doesn't
        appear to be setup to handle C++ exceptions, so we settle for printing a
        message and not segfaulting.
    """
    albModule = albedo.Albedo()
    # silence expected error message
    albModule.bskLogger.setLogLevel(bskLogging.BSK_SILENT)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createEarth()
    planetPositionMsg = messaging.SpicePlanetStateMsgPayload()
    planetPositionMsg.PlanetName = "earth"
    planetInMsg = messaging.SpicePlanetStateMsg().write(planetPositionMsg)

    sunPositionMsg = messaging.SpicePlanetStateMsgPayload()
    sunInMsg = messaging.SpicePlanetStateMsg().write(sunPositionMsg)
    albModule.sunPositionInMsg.subscribeTo(sunInMsg)

    scStateMsg = messaging.SCStatesMsgPayload()
    scInMsg = messaging.SCStatesMsg().write(scStateMsg)
    albModule.spacecraftStateInMsg.subscribeTo(scInMsg)

    with pytest.raises(BasiliskError):
        albModule.addPlanetandAlbedoDataModel(
            planetInMsg, str(tmp_path), "does_not_exist.file"
        )
        albModule.Reset(0)

    # the fact that we got here without segfaulting means the test
    # passed
    assert True


def _run_albedo_at_planet_offset(planet_offset_m):
    """
    Run Albedo (ALBEDO_AVG_EXPLICIT, earth, eclipseCase=True) with the whole
    scene translated by ``planet_offset_m``.  Returns ``albedoAtInstrument``.

    Translating planet + Sun + SC by the same vector leaves every relative
    distance and angle unchanged, so the eclipse-model output must be
    identical regardless of the offset.  This invariance is broken when
    ``isPatchEclipsed()`` uses the inertial Sun position instead of
    the planet-relative Sun vector ``r_SP_N = r_SN_N - r_PN_N``.
    """
    offset = np.asarray(planet_offset_m, dtype=float)

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("testProc_p1")
    proc.addTask(sim.CreateNewTask("testTask_p1", macros.sec2nano(1.0)))

    planetInMsg = messaging.SpicePlanetStateMsg()

    albModule = albedo.Albedo()
    albModule.ModelTag = "Albedo_p1"
    # ALBEDO_AVG_EXPLICIT with fine grid — same parameters as test_unitAlbedo
    albModule.addPlanetandAlbedoAverageModel(planetInMsg, 0.5, 200, 400)
    albModule.setEclipseCase(True)

    # Sun at -x relative to planet, shifted by offset
    sun_pos = offset + np.array([-om.AU * 1000.0, 0.0, 0.0])
    sunMsg = messaging.SpicePlanetStateMsgPayload()
    sunMsg.PositionVector = sun_pos.tolist()
    sunInMsg = messaging.SpicePlanetStateMsg().write(sunMsg)
    albModule.sunPositionInMsg.subscribeTo(sunInMsg)

    # Planet at offset
    planetMsg = messaging.SpicePlanetStateMsgPayload()
    planetMsg.PositionVector = offset.tolist()
    planetMsg.PlanetName = "earth"
    planetMsg.J20002Pfix = np.identity(3)
    planetInMsg.write(planetMsg)

    # SC at alpha=71° from x, altitude 6000 km above planet equatorial radius
    req_m = om.REQ_EARTH * 1e3          # km → m
    rSC_m = req_m + 6000e3
    alpha = 71.0 * macros.D2R
    sc_pos = offset + np.array([rSC_m * np.cos(alpha), rSC_m * np.sin(alpha), 0.0])
    scMsg = messaging.SCStatesMsgPayload()
    scMsg.r_BN_N = sc_pos.tolist()
    scMsg.sigma_BN = [0.0, 0.0, 0.0]
    scInMsg = messaging.SCStatesMsg().write(scMsg)
    albModule.spacecraftStateInMsg.subscribeTo(scInMsg)

    cfg = albedo.instConfig_t()
    cfg.fov = 80.0 * macros.D2R
    cfg.nHat_B = np.array([-np.cos(alpha), -np.sin(alpha), 0.0])
    cfg.r_IB_B = np.array([0.0, 0.0, 0.0])
    albModule.addInstrumentConfig(cfg)

    sim.AddModelToTask("testTask_p1", albModule)
    dataLog = albModule.albOutMsgs[0].recorder()
    sim.AddModelToTask("testTask_p1", dataLog)

    sim.InitializeSimulation()
    sim.TotalSim.SingleStepProcesses()
    return float(dataLog.albedoAtInstrument[0])


def test_albedo_eclipse_uses_planet_relative_sun_vector():
    """
    ``isPatchEclipsed()`` must use the planet-relative Sun
    vector ``r_SP_N = r_SN_N - r_PN_N``, not the inertial Sun position.

    A rigid translation of the whole scene (planet + Sun + SC) leaves every
    relative distance and angle unchanged.  Eclipse output must therefore be
    identical regardless of where the scene is placed in inertial space.

    ``r_SN_N`` was passed directly to the penumbra geometry instead of
    ``r_SP_N``.  With the planet offset by 1e12 m (~6.7 AU), the inertial Sun
    distance grows from 1 AU to ~7.7 AU.
    """
    alb_origin = _run_albedo_at_planet_offset([0.0, 0.0, 0.0])
    alb_offset = _run_albedo_at_planet_offset([1e12, 0.0, 0.0])

    assert alb_origin > 0.0, "reference albedo at origin must be positive"

    rel_err = abs(alb_origin - alb_offset) / alb_origin
    assert rel_err < 1e-10, (
        f"Eclipse output is not translation-invariant: "
        f"origin={alb_origin:.15e}, offset={alb_offset:.15e}, "
        f"rel_err={rel_err:.2e}.  "
        f"isPatchEclipsed() likely used r_SN_N instead of r_SP_N."
    )


def _albedo_run_earth(nHat_B, fov_rad, *,
                      altitudeRateLimit=-1.0,
                      illuminationFactorAtdA=1.0,
                      ALB_avg=0.5, numLat=200, numLon=400,
                      logLevel=None):
    """
    Run Albedo (ALBEDO_AVG_EXPLICIT, earth, eclipseCase=False) for one step.

    Standard geometry:
      Earth at origin, Sun at -x AU, SC at (REQ+6000 km)·[cos 71°, sin 71°, 0].

    logLevel: optional bskLogging level (e.g. BSK_ERROR) to suppress expected warnings.

    Returns the recorder attached to albOutMsgs[0].
    """
    bskLogging.setDefaultLogLevel(logLevel)
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("tp_a")
    proc.addTask(sim.CreateNewTask("tt_a", macros.sec2nano(1.0)))

    planetInMsg = messaging.SpicePlanetStateMsg()

    albModule = albedo.Albedo()
    albModule.ModelTag = "alb_helper"
    albModule.addPlanetandAlbedoAverageModel(planetInMsg, ALB_avg, numLat, numLon)
    albModule.altitudeRateLimit      = altitudeRateLimit
    albModule.illuminationFactorAtdA = illuminationFactorAtdA

    sunPayload = messaging.SpicePlanetStateMsgPayload()
    sunPayload.PositionVector = [-om.AU * 1000.0, 0.0, 0.0]
    sunInMsg = messaging.SpicePlanetStateMsg().write(sunPayload)  # keep alive
    albModule.sunPositionInMsg.subscribeTo(sunInMsg)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    req = planet.radEquator
    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.PositionVector = [0.0, 0.0, 0.0]
    planetPayload.PlanetName     = "earth"
    planetPayload.J20002Pfix     = np.identity(3)
    planetInMsg.write(planetPayload)

    rSC   = req + 6000.0 * 1000.0
    alpha = 71.0 * macros.D2R
    scPayload = messaging.SCStatesMsgPayload()
    scPayload.r_BN_N   = np.dot(rSC, [np.cos(alpha), np.sin(alpha), 0.0])
    scPayload.sigma_BN = [0.0, 0.0, 0.0]
    scInMsg = messaging.SCStatesMsg().write(scPayload)           # keep alive
    albModule.spacecraftStateInMsg.subscribeTo(scInMsg)

    cfg = albedo.instConfig_t()
    cfg.fov    = fov_rad
    cfg.nHat_B = np.asarray(nHat_B, dtype=float)
    cfg.r_IB_B = np.zeros(3)
    albModule.addInstrumentConfig(cfg)

    sim.AddModelToTask("tt_a", albModule)
    log = albModule.albOutMsgs[0].recorder()
    sim.AddModelToTask("tt_a", log)
    sim.InitializeSimulation()
    sim.TotalSim.SingleStepProcesses()
    return log


def test_albedo_altitudeRateLimit_suppresses_output():
    """
    Setting altitudeRateLimit=0 must zero albedo for any satellite above the surface.

    The altitude/radius ratio for the standard 6000 km orbit is ~0.94, which exceeds
    a limit of 0.  The module must skip that planet and write 0 for all output fields.
    """
    alpha = 71.0 * macros.D2R
    nHat  = np.array([-np.cos(alpha), -np.sin(alpha), 0.0])
    fov   = 80.0 * macros.D2R

    log_free    = _albedo_run_earth(nHat, fov, logLevel=bskLogging.BSK_WARNING)
    log_limited = _albedo_run_earth(nHat, fov, altitudeRateLimit=0.0,
                                    logLevel=bskLogging.BSK_ERROR)

    assert log_free.albedoAtInstrument[0] > 0.0, "baseline albedo must be positive"
    assert log_limited.albedoAtInstrument[0] == 0.0, (
        f"altitudeRateLimit=0 must suppress albedo; "
        f"got {log_limited.albedoAtInstrument[0]:.4e}"
    )


def test_albedo_instConfig_defaults_fallback():
    """
    Default instConfig_t (fov<0, nHat_B=zeros) must use module-level defaults.

    The test overrides albModule.nHat_B_default and albModule.fov_default to
    match a useful geometry, adds a default-constructed instConfig_t, and
    verifies that the result is identical to an explicit config with the same
    values.
    """
    alpha       = 71.0 * macros.D2R
    useful_nHat = np.array([-np.cos(alpha), -np.sin(alpha), 0.0])
    useful_fov  = 80.0 * macros.D2R

    # ---- reference: explicit instConfig_t ----
    log_explicit = _albedo_run_earth(useful_nHat, useful_fov,
                                     logLevel=bskLogging.BSK_WARNING)
    alb_explicit = log_explicit.albedoAtInstrument[0]
    assert alb_explicit > 0.0, "reference albedo must be positive"

    # ---- default instConfig_t with overridden module-level defaults ----
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("tp_d")
    proc.addTask(sim.CreateNewTask("tt_d", macros.sec2nano(1.0)))

    planetInMsg = messaging.SpicePlanetStateMsg()
    albModule = albedo.Albedo()
    albModule.ModelTag = "alb_defaults"
    albModule.addPlanetandAlbedoAverageModel(planetInMsg, 0.5, 200, 400)
    # Override module-level defaults to match the reference config
    albModule.nHat_B_default = useful_nHat
    albModule.fov_default    = useful_fov
    # Default-constructed instConfig_t: fov=-1, nHat_B=zeros → triggers fallback
    albModule.bskLogger.setLogLevel(bskLogging.BSK_ERROR)
    albModule.addInstrumentConfig(albedo.instConfig_t())

    sunPayload = messaging.SpicePlanetStateMsgPayload()
    sunPayload.PositionVector = [-om.AU * 1000.0, 0.0, 0.0]
    sunInMsg_d = messaging.SpicePlanetStateMsg().write(sunPayload)  # keep alive
    albModule.sunPositionInMsg.subscribeTo(sunInMsg_d)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    req = planet.radEquator
    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.PositionVector = [0.0, 0.0, 0.0]
    planetPayload.PlanetName     = "earth"
    planetPayload.J20002Pfix     = np.identity(3)
    planetInMsg.write(planetPayload)

    rSC   = req + 6000.0 * 1000.0
    scPayload = messaging.SCStatesMsgPayload()
    scPayload.r_BN_N   = np.dot(rSC, [np.cos(alpha), np.sin(alpha), 0.0])
    scPayload.sigma_BN = [0.0, 0.0, 0.0]
    scInMsg_d = messaging.SCStatesMsg().write(scPayload)           # keep alive
    albModule.spacecraftStateInMsg.subscribeTo(scInMsg_d)

    sim.AddModelToTask("tt_d", albModule)
    log_default = albModule.albOutMsgs[0].recorder()
    sim.AddModelToTask("tt_d", log_default)
    sim.InitializeSimulation()
    sim.TotalSim.SingleStepProcesses()

    assert log_default.albedoAtInstrument[0] == pytest.approx(alb_explicit, rel=1e-10), (
        "default instConfig_t with overridden module defaults must match explicit config"
    )


def test_albedo_nHat_B_normalization():
    """
    Non-unit nHat_B must be normalised; output must be identical to the unit version.
    """
    alpha    = 71.0 * macros.D2R
    nHat     = np.array([-np.cos(alpha), -np.sin(alpha), 0.0])
    nHat_big = nHat * 5.0   # same direction, arbitrary scale factor
    fov      = 80.0 * macros.D2R

    log_unit    = _albedo_run_earth(nHat,     fov, logLevel=bskLogging.BSK_WARNING)
    log_nonunit = _albedo_run_earth(nHat_big, fov, logLevel=bskLogging.BSK_WARNING)

    assert log_unit.albedoAtInstrument[0] > 0.0, "baseline albedo must be positive"
    assert log_unit.albedoAtInstrument[0] == pytest.approx(
        log_nonunit.albedoAtInstrument[0], rel=1e-10
    ), "scaled nHat_B must give identical output after normalisation"


def test_albedo_illuminationFactor_scaling():
    """
    User-set illuminationFactorAtdA (eclipseCase=False) must scale albedo linearly.

    All other parameters are fixed; only illuminationFactorAtdA changes from 1.0
    to 0.5.  Both albedoAtInstrument and AfluxAtInstrument must halve.
    """
    alpha = 71.0 * macros.D2R
    nHat  = np.array([-np.cos(alpha), -np.sin(alpha), 0.0])
    fov   = 80.0 * macros.D2R

    log_full = _albedo_run_earth(nHat, fov, illuminationFactorAtdA=1.0, logLevel=bskLogging.BSK_WARNING)
    log_half = _albedo_run_earth(nHat, fov, illuminationFactorAtdA=0.5, logLevel=bskLogging.BSK_WARNING)

    alb_full = log_full.albedoAtInstrument[0]
    alb_half = log_half.albedoAtInstrument[0]

    assert alb_full > 0.0, "baseline must be positive"
    assert alb_half == pytest.approx(alb_full * 0.5, rel=1e-10), (
        f"illuminationFactorAtdA=0.5 must halve albedo; "
        f"full={alb_full:.6e}  half={alb_half:.6e}"
    )


def test_albedo_AfluxAtInstrument_positive_and_consistent():
    """
    AfluxAtInstrument and AfluxAtInstrumentMax (W/m^2) must both be positive.
    AfluxAtInstrumentMax ignores the FOV cone filter, so it must be >= AfluxAtInstrument.
    """
    alpha = 71.0 * macros.D2R
    nHat  = np.array([-np.cos(alpha), -np.sin(alpha), 0.0])
    fov   = 80.0 * macros.D2R

    log = _albedo_run_earth(nHat, fov, logLevel=bskLogging.BSK_WARNING)

    aFlux    = log.AfluxAtInstrument[0]
    aFluxMax = log.AfluxAtInstrumentMax[0]

    assert aFlux    > 0.0, f"AfluxAtInstrument must be > 0; got {aFlux}"
    assert aFluxMax > 0.0, f"AfluxAtInstrumentMax must be > 0; got {aFluxMax}"
    assert aFluxMax >= aFlux - 1e-12, (
        f"AfluxAtInstrumentMax ({aFluxMax:.6e}) must be >= AfluxAtInstrument ({aFlux:.6e})"
    )


def _run_albedo_multi_planet_scene(planet_positions_m, sun_position_m, sc_position_m):
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("tp_multi")
    proc.addTask(sim.CreateNewTask("tt_multi", macros.sec2nano(1.0)))

    alb_module = albedo.Albedo()
    alb_module.ModelTag = "alb_multi"

    cfg = albedo.instConfig_t()
    cfg.fov = 120.0 * macros.D2R
    cfg.nHat_B = np.array([1.0, 0.0, 0.0])
    cfg.r_IB_B = np.array([0.0, 0.0, 0.0])
    alb_module.addInstrumentConfig(cfg)

    sun_payload = messaging.SpicePlanetStateMsgPayload()
    sun_payload.PositionVector = list(sun_position_m)
    sun_msg = messaging.SpicePlanetStateMsg()
    sun_msg.write(sun_payload)
    alb_module.sunPositionInMsg.subscribeTo(sun_msg)

    sc_payload = messaging.SCStatesMsgPayload()
    sc_payload.r_BN_N = list(sc_position_m)
    sc_payload.sigma_BN = [0.0, 0.0, 0.0]
    sc_msg = messaging.SCStatesMsg()
    sc_msg.write(sc_payload)
    alb_module.spacecraftStateInMsg.subscribeTo(sc_msg)

    # Keep planet messages alive for full sim step
    planet_msgs = []
    for planet_pos in planet_positions_m:
        pmsg = messaging.SpicePlanetStateMsg()
        planet_msgs.append(pmsg)
        alb_module.addPlanetandAlbedoAverageModel(pmsg, 0.5, 120, 240)

        pp = messaging.SpicePlanetStateMsgPayload()
        pp.PositionVector = list(planet_pos)
        pp.PlanetName = "earth"
        pp.J20002Pfix = np.identity(3)
        pmsg.write(pp)

    sim.AddModelToTask("tt_multi", alb_module)
    log = alb_module.albOutMsgs[0].recorder()
    sim.AddModelToTask("tt_multi", log)

    sim.InitializeSimulation()
    sim.TotalSim.SingleStepProcesses()

    # Return messages too so references stay alive until after run
    return log, planet_msgs, sun_msg, sc_msg


def test_albedo_two_planet_superposition_ratio_max():
    sun = np.array([-1.0e8, 0.0, 0.0])   # [m]
    p_a = np.array([-2.0e7, 0.0, 0.0])   # [m]
    p_b = np.array([ 2.0e7, 0.0, 0.0])   # [m]
    sc  = np.array([ 0.0, 8.0e7, 0.0])   # [m]

    log_a, *_  = _run_albedo_multi_planet_scene([p_a], sun, sc)
    log_b, *_  = _run_albedo_multi_planet_scene([p_b], sun, sc)
    log_ab, *_ = _run_albedo_multi_planet_scene([p_a, p_b], sun, sc)

    a = float(log_a.albedoAtInstrumentMax[0])
    b = float(log_b.albedoAtInstrumentMax[0])
    ab = float(log_ab.albedoAtInstrumentMax[0])

    assert a > 0.0 and b > 0.0 and ab > 0.0
    assert ab == pytest.approx(a + b, rel=5e-3), (
        f"Superposition failed for albedoAtInstrumentMax: AB={ab:.6e}, A+B={a+b:.6e}"
    )


def test_albedo_two_planet_order_invariance_max_fields():
    sun = np.array([-1.0e8, 0.0, 0.0])   # [m]
    p_a = np.array([-2.0e7, 0.0, 0.0])   # [m]
    p_b = np.array([ 2.0e7, 0.0, 0.0])   # [m]
    sc  = np.array([ 0.0, 8.0e7, 0.0])   # [m]

    log_ab, *_ = _run_albedo_multi_planet_scene([p_a, p_b], sun, sc)
    log_ba, *_ = _run_albedo_multi_planet_scene([p_b, p_a], sun, sc)

    assert float(log_ab.albedoAtInstrumentMax[0]) == pytest.approx(
        float(log_ba.albedoAtInstrumentMax[0]), rel=1e-10
    )
    assert float(log_ab.AfluxAtInstrumentMax[0]) == pytest.approx(
        float(log_ba.AfluxAtInstrumentMax[0]), rel=1e-10
    )


def test_ir_flux_conservation():
    """Test IR flux conservation for PlanetGrid (Lambertian sphere)."""
    albedo_mod = albedo.Albedo()
    grid = albedo.PlanetGrid()
    grid.nLat = 180
    grid.nLon = 360
    grid.REQ_m = 6371000.0  # [m]
    grid.RP_m = 6371000.0  # [m]
    grid.irFluxMean = 237.0  # [W/m^2]
    grid.albedoAvg = 0.0  # [-]
    grid.useAlbedoData = False
    grid.initialize(albedo_mod.bskLogger)

    d_center = 1e9  # [m]

    r_sat    = np.array([d_center, 0.0, 0.0])  # [m]
    r_planet = np.array([0.0, 0.0, 0.0])  # [m]
    r_sun    = np.array([1e11, 0.0, 0.0])  # [m]
    J = np.eye(3)

    patches = grid.computePatches(r_sat, r_sun, r_planet, J, grid.REQ_m, 1361.0)
    total_flux = sum(p.irFlux for p in patches)  # [W/m^2]

    # Far-field Lambertian sphere: E = M * R^2 / D^2
    expected = grid.irFluxMean * grid.REQ_m**2 / d_center**2  # [W/m^2]

    rel_error = abs(total_flux - expected) / expected

    print(f"Computed:  {total_flux:.6e} W/m^2")
    print(f"Expected:  {expected:.6e} W/m^2")
    print(f"Rel error: {rel_error:.2e}")
    print(f"Patches:   {len(patches)}")

    assert total_flux > 0, "No IR flux"
    assert len(patches) > 0, "No visible patch"
    assert rel_error < 1e-4, f"Relative error: {rel_error:.4e} >= 1e-4"


def test_planet_grid_compute_patches_requires_initialize():
    """Verify direct PlanetGrid calls raise a Python exception before initialization."""
    grid = albedo.PlanetGrid()
    grid.nLat = 1
    grid.nLon = 1

    r_sat = np.array([7000000.0, 0.0, 0.0])  # [m]
    r_sun = np.array([1.0e11, 0.0, 0.0])  # [m]
    r_planet = np.array([0.0, 0.0, 0.0])  # [m]
    J = np.eye(3)
    planet_radius = 6371000.0  # [m]
    solar_flux = 1361.0  # [W/m^2]

    with pytest.raises(BasiliskError, match="grid is not initialized"):
        grid.computePatches(r_sat, r_sun, r_planet, J, planet_radius, solar_flux)


def test_planet_grid_initialize_invalid_albedo_file_raises():
    grid = albedo.PlanetGrid()
    albedo_mod = albedo.Albedo()

    grid.useAlbedoData = True
    #grid.albedoDataPath = "/tmp"  # valid directory
    grid.albedoDataFile = "nope.csv"

    # initialize should raise a catchable BasiliskError
    with pytest.raises(BasiliskError):
        grid.initialize(albedo_mod.bskLogger)


if __name__ == "__main__":
    # unitAlbedo('earth', 'ALBEDO_AVG_EXPLICIT', True)
    unitAlbedo("mars", "ALBEDO_AVG_IMPLICIT", False)
    test_albedo_eclipse_uses_planet_relative_sun_vector()
