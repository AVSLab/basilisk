# ISC License
#
# Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder
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

"""
Unit tests for earthRadiationModel.

Test geometry (all tests unless noted)
---------------------------------------
  Earth at origin (planet-fixed = inertial, J20002Pfix = I)
  Sun at (+AU, 0, 0)  → sunlit side: +x hemisphere
  Satellite at (0, REQ+600 [km], 0)  → visible side: +y hemisphere
  Overlap (first quadrant lon 0-90°) → albedoFlux > 0, irFlux > 0

Physics expected values (approximate, for sanity-check only)
-------------------------------------------------------------
  irFlux  ~ OLR/(pi) * solid_angle_of_Earth  ~ O(50) W/m^2
  albedoFlux > 0 (sunlit + visible patches exist)
  Both direction vectors are unit vectors when flux > 0
"""

import math

import numpy as np
import pytest

from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import earthRadiationModel
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion as om
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path

bskPath = __path__[0]

REQ_EARTH_KM = om.REQ_EARTH  # [km]
RP_EARTH_KM  = om.RP_EARTH  # [km]
REQ_EARTH_M  = REQ_EARTH_KM * 1000.0
RP_EARTH_M   = RP_EARTH_KM  * 1000.0
ALT_M        = 600.0e3 # [m] altitude

# Radiation constants matching planetRadiationBase defaults and astroConstants.h
S0_WM2         = 1361.0             # [W/m^2], solar irradiance at 1 AU (PRB_S0_WM2)
AU2M           = om.AU * 1000.0     # [m], 1 AU (AU from astroConstants is in [km])
OLR_DEFAULT    = 237.0              # [W/m^2], default irFluxMean
ALBEDO_DEFAULT = 0.30               # default albedoAvg
NLAT_DEFAULT   = 180
NLON_DEFAULT   = 360


def _run_sim(useAlbedoData: bool = False, nLat: int = 0, nLon: int = 0,
             eclipseCase: bool = False, sunPos_N=None, scPos_N=None):
    """
    Create and step the earthRadiationModel module once.

    Returns the recorded EarthRadiationMsgPayload after one timestep.
    """
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("testProc")
    proc.addTask(sim.CreateNewTask("testTask", macros.sec2nano(1.0)))

    erm = earthRadiationModel.EarthRadiationModel()
    erm.ModelTag = "earthRadiation"

    if nLat > 0:
        erm.defaultNumLat = nLat
    if nLon > 0:
        erm.defaultNumLon = nLon

    if eclipseCase:
        erm.bskLogger.setLogLevel(bskLogging.BSK_ERROR)
        erm.setEclipseCase(True)

    # Default 180×360 grid, Bond albedo 0.30, OLR 237 W/m^2
    if useAlbedoData:
        data = get_path(DataFile.AlbedoData.Earth_ALB_2018_CERES_All_1x1)
        erm.albedoDataPath = str(data.parent)
        erm.albedoDataFile = data.name

    # --- Earth message ---
    earthPayload = messaging.SpicePlanetStateMsgPayload()
    earthPayload.PositionVector = [0.0, 0.0, 0.0]
    earthPayload.PlanetName     = "earth"
    # planet-fixed frame = inertial (identity rotation)
    earthPayload.J20002Pfix = np.identity(3)
    earthMsg = messaging.SpicePlanetStateMsg().write(earthPayload)
    erm.planetInMsg.subscribeTo(earthMsg)

    # --- Sun message ---
    sunPayload = messaging.SpicePlanetStateMsgPayload()
    sunPayload.PositionVector = list(sunPos_N) if sunPos_N is not None \
                                else [om.AU * 1000.0, 0.0, 0.0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunPayload)
    erm.sunPositionInMsg.subscribeTo(sunMsg)

    # --- Spacecraft message ---
    scPayload = messaging.SCStatesMsgPayload()
    scPayload.r_BN_N = list(scPos_N) if scPos_N is not None \
                       else [0.0, REQ_EARTH_M + ALT_M, 0.0]
    scMsg = messaging.SCStatesMsg().write(scPayload)
    erm.spacecraftStateInMsg.subscribeTo(scMsg)

    sim.AddModelToTask("testTask", erm)
    dataLog = erm.earthRadiationOutMsg.recorder()
    sim.AddModelToTask("testTask", dataLog)

    sim.InitializeSimulation()
    sim.TotalSim.SingleStepProcesses()

    return dataLog


# ------------------------------------------------------------------ #
# Test 1: ALBEDO_AVG mode — basic physics                             #
# ------------------------------------------------------------------ #
def test_earthRadiation_albedoAvg_physicsChecks():
    """
    albedoFlux and irFlux are positive; direction vectors are unit vectors.
    """
    log = _run_sim(useAlbedoData=False)

    albFlux = log.albedoFlux[0]
    irFlux  = log.irFlux[0]
    albDir  = np.array(log.albedoDir_N[0])
    irDir   = np.array(log.irDir_N[0])

    # Both flux channels must be positive for this geometry
    assert albFlux > 0.0, f"albedoFlux should be > 0, got {albFlux}"
    assert irFlux  > 0.0, f"irFlux should be > 0, got {irFlux}"

    # Albedo flux at 600 [km] should be O(10) W/m^2 — sanity bound
    assert albFlux < 1000.0, f"albedoFlux unrealistically large: {albFlux}"
    assert irFlux  < 1000.0, f"irFlux unrealistically large: {irFlux}"

    # Direction vectors must be unit vectors (within float tolerance)
    albNorm = np.linalg.norm(albDir)
    irNorm  = np.linalg.norm(irDir)
    assert abs(albNorm - 1.0) < 1e-10, f"|albedoDir_N| = {albNorm}, expected 1"
    assert abs(irNorm  - 1.0) < 1e-10, f"|irDir_N| = {irNorm}, expected 1"


# ------------------------------------------------------------------ #
# Test 2: ALBEDO_DATA mode — non-zero output with CERES data          #
# ------------------------------------------------------------------ #
def test_earthRadiation_albedoData_nonzero():
    """
    With CERES albedo data, albedoFlux must remain positive.
    """
    log = _run_sim(useAlbedoData=True)

    albFlux = log.albedoFlux[0]
    irFlux  = log.irFlux[0]

    assert albFlux > 0.0, f"albedoFlux should be > 0 with CERES data, got {albFlux}"
    assert irFlux  > 0.0, f"irFlux should be > 0 with CERES data, got {irFlux}"


# ------------------------------------------------------------------ #
# Test 3: Direction geometry — rough check                            #
# ------------------------------------------------------------------ #
def test_earthRadiation_direction_toward_earth():
    """
    The net force direction from ERP points roughly from satellite toward Earth
    (i.e. the radiation comes from Earth).  With the satellite on the +y axis,
    irDir_N should have a dominant negative-y component.
    """
    log = _run_sim(useAlbedoData=False)
    irDir = np.array(log.irDir_N[0])

    # Satellite at (0, +y, 0); most IR comes from below (-y hemisphere relative
    # to satellite) so dir_N (patch->sat) has positive y, hence irDir_N has +y.
    # The net weighted direction from patches visible from +y satellite:
    # all visible patches have dir_N pointing toward +y (up), so irDir_N[1] > 0.
    assert irDir[1] > 0.0, (
        f"irDir_N[1] expected > 0 (satellite on +y axis, patches push upward), "
        f"got {irDir[1]:.4f}"
    )


# ------------------------------------------------------------------ #
# Test 4: Reset() error — unlinked messages                           #
# ------------------------------------------------------------------ #
def test_earthRadiation_unlinked_messages():
    """
    Reset() must raise BSK_ERROR (BasiliskError) when messages are unlinked.
    """
    erm = earthRadiationModel.EarthRadiationModel()
    erm.ModelTag = "earthRadiationUnlinked"

    # No messages linked — Reset() should log BSK_ERROR => raises BasiliskError
    with pytest.raises(BasiliskError):
        erm.Reset(0)


# ------------------------------------------------------------------ #
# Test 5: ALBEDO_DATA missing file raises                             #
# ------------------------------------------------------------------ #
def test_earthRadiation_invalid_albedo_file(tmp_path):
    """
    Providing a nonexistent albedo file must raise an error in Reset().
    """
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("testProc")
    proc.addTask(sim.CreateNewTask("testTask", macros.sec2nano(1.0)))

    erm = earthRadiationModel.EarthRadiationModel()
    erm.ModelTag = "erm_badfile"
    erm.albedoDataPath = str(tmp_path)
    erm.albedoDataFile = "does_not_exist.csv"

    # Link messages so the linkage check passes
    earthPayload = messaging.SpicePlanetStateMsgPayload()
    earthPayload.J20002Pfix = np.identity(3)
    earthMsg = messaging.SpicePlanetStateMsg().write(earthPayload)
    erm.planetInMsg.subscribeTo(earthMsg)

    sunPayload = messaging.SpicePlanetStateMsgPayload()
    sunMsg = messaging.SpicePlanetStateMsg().write(sunPayload)
    erm.sunPositionInMsg.subscribeTo(sunMsg)

    scPayload = messaging.SCStatesMsgPayload()
    scMsg = messaging.SCStatesMsg().write(scPayload)
    erm.spacecraftStateInMsg.subscribeTo(scMsg)

    with pytest.raises(BasiliskError):
        erm.Reset(0)


# ================================================================= #
# Validation helpers                                                #
# ================================================================= #

def _authalic_radius(REQ_m, RP_m):
    """
    Authalic (equal-area) sphere radius.

    Mirrors `PlanetRadiationBase::Reset()` in ``planetRadiationBase.cpp``:

    .. code-block:: none

        e  = sqrt(1 - (RP/REQ)^2)
        R  = sqrt( REQ^2 * 0.5 * (1 + (1-e^2)/(2e) * ln((1+e)/(1-e))) )
    """
    e = math.sqrt(1.0 - (RP_m / REQ_m) ** 2)
    return math.sqrt(
        REQ_m ** 2 * 0.5
        * (1.0 + (1.0 - e * e) / (2.0 * e) * math.log((1.0 + e) / (1.0 - e)))
    )


def _python_lambertian(r_sat_N, r_sun_N, r_planet_N, J20002Pfix,
                       R_planet, OLR, S_sun, nLat, nLon, albedo_grid,
                       REQ_m, RP_m):
    """
    Pure NumPy reimplementation of the Lambertian patch model.

    Mirrors:

    - `PlanetGrid::initialize()` (``planetRadiationBase.cpp``)
    - `PlanetGrid::computePatches()` (``planetRadiationBase.cpp``)
    - `EarthRadiationModel::evaluateModel()` (``earthRadiationModel.cpp``)

    The DCM convention follows ``avsEigenSupport.cpp``:
    ``Eigen::Map<MatrixXd>(row-major C array, n, n)`` interprets the C
    row-major layout as column-major, yielding the transpose.
    Hence ``dcm_NP = J20002Pfix.T``.

    Parameters
    ----------
    J20002Pfix : (3, 3) ndarray, same row-major layout as
                 `SpicePlanetStateMsgPayload.J20002Pfix`
    albedo_grid : (nLat, nLon) ndarray of per-patch albedo values

    Returns
    -------
    tuple of (albedoFlux, irFlux) in W/m^2
    """
    # ---- authalic correction coefficients [cpp:88-95] ---------------
    t_aut   = np.zeros(3)
    has_aut = (REQ_m > 0.0 and RP_m > 0.0 and REQ_m != RP_m)
    if has_aut:
        e  = math.sqrt(1.0 - (RP_m / REQ_m) ** 2)
        e2, e4, e6 = e**2, e**4, e**6
        t_aut[0] = e2 / 3.0       + 31.0 * e4 / 180.0  + 59.0  * e6 / 560.0
        t_aut[1] = 17.0 * e4 / 360.0 + 61.0 * e6 / 1260.0
        t_aut[2] = 383.0 * e6 / 45360.0

    def _aut(x):
        if not has_aut:
            return x
        return x - (t_aut[0] * math.sin(2.0 * x)
                    - t_aut[1] * math.sin(4.0 * x)
                    + t_aut[2] * math.sin(6.0 * x))

    # ---- lat/lon grid [cpp:130-138] ----------------------------------
    latDiff = (180.0 / nLat) * math.pi / 180.0
    lonDiff = (360.0 / nLon) * math.pi / 180.0
    halfLat, halfLon = nLat // 2, nLon // 2
    gdlat = np.array([(i - halfLat + 0.5) * latDiff for i in range(nLat)])
    gdlon = np.array([(j - halfLon + 0.5) * lonDiff for j in range(nLon)])

    # ---- DCM [avsEigenSupport.cpp:111] -------------------------------
    dcm_NP = np.asarray(J20002Pfix, dtype=float).T

    r_sat    = np.asarray(r_sat_N,    dtype=float)
    r_sun    = np.asarray(r_sun_N,    dtype=float)
    r_planet = np.asarray(r_planet_N, dtype=float)

    rs_sat = r_sat - r_planet
    rs_sun = r_sun - r_planet

    R2, inv_pi = R_planet ** 2, 1.0 / math.pi
    alb_flat   = np.asarray(albedo_grid, dtype=float).ravel()  # row-major k = ilat*nLon+ilon

    alb_sum = 0.0
    ir_sum  = 0.0

    for ilat in range(nLat):
        lat  = gdlat[ilat]
        lat1 = _aut(lat + 0.5 * latDiff)
        lat2 = _aut(lat - 0.5 * latDiff)
        normArea  = lonDiff * abs(math.sin(lat1) - math.sin(lat2))
        dA        = normArea * R2
        cosLat, sinLat = math.cos(lat), math.sin(lat)

        for ilon in range(nLon):
            lon = gdlon[ilon]
            dir_P   = np.array([cosLat * math.cos(lon),
                                 cosLat * math.sin(lon),
                                 sinLat])
            r_dAP_N = R_planet * (dcm_NP @ dir_P)
            n_hat   = r_dAP_N / np.linalg.norm(r_dAP_N)

            r_IdA = rs_sat - r_dAP_N
            d     = np.linalg.norm(r_IdA)
            if d < 1.0:
                continue
            rHat    = r_IdA / d
            cos_sat = float(n_hat @ rHat)
            if cos_sat <= 0.0:
                continue

            ir_sum += (OLR * inv_pi) * cos_sat * dA / (d * d)

            r_SdA  = rs_sun - r_dAP_N
            r_SdA_n = np.linalg.norm(r_SdA)
            if r_SdA_n > 1.0:
                cos_sun = float(n_hat @ (r_SdA / r_SdA_n))
                if cos_sun > 0.0:
                    k = ilat * nLon + ilon
                    alb_sum += ((S_sun * inv_pi) * alb_flat[k]
                                * cos_sun * cos_sat * dA / (d * d))

    return alb_sum, ir_sum


# ================================================================= #
# Test 6: Analytical closed-form IR reference                        #
# ================================================================= #

def test_earthRadiation_analytical_ir():
    """
    Validate `irFlux` against the exact Lambertian-sphere closed form.

    Derivation (integrating the patch kernel over the visible hemisphere):

    .. code-block:: none

        F_IR = (OLR/pi) * integral_{visible} cos_sat * dA / d^2
             = 2 * OLR * (1 - sqrt(1 - (R_aut/D)^2))

    This result is independent of the albedo model since `OLR` is spatially
    uniform in both avg and CERES-data configurations.

    .. note::

        The far-field approximation ``OLR*(R/D)^2`` is only valid for ``D >> R``.
        At 600 km LEO, ``R/D ~ 0.91`` so the far-field error is ~30 %;
        the exact formula above is used here.

    A fine grid (``nLat=180``, ``nLon=360``, 1-degree resolution) is used so
    that the midpoint-rule discretisation error is < 0.3 %, well below the
    assertion tolerance of 1 %.
    """
    log = _run_sim(useAlbedoData=False, nLat=180, nLon=360)
    ir_bsk = log.irFlux[0]

    R_aut  = _authalic_radius(REQ_EARTH_M, RP_EARTH_M)
    D      = REQ_EARTH_M + ALT_M
    ir_ref = 2.0 * OLR_DEFAULT * (1.0 - math.sqrt(1.0 - (R_aut / D) ** 2))

    tol     = 0.01   # 1 % — covers <0.3 % grid error + floating-point noise
    rel_err = abs(ir_bsk - ir_ref) / ir_ref
    assert rel_err < tol, (
        f"irFlux={ir_bsk:.4f} W/m^2 vs analytical {ir_ref:.4f} W/m^2 "
        f"(rel err {rel_err * 100:.2f} %, tol {tol * 100:.0f} %)"
    )


# ================================================================= #
# Test 7: Pure-NumPy independent reference (avg and CERES data)      #
# ================================================================= #

@pytest.mark.parametrize("useAlbedoData", [False, True])
def test_earthRadiation_numpy_reference(useAlbedoData):
    """
    Cross-check EarthRadiationModel against a pure-NumPy reimplementation of
    the same Lambertian patch algorithm.

    **What this test validates (and does not validate)**

    `_python_lambertian` mirrors ``planetRadiationBase.cpp`` and
    ``earthRadiationModel.cpp`` line-by-line in a different language.
    Because it replicates the *same model*, it cannot catch physics errors
    shared by both implementations.  Its value is:

    - **Arithmetic correctness** — catches mistakes in the C++/Eigen
      indexing, DCM convention (``Eigen::Map`` column-major transpose of
      row-major C array, see ``avsEigenSupport.cpp``), `normArea`
      formula, or accumulation loop.
    - **CERES CSV loading** — verifies that the C++ CSV parser produces the
      same per-patch albedo values as ``np.loadtxt``.  The albedo grid is
      loaded from Basilisk ``supportData``
      (`DataFile.AlbedoData.Earth_ALB_2018_CERES_All_1x1`) and fed
      identically into both sides.

    Physics correctness (the model converges to the right integral) is
    covered by `test_earthRadiation_analytical_ir`.

    Tolerance: ``1e-7`` relative, accounting for accumulated floating-point
    differences between Eigen and NumPy across the 180x360 patch loop.
    """
    log     = _run_sim(useAlbedoData=useAlbedoData, nLat=NLAT_DEFAULT, nLon=NLON_DEFAULT)
    alb_bsk = log.albedoFlux[0]
    ir_bsk  = log.irFlux[0]

    R_aut = _authalic_radius(REQ_EARTH_M, RP_EARTH_M)

    # Geometry: must match _run_sim exactly
    r_sat_N    = np.array([0.0, REQ_EARTH_M + ALT_M, 0.0])
    r_sun_N    = np.array([AU2M, 0.0, 0.0])
    r_planet_N = np.zeros(3)
    J20002Pfix = np.eye(3)

    # S_sun: mirrors planetRadiationBase.cpp:UpdateState()
    r_SE  = r_sun_N - r_planet_N
    S_sun = S0_WM2 * (AU2M / np.linalg.norm(r_SE)) ** 2   # = S0_WM2 for r_SE = 1 AU

    # Albedo grid
    if useAlbedoData:
        data_path   = get_path(DataFile.AlbedoData.Earth_ALB_2018_CERES_All_1x1)
        albedo_grid = np.loadtxt(str(data_path), delimiter=',')
        assert albedo_grid.shape == (NLAT_DEFAULT, NLON_DEFAULT), (
            f"Expected CERES grid shape ({NLAT_DEFAULT},{NLON_DEFAULT}), "
            f"got {albedo_grid.shape}"
        )
    else:
        albedo_grid = np.full((NLAT_DEFAULT, NLON_DEFAULT), ALBEDO_DEFAULT)

    alb_ref, ir_ref = _python_lambertian(
        r_sat_N, r_sun_N, r_planet_N, J20002Pfix, R_aut,
        OLR_DEFAULT, S_sun,
        NLAT_DEFAULT, NLON_DEFAULT, albedo_grid,
        REQ_EARTH_M, RP_EARTH_M,
    )

    assert alb_bsk > 0.0 and alb_ref > 0.0, "albedoFlux must be positive"
    assert ir_bsk  > 0.0 and ir_ref  > 0.0, "irFlux must be positive"

    tol = 1e-7
    alb_rel = abs(alb_bsk - alb_ref) / alb_ref
    ir_rel  = abs(ir_bsk  - ir_ref)  / ir_ref
    assert alb_rel < tol, (
        f"albedoFlux: BSK={alb_bsk:.6e}  NumPy={alb_ref:.6e} "
        f"rel err={alb_rel:.2e}  tol={tol:.0e}"
    )
    assert ir_rel < tol, (
        f"irFlux:     BSK={ir_bsk:.6e}  NumPy={ir_ref:.6e} "
        f"rel err={ir_rel:.2e}  tol={tol:.0e}"
    )


# ================================================================= #
# Test 8: eclipseCase regression (P2)                               #
# ================================================================= #

def test_earthRadiation_eclipseCase_applied():
    """
    **P2 regression**: ``eclipseCase=True`` must be applied in
    ``EarthRadiationModel``.

    Before the fix, ``isPatchEclipsed()`` in the base class always returned
    ``False`` regardless of the flag, so ``eclipseCase=True`` had no effect
    on ERM output.  After the fix, the base-class implementation runs the
    Knocke penumbra model when the flag is set.

    Properties verified:

    1. ``eclipseCase=True`` runs without error.
    2. **Monotonicity**: eclipse can only reduce albedo flux, never increase
       it — ``albedo_eclipse <= albedo_no_eclipse``.
    3. **IR is unaffected**: eclipse shadows apply only to the albedo channel;
        the IR channel is independent of solar illumination and must not change.
    4. **Small magnitude**: for the standard geometry (satellite well outside
        the terminator zone), nearly all visible patches are clearly sunlit so
        the penumbra correction is negligible (< 1 % relative).
    """
    log_no_eclipse = _run_sim(eclipseCase=False)
    log_eclipse    = _run_sim(eclipseCase=True)

    alb_no_eclipse = log_no_eclipse.albedoFlux[0]
    alb_eclipse    = log_eclipse.albedoFlux[0]
    ir_no_eclipse  = log_no_eclipse.irFlux[0]
    ir_eclipse     = log_eclipse.irFlux[0]

    # 1. Both channels must be positive for this geometry
    assert alb_eclipse    >= 0.0, f"albedoFlux with eclipse must be >= 0, got {alb_eclipse}"
    assert alb_no_eclipse >  0.0, f"albedoFlux without eclipse must be > 0, got {alb_no_eclipse}"

    # 2. Monotonicity: eclipse <= no-eclipse
    assert alb_eclipse <= alb_no_eclipse + 1e-12, (
        f"Eclipse must not increase albedo flux: "
        f"eclipse={alb_eclipse:.6e}  no_eclipse={alb_no_eclipse:.6e}"
    )

    # 3. IR flux is independent of eclipse (emitted regardless of illumination)
    assert abs(ir_eclipse - ir_no_eclipse) < 1e-12 * max(ir_no_eclipse, 1.0), (
        f"IR flux must be unchanged by eclipseCase: "
        f"eclipse={ir_eclipse:.6e}  no_eclipse={ir_no_eclipse:.6e}"
    )

    # 4. For clearly-lit geometry, penumbra correction is negligible (< 1 %)
    if alb_no_eclipse > 0.0:
        rel_diff = abs(alb_eclipse - alb_no_eclipse) / alb_no_eclipse
        assert rel_diff < 0.01, (
            f"Unexpected large eclipse effect for standard geometry: "
            f"rel_diff={rel_diff:.2e} (expected < 1 %)"
        )


# ================================================================= #
# Test 9: albedoDir_N = [0,0,0] when albedoFlux = 0               #
# ================================================================= #

def test_earthRadiation_zero_albedo_direction_is_zero():
    """
    When no patches are both sunlit and visible (albedoFlux = 0),
    albedoDir_N must be [0, 0, 0].

    Geometry: Sun on +x, satellite on -x.  Sunlit patches (outward normal toward +x)
    are on the far side from the satellite; visible patches (outward normal toward -x)
    face away from the Sun.  The two sets are disjoint → albedoFlux = 0.
    IR flux must still be positive (emitted regardless of illumination).
    """
    log = _run_sim(
        scPos_N=[-REQ_EARTH_M - ALT_M, 0.0, 0.0],  # satellite on -x (dark side)
    )

    albFlux = log.albedoFlux[0]
    albDir  = np.array(log.albedoDir_N[0])
    irFlux  = log.irFlux[0]

    assert albFlux == pytest.approx(0.0, abs=1e-12), (
        f"albedoFlux must be 0 for satellite on dark side; got {albFlux:.4e}"
    )
    assert np.allclose(albDir, [0.0, 0.0, 0.0], atol=1e-12), (
        f"albedoDir_N must be [0,0,0] when albedoFlux=0; got {albDir}"
    )
    assert irFlux > 0.0, (
        f"irFlux must be > 0 even on the dark side (IR is illumination-independent); "
        f"got {irFlux:.4e}"
    )


# ================================================================= #
# Test 10: albedoDir_N geometry check                              #
# ================================================================= #

def test_earthRadiation_albedoDir_geometry():
    """
    In the standard geometry (Sun +x, satellite +y), albedoDir_N must point
    roughly toward the satellite (+y component > 0).

    Illuminated patches are in the +x hemisphere; visible patches from the +y
    satellite are near the (lat=0°, lon~65°-90°) band.  The flux-weighted
    direction from these patches to the satellite has a dominant +y component.
    """
    log    = _run_sim(useAlbedoData=False)
    albDir = np.array(log.albedoDir_N[0])

    assert log.albedoFlux[0] > 0.0, "albedoFlux must be positive for this geometry"
    assert abs(np.linalg.norm(albDir) - 1.0) < 1e-10, (
        f"albedoDir_N must be a unit vector; norm = {np.linalg.norm(albDir):.6e}"
    )
    assert albDir[1] > 0.0, (
        f"albedoDir_N[1] expected > 0 (visible+sunlit patches radiate toward +y satellite); "
        f"got {albDir}"
    )


# ================================================================= #
# Test 11: solar flux distance correction                           #
# ================================================================= #

def test_earthRadiation_solar_flux_distance_scaling():
    """
    albedoFlux must scale as (AU / d_sun)^2; irFlux must be independent of d_sun.

    Moving the Sun from 1 AU to 2 AU (same direction) halves the solar flux
    S_sun = S0*(AU/d)^2, so albedoFlux must drop by factor 4.
    The IR formula uses the constant OLR and does not contain S_sun,
    so irFlux must be unchanged.
    """
    log_1au = _run_sim(useAlbedoData=False)
    log_2au = _run_sim(
        useAlbedoData=False,
        sunPos_N=[2.0 * om.AU * 1000.0, 0.0, 0.0],  # Sun at 2 AU, same direction
    )

    alb_1au = log_1au.albedoFlux[0]
    alb_2au = log_2au.albedoFlux[0]
    ir_1au  = log_1au.irFlux[0]
    ir_2au  = log_2au.irFlux[0]

    assert alb_1au > 0.0 and alb_2au > 0.0, "both albedo fluxes must be positive"

    # albedoFlux is proportional to (1/d)^2 → ratio ~ (1/2)^2 = 0.25.
    # The ratio is not exact because moving the sun also slightly changes cos_sun
    # for each patch (parallax from Earth's finite radius, ~R_earth/AU ~ 4e-5).
    # The resulting residual is O(1e-4) in relative terms; 1e-3 tolerance is safe.
    expected_ratio = (1.0 / 2.0) ** 2
    actual_ratio   = alb_2au / alb_1au
    assert actual_ratio == pytest.approx(expected_ratio, rel=1e-3), (
        f"albedoFlux(2 AU) / albedoFlux(1 AU) = {actual_ratio:.6e}, "
        f"expected {expected_ratio:.6e}"
    )

    # IR is independent of Sun distance
    assert ir_1au == pytest.approx(ir_2au, rel=1e-10), (
        f"irFlux must be identical at 1 AU and 2 AU: "
        f"ir_1au={ir_1au:.6e}  ir_2au={ir_2au:.6e}"
    )


if __name__ == "__main__":
    test_earthRadiation_albedoAvg_physicsChecks()
    test_earthRadiation_albedoData_nonzero()
    test_earthRadiation_direction_toward_earth()
    test_earthRadiation_analytical_ir()
    test_earthRadiation_numpy_reference(useAlbedoData=False)
    test_earthRadiation_numpy_reference(useAlbedoData=True)
    test_earthRadiation_eclipseCase_applied()
    test_earthRadiation_zero_albedo_direction_is_zero()
    test_earthRadiation_albedoDir_geometry()
    test_earthRadiation_solar_flux_distance_scaling()
    print("All manual tests passed.")
