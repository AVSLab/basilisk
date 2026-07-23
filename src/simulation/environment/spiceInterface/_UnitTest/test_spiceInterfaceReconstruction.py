
# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

r"""
Unit tests for the opt-in SPICE query reconstruction feature of :ref:`spiceInterface`.

These cover the invariants that the feature promises:

* **Bit-for-bit default preservation** -- with nothing configured, the planet-state output is
  byte-identical to the exact per-call SPICE path.
* **Reconstruction accuracy** -- a Hermite-interpolated planet tracks the exact SPICE position
  to a small tolerance while querying SPICE far fewer times.
* **Query-count savings** -- ``getPlanetSpiceQueryCount`` reflects ~horizon/knotStep queries,
  not once per task step.
* **Single-knot orientation extrapolation is exact for a constant spinner** (Earth's IAU frame).
* **Channels are independent and toggle-off-able** -- a disabled channel emits the trivial
  default (zero position/velocity or identity orientation) without querying SPICE.
* **Derivative self-consistency** -- the emitted ``VelocityVector`` matches a finite difference
  of the reconstructed position.
"""

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk import __path__
bskPath = __path__[0]
from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.simulation import spiceInterface

DATE = "2021 MARCH 1 00:00:00.0 (UTC)"
# Names/date resolvable with the module's default DE430 kernel set (the de430.bsp SPK provides
# "mars barycenter", not "mars"). Earth carries both a position and a constant-rate IAU_EARTH
# orientation, so it exercises both reconstruction channels.
PLANETS = ["earth", "mars barycenter", "sun"]
OTHERS = ["mars barycenter", "sun"]


def _newSpice():
    # Let the module auto-configure its default kernels (naif0012/pck00010/de-403-masses/de430),
    # fetched on demand by the data fetcher -- the same path the other unit tests rely on.
    s = spiceInterface.SpiceInterface()
    s.addPlanetNames(spiceInterface.StringVector(PLANETS))
    s.UTCCalInit = DATE
    return s


def _runToStop(configureFn, dt, tf):
    """Build a sim with a fresh SpiceInterface on a regular task, apply configureFn(spice),
    run to tf, and return {planetName: payload-copy} of the final planet states + the spice object."""
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    proc.addTask(sim.CreateNewTask("task", macros.sec2nano(dt)))
    spice = _newSpice()
    if configureFn is not None:
        configureFn(spice)
    sim.AddModelToTask("task", spice)
    logs = [spice.planetStateOutMsgs[i].recorder() for i in range(len(PLANETS))]
    for lg in logs:
        sim.AddModelToTask("task", lg)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(tf))
    sim.ExecuteSimulation()
    out = {}
    for i, name in enumerate(PLANETS):
        p = spice.planetStateOutMsgs[i].read()
        out[name] = dict(
            pos=np.array(p.PositionVector),
            vel=np.array(p.VelocityVector),
            dcm=np.array(p.J20002Pfix),
            dcmDot=np.array(p.J20002Pfix_dot),
            J2000Current=p.J2000Current,
            computeOrient=p.computeOrient,
            name=bytes(p.PlanetName).split(b"\x00")[0].decode() if isinstance(p.PlanetName, (bytes, bytearray)) else str(p.PlanetName),
        )
    return out, spice


def test_defaultsAreBitIdentical():
    """With nothing configured, every planet field must be byte-identical to the exact path.
    (Configuring one planet must not perturb the others, nor the default output of any field.)"""
    dt, tf = 10.0, 300.0
    base, _ = _runToStop(None, dt, tf)

    # Reconstruct earth only; mars & sun must stay bit-identical to the exact run, and earth's
    # J2000Current / computeOrient / PlanetName (non-numeric fields) must also be unchanged.
    def cfg(s):
        s.setPlanetPositionReconstruction("earth", macros.sec2nano(60.0), 2, True, True)
    withInterp, _ = _runToStop(cfg, dt, tf)

    for name in OTHERS:
        assert np.array_equal(base[name]["pos"], withInterp[name]["pos"]), name
        assert np.array_equal(base[name]["vel"], withInterp[name]["vel"]), name
        assert np.array_equal(base[name]["dcm"], withInterp[name]["dcm"]), name
        assert np.array_equal(base[name]["dcmDot"], withInterp[name]["dcmDot"]), name

    # earth non-numeric fields unchanged
    assert base["earth"]["J2000Current"] == withInterp["earth"]["J2000Current"]
    assert base["earth"]["computeOrient"] == withInterp["earth"]["computeOrient"]
    assert base["earth"]["name"] == withInterp["earth"]["name"]

    # A completely unconfigured run twice is trivially identical (guards the exact path itself).
    base2, _ = _runToStop(None, dt, tf)
    for name in PLANETS:
        assert np.array_equal(base[name]["pos"], base2[name]["pos"])
        assert np.array_equal(base[name]["dcm"], base2[name]["dcm"])


def test_positionReconstructionAccurateAndCheaper():
    """Hermite position reconstruction on a coarse grid tracks exact SPICE to well under a metre
    over a short horizon, at far fewer SPICE queries than one-per-step."""
    dt, tf = 10.0, 600.0                      # 60 steps
    exact, _ = _runToStop(None, dt, tf)

    knot = macros.sec2nano(120.0)             # 12x the task step
    def cfg(s):
        s.setPlanetPositionReconstruction("earth", knot, 2, True, True)
    recon, spice = _runToStop(cfg, dt, tf)

    err = np.linalg.norm(recon["earth"]["pos"] - exact["earth"]["pos"])
    assert err < 1.0, f"position reconstruction error too large: {err} m"

    nPos, nOrient = spice.getPlanetSpiceQueryCount("earth")
    # ~ horizon/knot (+ small stencil slack), and certainly far below the exact 60 steps.
    assert nPos <= 12, f"expected ~horizon/knot position queries, got {nPos}"
    # The orientation channel is on the exact path, so it counts one SPICE call per step: many more
    # than the reconstructed position channel.
    assert nOrient > nPos, f"exact orientation channel should query every step, got {nOrient}"


def test_singleKnotOrientationExtrapolationExactForConstantSpinner():
    """Earth's IAU frame is a constant-rate spinner, so extrapolating orientation from a SINGLE
    knot (nKnots=1, interpolate=False) reproduces the exact per-call sxform orientation, using
    only one sxform query for the whole run."""
    dt, tf = 10.0, 600.0
    exact, _ = _runToStop(None, dt, tf)

    def cfg(s):
        s.setPlanetOrientationReconstruction("earth", macros.sec2nano(100000.0), 1, True, False)  # one knot, extrapolate
    recon, spice = _runToStop(cfg, dt, tf)

    dcmErr = np.max(np.abs(recon["earth"]["dcm"] - exact["earth"]["dcm"]))
    assert dcmErr < 1e-9, f"single-knot orientation extrapolation not exact: max|dDCM|={dcmErr}"

    nPos, nOrient = spice.getPlanetSpiceQueryCount("earth")
    assert nOrient == 1, f"constant spinner should need exactly one sxform query, got {nOrient}"
    # The position channel is on the exact path, so it counts one SPICE call per step.
    assert nPos > nOrient, f"exact position channel should query every step, got {nPos}"


def test_disabledChannelsEmitTrivialDefault():
    """A disabled position channel emits zero pos/vel; a disabled orientation channel emits the
    identity DCM and zero rate -- and neither queries SPICE."""
    dt, tf = 10.0, 100.0

    def cfg(s):
        s.setPlanetPositionDisabled("earth")
        s.setPlanetOrientationDisabled("earth")
    recon, spice = _runToStop(cfg, dt, tf)

    assert np.allclose(recon["earth"]["pos"], 0.0)
    assert np.allclose(recon["earth"]["vel"], 0.0)
    assert np.allclose(recon["earth"]["dcm"], np.eye(3))
    assert np.allclose(recon["earth"]["dcmDot"], 0.0)

    nPos, nOrient = spice.getPlanetSpiceQueryCount("earth")
    assert nPos == 0 and nOrient == 0, "disabled channels must not query SPICE"

    # J2000Current / computeOrient / PlanetName are still filled as usual.
    assert recon["earth"]["J2000Current"] > 0.0
    assert recon["earth"]["name"] == "earth"


def test_velocityIsDerivativeOfReconstructedPosition():
    """The emitted VelocityVector must equal the time derivative of the reconstructed position,
    so a downstream Euler step stays self-consistent. Sampled from a single run's recorder (a
    fine task cadence) and finite-differenced with the actual logging step."""
    dt = 1.0
    knot = macros.sec2nano(90.0)
    tf = 300.0

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    proc.addTask(sim.CreateNewTask("task", macros.sec2nano(dt)))
    spice = _newSpice()
    spice.setPlanetPositionReconstruction("earth", knot, 2, True, True)
    sim.AddModelToTask("task", spice)
    rec = spice.planetStateOutMsgs[0].recorder()   # earth
    sim.AddModelToTask("task", rec)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(tf))
    sim.ExecuteSimulation()

    pos = np.array(rec.PositionVector)   # (N,3)
    vel = np.array(rec.VelocityVector)
    # central finite difference of logged position vs the logged (emitted) velocity at interior pts
    vFD = (pos[2:] - pos[:-2]) / (2.0 * dt)
    vEmit = vel[1:-1]
    relErr = np.linalg.norm(vFD - vEmit, axis=1) / np.maximum(np.linalg.norm(vEmit, axis=1), 1e-9)
    assert np.max(relErr) < 1e-4, f"emitted velocity inconsistent with dP/dt: max relErr={np.max(relErr)}"


def test_integratorAgnosticQueryCountOnDynamicsTask():
    """On an MJScene dynamics task the module's UpdateState is called once per integrator STAGE,
    so the exact path would query SPICE (stages x steps) times. With reconstruction the knot cache
    collapses those onto ~horizon/knotStep queries, and that count is the SAME for a fixed RK4 and
    an adaptive RKF45 integrator -- the integrator-agnostic property the feature exists to deliver."""
    mujoco = pytest.importorskip("Basilisk.simulation.mujoco")
    from Basilisk.simulation import svIntegrators

    scXml = """
    <mujoco>
        <worldbody>
            <body name="sat"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
        </worldbody>
    </mujoco>
    """
    knot = macros.sec2nano(120.0)
    tf = 1200.0     # 10 knot intervals

    def queryCount(makeIntegrator, dt):
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("proc")
        proc.addTask(sim.CreateNewTask("task", macros.sec2nano(dt)))
        scene = mujoco.MJScene(scXml)
        sim.AddModelToTask("task", scene)
        integ = makeIntegrator(scene)
        scene.setIntegrator(integ)

        spice = _newSpice()
        spice.setPlanetPositionReconstruction("earth", knot, 2, True, True)
        scene.AddModelToDynamicsTask(spice)

        keep = [integ, spice]  # keep Python refs alive (svIntegrator GC segfault guard)  # noqa: F841
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(tf))
        sim.ExecuteSimulation()
        nPos, _ = spice.getPlanetSpiceQueryCount("earth")
        return nPos

    nRK4 = queryCount(lambda sc: svIntegrators.svIntegratorRK4(sc), 20.0)
    nRKF45 = queryCount(lambda sc: svIntegrators.svIntegratorRKF45(sc), 20.0)

    # ~horizon/knot (+ a little stencil slack), and identical across integrators despite very
    # different stage/sub-step counts.
    assert nRK4 <= 15, f"expected ~horizon/knot queries on dynamics task, got {nRK4}"
    assert nRK4 == nRKF45, f"query count must be integrator-agnostic: RK4={nRK4} RKF45={nRKF45}"


def test_exactSpiceResetRestoresDefault():
    """setPlanetExactSpice reverts both channels; output returns to the exact path bit-for-bit."""
    dt, tf = 10.0, 200.0
    exact, _ = _runToStop(None, dt, tf)

    def cfg(s):
        s.setPlanetPositionReconstruction("earth", macros.sec2nano(60.0), 2, True, True)
        s.setPlanetOrientationReconstruction("earth", macros.sec2nano(60.0), 2, True, True)
        s.setPlanetExactSpice("earth")   # undo both
    reverted, spice = _runToStop(cfg, dt, tf)

    assert np.array_equal(exact["earth"]["pos"], reverted["earth"]["pos"])
    assert np.array_equal(exact["earth"]["dcm"], reverted["earth"]["dcm"])
    # Both channels are back on the exact path, which counts one SPICE call per step (never knots).
    # With the coarse 60 s knot grid they were configured with, a knot-only count would be tiny;
    # the per-step counts here confirm the exact path is running.
    nPos, nOrient = spice.getPlanetSpiceQueryCount("earth")
    assert nPos == nOrient and nPos > tf / dt, \
        f"exact-reverted channels should query every step on both channels, got {nPos}, {nOrient}"


def test_singleKnotDefaultsAreExactRegardlessOfInterpolateFlag():
    """A single-knot stencil (nKnots=1) is the enclosing knot for BOTH interpolate settings, so
    orientation reconstruction of a constant spinner is exact even with the enabled default
    interpolate=True (regression: a naive 1-knot formula would pick the future knot k+1)."""
    dt, tf = 10.0, 600.0
    exact, _ = _runToStop(None, dt, tf)

    for interp in (True, False):
        def cfg(s, interp=interp):
            s.setPlanetOrientationReconstruction("earth", macros.sec2nano(100000.0), 1, True, interp)
        recon, spice = _runToStop(cfg, dt, tf)
        dcmErr = np.max(np.abs(recon["earth"]["dcm"] - exact["earth"]["dcm"]))
        assert dcmErr < 1e-9, f"nKnots=1 interpolate={interp} not exact: max|dDCM|={dcmErr}"
        _, nOrient = spice.getPlanetSpiceQueryCount("earth")
        assert nOrient == 1, f"nKnots=1 interpolate={interp} should need one query, got {nOrient}"


def test_oddKnotStencilIsCentered():
    """A 3-knot interpolation stencil is centred on the current interval ([k-1, k, k+1]); a
    forward-biased [k, k+1, k+2] stencil would be less accurate. Guards buildStencil's centering:
    3-knot reconstruction must be at least as accurate as 2-knot on the same coarse grid."""
    dt, tf = 10.0, 600.0
    exact, _ = _runToStop(None, dt, tf)
    knot = macros.sec2nano(150.0)

    def cfg2(s):
        s.setPlanetPositionReconstruction("earth", knot, 2, True, True)
    def cfg3(s):
        s.setPlanetPositionReconstruction("earth", knot, 3, True, True)
    r2, _ = _runToStop(cfg2, dt, tf)
    r3, _ = _runToStop(cfg3, dt, tf)

    err2 = np.linalg.norm(r2["earth"]["pos"] - exact["earth"]["pos"])
    err3 = np.linalg.norm(r3["earth"]["pos"] - exact["earth"]["pos"])
    # A centred cubic (3-knot Hermite) should not be worse than the 2-knot fit; a forward-biased
    # stencil would regress here. Both are tiny; require 3-knot within a small factor of 2-knot.
    assert err3 <= max(err2 * 2.0, 1e-3), f"3-knot stencil not centred: err2={err2} err3={err3}"


def test_reconfiguringKnotStepClearsStaleCache():
    """Reconfiguring a channel's knotStep must drop the cached knots: they are keyed by integer
    index on the OLD grid, so reusing them would map indices to the wrong epochs. Reconfigure the
    grid mid-life (after one run has populated knots) and confirm a second run still tracks exact
    SPICE, i.e. it re-queried on the new grid rather than reusing the old samples."""
    dt, tf = 10.0, 600.0
    exact, _ = _runToStop(None, dt, tf)

    # Build one spice object, run once with a coarse grid (populates knots), then reconfigure to a
    # much finer grid and run again. Stale reuse would leave the fine run tracking the coarse epochs.
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    proc.addTask(sim.CreateNewTask("task", macros.sec2nano(dt)))
    spice = _newSpice()
    spice.setPlanetPositionReconstruction("earth", macros.sec2nano(300.0), 2, True, True)
    sim.AddModelToTask("task", spice)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(tf))
    sim.ExecuteSimulation()

    spice.setPlanetPositionReconstruction("earth", macros.sec2nano(120.0), 2, True, True)  # new grid
    sim.ConfigureStopTime(macros.sec2nano(2 * tf))
    sim.ExecuteSimulation()

    p = spice.planetStateOutMsgs[0].read()
    pos = np.array(p.PositionVector)
    # Truth at the final time (2*tf) on the exact path.
    exact2, _ = _runToStop(None, dt, 2 * tf)
    err = np.linalg.norm(pos - exact2["earth"]["pos"])
    assert err < 1.0, f"reconfigured knotStep reused stale cache: {err} m from exact SPICE"


def test_enablingReconstructionAfterResetAnchorsAtEpoch():
    """Enabling reconstruction for the FIRST time between two ExecuteSimulation() segments (so the
    reconstruction object is created after Reset already set the epoch) must anchor knots at the
    init epoch, not J2000 ET 0. Otherwise the knot grid would query the wrong epoch and the state
    would be wildly wrong. Regression for the deferred-creation et0 anchoring."""
    dt, tf = 10.0, 300.0

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    proc.addTask(sim.CreateNewTask("task", macros.sec2nano(dt)))
    spice = _newSpice()                    # NOTHING configured -> recon object not yet created
    sim.AddModelToTask("task", spice)
    sim.InitializeSimulation()             # Reset runs here with recon still null
    sim.ConfigureStopTime(macros.sec2nano(tf))
    sim.ExecuteSimulation()

    # First enable reconstruction now, mid-run (this is when the recon object gets created).
    spice.setPlanetPositionReconstruction("earth", macros.sec2nano(120.0), 2, True, True)
    sim.ConfigureStopTime(macros.sec2nano(2 * tf))
    sim.ExecuteSimulation()

    pos = np.array(spice.planetStateOutMsgs[0].read().PositionVector)
    exact2, _ = _runToStop(None, dt, 2 * tf)
    err = np.linalg.norm(pos - exact2["earth"]["pos"])
    assert err < 1.0, f"reconstruction enabled after Reset anchored at wrong epoch: {err} m"


def test_queryCountTracksExactPathWithNoReconstruction():
    """getPlanetSpiceQueryCount reports SPICE calls even when NO reconstruction is configured: a
    plain exact-path planet queries both channels once per update, so the counts equal the number of
    updates. An added-but-never-queried name reads {0, 0}; a name never added reads {-1, -1}."""
    dt, tf = 10.0, 300.0
    _, spice = _runToStop(None, dt, tf)   # nothing configured: pure exact path

    # earth carries a position and a constant-rate IAU_EARTH orientation, so both channels query
    # SPICE once per update, i.e. about one per task step over the run (plus Reset's own update).
    steps = int(round(tf / dt))
    nPos, nOrient = spice.getPlanetSpiceQueryCount("earth")
    assert steps <= nPos <= steps + 2, f"exact-path position queries ~one per step: got {nPos}"
    assert nOrient == nPos, f"both exact channels query per update: pos {nPos}, orient {nOrient}"

    # every added planet is tracked on the exact path: its position channel queries per update,
    # and its orientation channel queries per update when it has a frame or is 0 when it does not.
    sPos, sOrient = spice.getPlanetSpiceQueryCount("mars barycenter")
    assert sPos == nPos, f"added planet position queries per update: got {sPos}"
    assert sOrient in (0, nPos), f"orientation queries are per-update or (frameless) zero: {sOrient}"

    # a name that was never added via addPlanetNames is the only {-1, -1} case.
    assert tuple(spice.getPlanetSpiceQueryCount("jupiter")) == (-1, -1)


def test_misconfiguredPlanetWarnsNotThrows():
    """A reconstruction config for a planet that was never added (or has no orientation frame)
    must WARN and keep running, not throw BasiliskError and abort the sim. The stale entry is
    pruned, so its query count reports unconfigured afterwards."""
    dt, tf = 10.0, 100.0

    # (a) unknown planet name: sim must complete, entry pruned.
    def cfgUnknown(s):
        s.setPlanetPositionReconstruction("jupiter", macros.sec2nano(60.0), 2, True, True)
    _, spice = _runToStop(cfgUnknown, dt, tf)  # must not raise
    assert tuple(spice.getPlanetSpiceQueryCount("jupiter")) == (-1, -1), \
        "stale (unadded) planet config should be pruned at Reset"

    # (b) setPlanetExactSpice on a never-configured planet must not abort the sim.
    def cfgExact(s):
        s.setPlanetExactSpice("venus")
    _runToStop(cfgExact, dt, tf)  # must not raise


if __name__ == "__main__":
    test_defaultsAreBitIdentical()
    test_positionReconstructionAccurateAndCheaper()
    test_singleKnotOrientationExtrapolationExactForConstantSpinner()
    test_disabledChannelsEmitTrivialDefault()
    test_velocityIsDerivativeOfReconstructedPosition()
    test_exactSpiceResetRestoresDefault()
    test_singleKnotDefaultsAreExactRegardlessOfInterpolateFlag()
    test_oddKnotStencilIsCentered()
    test_reconfiguringKnotStepClearsStaleCache()
    test_enablingReconstructionAfterResetAnchorsAtEpoch()
    test_queryCountTracksExactPathWithNoReconstruction()
    test_misconfiguredPlanetWarnsNotThrows()
    print("all reconstruction unit tests passed")
