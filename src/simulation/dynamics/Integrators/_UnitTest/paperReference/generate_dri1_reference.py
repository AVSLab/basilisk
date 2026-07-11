#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
"""
Paper-faithful reference for the DRI1 (Debrabant-Roessler) weak-order-2 Ito integrator
with genuinely NON-COMMUTATIVE noise.

Why not the Julia reference: StochasticDiffEq.jl's DRI1 non-diagonal (matrix-noise)
branch raises an ``UndefVarError`` (an ``l``/``k`` loop-variable mix-up in its
non-diagonal code path), so it cannot solve a coupled-noise DRI1 problem at all. The
diagonal path works and is covered by the Julia test, but that path only weakly
exercises the mixed-integral cross-terms. To test DRI1's headline capability -
non-commutative noise - we implement DRI1 directly, generalized to vector-valued
diffusion columns, transcribed from the DRI1 constant-cache perform_step.

The oracle is validated (see ``validate()``) to reproduce weak order 2 on a
non-commutative linear Ito SDE with a known mean E[X_T] = expm(A T) X0. As elsewhere the
Gaussian increments are prescribed so the Basilisk test can replay them exactly.

Run:  python3 generate_dri1_reference.py           (writes data/dri1_reference_trajectories.json)
      python3 generate_dri1_reference.py validate  (weak-order check)
"""
from __future__ import annotations

import json
import os

import numpy as np

from paperCommon import three_point, two_point

# Coefficient sets sharing the DRI1 step (DRI1 and the Roessler-2009 RI family). Keys
# match the DRI1ConstantCache fields.
COEFFS = {
    "DRI1": dict(
        a021=1 / 2, a031=-1.0, a032=2.0, a121=342 / 491, a131=342 / 491,
        b021=(6 - np.sqrt(6)) / 10, b031=(3 + 2 * np.sqrt(6)) / 5,
        b121=3 * np.sqrt(38 / 491), b131=-3 * np.sqrt(38 / 491),
        b221=-214 / 513 * np.sqrt(1105 / 991), b222=-491 / 513 * np.sqrt(221 / 4955),
        b223=-491 / 513 * np.sqrt(221 / 4955), b231=214 / 513 * np.sqrt(1105 / 991),
        b232=491 / 513 * np.sqrt(221 / 4955), b233=491 / 513 * np.sqrt(221 / 4955),
        alpha1=1 / 6, alpha2=2 / 3, alpha3=1 / 6,
        beta11=193 / 684, beta12=491 / 1368, beta13=491 / 1368,
        beta22=1 / 6 * np.sqrt(491 / 38), beta23=-1 / 6 * np.sqrt(491 / 38),
        beta31=-4955 / 7072, beta32=4955 / 14144, beta33=4955 / 14144,
        beta42=-1 / 8 * np.sqrt(4955 / 221), beta43=1 / 8 * np.sqrt(4955 / 221)),
    "RI1": dict(
        a021=2 / 3, a031=-1 / 3, a032=1.0, a121=1.0, a131=1.0,
        b021=1.0, b031=0.0, b121=1.0, b131=-1.0,
        b221=1.0, b222=0.0, b223=0.0, b231=-1.0, b232=0.0, b233=0.0,
        alpha1=1 / 4, alpha2=1 / 2, alpha3=1 / 4,
        beta11=1 / 2, beta12=1 / 4, beta13=1 / 4, beta22=1 / 2, beta23=-1 / 2,
        beta31=-1 / 2, beta32=1 / 4, beta33=1 / 4, beta42=1 / 2, beta43=-1 / 2),
    "RI3": dict(
        a021=1.0, a031=1 / 4, a032=1 / 4, a121=1.0, a131=1.0,
        b021=(3 - 2 * np.sqrt(6)) / 5, b031=(6 + np.sqrt(6)) / 10, b121=1.0, b131=-1.0,
        b221=1.0, b222=0.0, b223=0.0, b231=-1.0, b232=0.0, b233=0.0,
        alpha1=1 / 6, alpha2=1 / 6, alpha3=2 / 3,
        beta11=1 / 2, beta12=1 / 4, beta13=1 / 4, beta22=1 / 2, beta23=-1 / 2,
        beta31=-1 / 2, beta32=1 / 4, beta33=1 / 4, beta42=1 / 2, beta43=-1 / 2),
    "RI5": dict(
        a021=1.0, a031=25 / 144, a032=35 / 144, a121=1 / 4, a131=1 / 4,
        b021=1 / 3, b031=-5 / 6, b121=1 / 2, b131=-1 / 2,
        b221=1.0, b222=0.0, b223=0.0, b231=-1.0, b232=0.0, b233=0.0,
        alpha1=1 / 10, alpha2=3 / 14, alpha3=24 / 35,
        beta11=1.0, beta12=-1.0, beta13=-1.0, beta22=1.0, beta23=-1.0,
        beta31=1 / 2, beta32=-1 / 4, beta33=-1 / 4, beta42=1 / 2, beta43=-1 / 2),
    "RI6": dict(
        a021=1.0, a031=0.0, a032=0.0, a121=1.0, a131=1.0,
        b021=1.0, b031=0.0, b121=1.0, b131=-1.0,
        b221=1.0, b222=0.0, b223=0.0, b231=-1.0, b232=0.0, b233=0.0,
        alpha1=1 / 2, alpha2=1 / 2, alpha3=0.0,
        beta11=1 / 2, beta12=1 / 4, beta13=1 / 4, beta22=1 / 2, beta23=-1 / 2,
        beta31=-1 / 2, beta32=1 / 4, beta33=1 / 4, beta42=1 / 2, beta43=-1 / 2),
}


def dri1_step(X, a, bcols, dWg, dZg, h, C=None):
    if C is None:
        C = COEFFS["DRI1"]
    (a021, a031, a032, a121, a131, b021, b031, b121, b131, b221, b222, b223, b231,
     b232, b233, alpha1, alpha2, alpha3, beta11, beta12, beta13, beta22, beta23,
     beta31, beta32, beta33, beta42, beta43) = (
        C["a021"], C["a031"], C["a032"], C["a121"], C["a131"], C["b021"], C["b031"],
        C["b121"], C["b131"], C["b221"], C["b222"], C["b223"], C["b231"], C["b232"],
        C["b233"], C["alpha1"], C["alpha2"], C["alpha3"], C["beta11"], C["beta12"],
        C["beta13"], C["beta22"], C["beta23"], C["beta31"], C["beta32"], C["beta33"],
        C["beta42"], C["beta43"])
    m = dWg.shape[1]; sqh = np.sqrt(h)
    _dW = three_point(dWg, h); _dZ = two_point(dZg, np.sqrt(h))
    chi1 = (_dW ** 2 - h) / 2.0

    def Ihat2(k, l):
        if k < l:
            return (_dW[:, k] * _dW[:, l] - sqh * _dZ[:, k]) / 2.0
        return (_dW[:, k] * _dW[:, l] + sqh * _dZ[:, l]) / 2.0

    k1 = a(X); g1 = bcols(X)
    H02 = X + a021 * k1 * h
    for l in range(m):
        H02 = H02 + b021 * g1[l] * _dW[:, l][:, None]
    k2 = a(H02)
    H03 = X + a031 * k1 * h + a032 * k2 * h
    for l in range(m):
        H03 = H03 + b031 * g1[l] * _dW[:, l][:, None]
    k3 = a(H03)

    H12 = [X + a121 * k1 * h + b121 * g1[k] * sqh for k in range(m)]
    H13 = [X + a131 * k1 * h + b131 * g1[k] * sqh for k in range(m)]
    g2 = [bcols(H12[k]) for k in range(m)]
    g3 = [bcols(H13[k]) for k in range(m)]

    H22 = [X + (b221 * g1[k] + b222 * g2[k][k] + b223 * g3[k][k]) * sqh for k in range(m)]
    H23 = [X + (b231 * g1[k] + b232 * g2[k][k] + b233 * g3[k][k]) * sqh for k in range(m)]
    gH22 = [bcols(H22[k]) for k in range(m)]
    gH23 = [bcols(H23[k]) for k in range(m)]

    u = X + alpha1 * k1 * h + alpha2 * k2 * h + alpha3 * k3 * h
    for k in range(m):
        u = u + g1[k] * (_dW[:, k] * beta11)[:, None]
        u = u + g2[k][k] * (_dW[:, k] * beta12 + chi1[:, k] * beta22 / sqh)[:, None]
        u = u + g3[k][k] * (_dW[:, k] * beta13 + chi1[:, k] * beta23 / sqh)[:, None]
        if m > 1:
            u = u + g1[k] * ((m - 1) * beta31 * _dW[:, k])[:, None]
            for l in range(m):
                if l == k:
                    continue
                ih = Ihat2(k, l)
                u = u + gH22[l][k] * (_dW[:, k] * beta32 + ih * beta42 / sqh)[:, None]
                u = u + gH23[l][k] * (_dW[:, k] * beta33 + ih * beta43 / sqh)[:, None]
    return u


# Non-commutative linear Ito test problem: dX = A X dt + B0 X dW0 + B1 X dW1,
# with [B0, B1] != 0. Basilisk reconstructs the same A, B0, B1.
A = np.array([[-0.5, 0.2], [0.1, -0.4]])
B0 = np.array([[0.3, 0.0], [0.0, 0.1]])
B1 = np.array([[0.0, 0.15], [0.25, 0.0]])


def a_prob(X):
    return X @ A.T


def bcols_prob(X):
    return [X @ B0.T, X @ B1.T]


def generate():
    dt = 1 / 8
    tf = 1.0
    x0 = [0.5, 1.0]
    m = 2
    n = 2
    ts = np.arange(0.0, tf + dt / 2, dt)
    nsteps = len(ts) - 1
    cases = []
    # DRI1 and the RI family (RI1/RI3/RI5/RI6) share the same step; all are exercised on
    # the same non-commutative linear Ito problem (a different seed per method).
    for method, seed in [("DRI1", 31), ("RI1", 32), ("RI3", 33), ("RI5", 34), ("RI6", 35)]:
        rng = np.random.default_rng(seed)
        dW = np.sqrt(dt) * rng.standard_normal((nsteps, m))
        dZ = np.sqrt(dt) * rng.standard_normal((nsteps, m))
        X = np.array(x0, dtype=float).reshape(1, n)
        traj = [X[0].tolist()]
        for step in range(nsteps):
            X = dri1_step(X, a_prob, bcols_prob, dW[step:step + 1], dZ[step:step + 1], dt,
                          COEFFS[method])
            traj.append(X[0].tolist())
        cases.append(dict(method=method, problem=f"{method.lower()}_noncommutative2d",
                          dt=dt, tf=tf, n_states=n, m=m, n_steps=nsteps, u0=x0,
                          dW=[dW[i].tolist() for i in range(nsteps)],
                          dZ=[dZ[i].tolist() for i in range(nsteps)],
                          times=ts.tolist(), trajectory=traj))
        print(f"generated {method} non-commutative, final={cases[-1]['trajectory'][-1]}")
    out = dict(generator="paper-faithful DRI1/RI (Debrabant-Roessler / Roessler), non-commutative",
               cases=cases)
    path = os.path.join(os.path.dirname(__file__), "dri1_reference_trajectories.json")
    with open(path, "w") as f:
        json.dump(out, f)
        f.write("\n")  # trailing newline for the end-of-file-fixer pre-commit hook
    print(f"Wrote {len(cases)} cases to {path}")


def validate(M=3_000_000):
    import scipy.linalg as sla
    T = 1.0
    x0 = np.array([0.5, 1.0])
    exact = sla.expm(A * T) @ x0  # E[X_T] for a linear Ito SDE
    rng = np.random.default_rng(7)
    print(f"[B0,B1] max = {np.max(np.abs(B0 @ B1 - B1 @ B0)):.3f} (non-commutative)")
    print(f"exact E[X_T] = {exact}")
    prev = None
    for ns in [1, 2, 4, 8]:
        h = T / ns
        X = np.tile(x0, (M, 1))
        for _ in range(ns):
            dW = np.sqrt(h) * rng.standard_normal((M, 2))
            dZ = np.sqrt(h) * rng.standard_normal((M, 2))
            X = dri1_step(X, a_prob, bcols_prob, dW, dZ, h)
        err = abs(X[:, 0].mean() - exact[0])
        rate = "" if prev is None else f"  rate={np.log2(prev / err):.2f}"
        print(f"  ns={ns} h={h:.4f} weak_err(E[X1])={err:.3e}{rate}")
        prev = err


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "validate":
        validate()
    else:
        generate()
