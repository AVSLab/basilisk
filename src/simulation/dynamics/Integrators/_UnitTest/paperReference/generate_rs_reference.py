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
Reference-trajectory generator for the Roessler RS1/RS2 Stratonovich weak-order-2
integrators with MULTIPLE (m > 1), possibly non-commutative, noise sources.

Why a separate reference from the Julia one: StochasticDiffEq.jl's in-place RS cache
has an array-aliasing defect in its multi-noise cross-terms (it does ``H22[k] = uprev``
- aliasing rather than copying the working state - then mutates it in place), so its
m > 1 trajectory does not match the method as published. This generator therefore
implements the method directly from the primary source,

    A. Roessler, "Second Order Runge-Kutta Methods for Stratonovich Stochastic
    Differential Equations", BIT Numer. Math. 47 (2007), 657-680,

equations (5.1)-(5.2) with the Table 5.2/5.3 coefficients. It is validated against the
paper's own published error tables (Table 6.1 commutative, Table 6.3 non-commutative)
by ``validate_paper_tables()`` below, which reproduces Roessler's RS1 weak errors to
3 significant figures and shows the expected weak order 2.

Like the Julia generator, this PRESCRIBES the underlying Gaussian increments (dW, dZ)
and applies exactly the transforms the Basilisk integrator uses
(``stochasticWeakRandomVariables``: three-point for Ihat, two-point for Itilde), so the
Basilisk test can replay the identical increments and compare to floating-point
tolerance. This script is not run in CI; the JSON it writes is committed.

Run:  python3 generate_rs_reference.py
"""
from __future__ import annotations

import json
import os

import numpy as np

from paperCommon import three_point, two_point


def rs_coeffs(kind: str):
    s = 4
    A0 = np.zeros((s, s)); A1 = np.zeros((s, s)); A2 = np.zeros((s, s))
    B0 = np.zeros((s, s)); B1 = np.zeros((s, s)); B2 = np.zeros((s, s)); B3 = np.zeros((s, s))
    A0[2, 0] = 1.0
    A1[2, 0] = 1.0; A1[3, 0] = 1.0
    B0[2, 0] = 1 / 4; B0[2, 1] = 3 / 4
    B1[1, 0] = 2 / 3; B1[2, 0] = 1 / 12; B1[2, 1] = 1 / 4
    B1[3, 0] = -5 / 4; B1[3, 1] = 1 / 4; B1[3, 2] = 2.0
    B3[2, 0] = 1 / 4; B3[2, 1] = 3 / 4; B3[3, 0] = 1 / 4; B3[3, 1] = 3 / 4
    B2[1, 0] = 1.0; B2[2, 0] = -1.0
    beta1 = np.array([1 / 8, 3 / 8, 3 / 8, 1 / 8])
    beta2 = np.array([0.0, -1 / 4, 1 / 4, 0.0])
    if kind == "RS1":
        alpha = np.array([0.0, 0.0, 1 / 2, 1 / 2])
    elif kind == "RS2":
        alpha = np.array([1 / 4, 1 / 4, 1 / 2, 0.0])
        A0[1, 0] = 2 / 3; A0[2, 0] = 1 / 6; A0[2, 1] = 1 / 2
    else:
        raise ValueError(kind)
    return dict(s=s, A0=A0, A1=A1, A2=A2, B0=B0, B1=B1, B2=B2, B3=B3,
                alpha=alpha, beta1=beta1, beta2=beta2)


def rs_step(X, a, bcols, Ihat, Itilde, h, C):
    """One RS step (vectorized over rows of X). a(X)->(M,d); bcols(X)->list of m (M,d);
    Ihat (M,m); Itilde (M,m-1). Returns (M,d)."""
    s = C["s"]; A0, A1, A2 = C["A0"], C["A1"], C["A2"]
    B0, B1, B2, B3 = C["B0"], C["B1"], C["B2"], C["B3"]
    alpha, beta1, beta2 = C["alpha"], C["beta1"], C["beta2"]
    sqh = np.sqrt(h); m = Ihat.shape[1]

    def ihat2(k, l):  # eq. (5.2)
        return (Ihat[:, k] * Itilde[:, l]) if l < k else (-Ihat[:, l] * Itilde[:, k])

    aH0 = [None] * s
    bHk = [[None] * s for _ in range(m)]
    H0 = [X.copy() for _ in range(s)]
    Hk = [[X.copy() for _ in range(s)] for _ in range(m)]
    Hhat = [[X.copy() for _ in range(s)] for _ in range(m)]

    for i in range(s):
        v = X.copy()
        for j in range(i):
            v = v + A0[i, j] * aH0[j] * h
            for l in range(m):
                v = v + B0[i, j] * bHk[l][j] * Ihat[:, l][:, None]
        H0[i] = v
        aH0[i] = a(H0[i])
        for k in range(m):
            v = X.copy()
            for j in range(i):
                v = v + A1[i, j] * aH0[j] * h + B1[i, j] * bHk[k][j] * Ihat[:, k][:, None]
                for l in range(m):
                    if l != k:
                        v = v + B3[i, j] * bHk[l][j] * Ihat[:, l][:, None]
            Hk[k][i] = v
        cols = [bcols(Hk[k][i]) for k in range(m)]
        for k in range(m):
            bHk[k][i] = cols[k][k]
        for k in range(m):
            v = X.copy()
            for j in range(i):
                v = v + A2[i, j] * aH0[j] * h
                for l in range(m):
                    if l != k:
                        v = v + B2[i, j] * bHk[l][j] * (ihat2(k, l) / sqh)[:, None]
            Hhat[k][i] = v

    u = X.copy()
    for i in range(s):
        u = u + alpha[i] * aH0[i] * h
    for i in range(s):
        for k in range(m):
            u = u + beta1[i] * bHk[k][i] * Ihat[:, k][:, None]
            u = u + beta2[i] * bcols(Hhat[k][i])[k] * sqh
    return u


# ---------------------------------------------------------------------------
# Test problems (mirror paperReference README + the Basilisk test's f/g).
# ---------------------------------------------------------------------------
def problem_noncommutative():
    """Roessler SDE (6.2): 2 states, 2 non-commutative noise sources."""
    def a(X):
        return np.stack([1.25 * X[:, 1] - 1.25 * X[:, 0],
                         0.25 * X[:, 0] - 0.25 * X[:, 1]], axis=1)

    def bcols(X):
        x1, x2 = X[:, 0], X[:, 1]
        b1 = np.stack([np.sqrt(3) / 2 * (x1 - x2), np.zeros_like(x1)], axis=1)
        b2 = np.stack([0.5 * (x1 + x2), x1], axis=1)
        return [b1, b2]

    return dict(name="noncommutative2d", u0=[0.1, 0.1], n=2, m=2, a=a, bcols=bcols)


def problem_commutative():
    """Roessler SDE (6.1): 2 states, 2 commutative (diagonal) noise sources."""
    av, bv = 299 / 200, 0.1

    def a(X):
        return av * X

    def bcols(X):
        b1 = np.zeros_like(X); b1[:, 0] = bv * X[:, 0]
        b2 = np.zeros_like(X); b2[:, 1] = bv * X[:, 1]
        return [b1, b2]

    return dict(name="commutative2d", u0=[0.1, 0.1], n=2, m=2, a=a, bcols=bcols)


PROBLEMS = [problem_noncommutative(), problem_commutative()]
METHODS = ["RS1", "RS2"]


def generate():
    dt = 1.0 / 8.0
    tf = 1.0
    seed = 424242
    cases = []
    for prob in PROBLEMS:
        for method in METHODS:
            C = rs_coeffs(method)
            n = prob["n"]; m = prob["m"]
            ts = np.arange(0.0, tf + dt / 2, dt)
            nsteps = len(ts) - 1
            rng = np.random.default_rng(seed)
            seed += 1
            dW = np.sqrt(dt) * rng.standard_normal((nsteps, m))
            dZ = np.sqrt(dt) * rng.standard_normal((nsteps, m))
            X = np.array(prob["u0"], dtype=float).reshape(1, n)
            traj = [X[0].tolist()]
            for step in range(nsteps):
                Ihat = np.array([[three_point(dW[step, k], dt) for k in range(m)]])
                Itilde = np.array([[two_point(dZ[step, k], np.sqrt(dt)) for k in range(m)]])
                X = rs_step(X, prob["a"], prob["bcols"], Ihat, Itilde, dt, C)
                traj.append(X[0].tolist())
            cases.append(dict(
                method=method, problem=prob["name"], dt=dt, tf=tf,
                n_states=n, m=m, n_steps=nsteps, seed=seed - 1,
                u0=prob["u0"],
                dW=[dW[i].tolist() for i in range(nsteps)],
                dZ=[dZ[i].tolist() for i in range(nsteps)],
                times=ts.tolist(),
                trajectory=traj,
            ))
            print(f"generated: {method} / {prob['name']}  final={traj[-1]}")
    out = dict(generator="paper: Roessler 2007 BIT 47:657 (RS1/RS2)", cases=cases)
    path = os.path.join(os.path.dirname(__file__), "rs_reference_trajectories.json")
    with open(path, "w") as f:
        json.dump(out, f)
        f.write("\n")  # trailing newline for the end-of-file-fixer pre-commit hook
    print(f"\nWrote {len(cases)} cases to {path}")


def validate_paper_tables(M=2_000_000):
    """Reproduce Roessler's published RS1 weak-error tables to confirm correctness."""
    rng = np.random.default_rng(0)
    T = 1.0

    def draw(m, h):
        sq3h = np.sqrt(3 * h)
        U = rng.random((M, m))
        Ih = np.where(U < 1 / 6, -sq3h, np.where(U < 1 / 3, sq3h, 0.0))
        It = np.where(rng.random((M, max(m - 1, 1))) < 0.5, np.sqrt(h), -np.sqrt(h))
        return Ih, It

    print("Validation vs Roessler Table 6.1 (commutative SDE 6.1), RS1:")
    print("  paper: h=1->8.57e-2, 1/2->3.56e-2, 1/4->1.18e-2, 1/8->3.40e-3")
    prob = problem_commutative(); C = rs_coeffs("RS1"); exact = 0.1 * np.exp(1.5 * T)
    for ns in [1, 2, 4, 8]:
        h = T / ns; X = np.tile(np.array(prob["u0"]), (M, 1))
        for _ in range(ns):
            Ih, It = draw(prob["m"], h); X = rs_step(X, prob["a"], prob["bcols"], Ih, It, h, C)
        print(f"    ns={ns} h={h:.4f} weak_err(E[X1])={abs(X[:, 0].mean() - exact):.3e}")

    print("Validation vs Roessler Table 6.3 (non-commutative SDE 6.2), RS1:")
    print("  paper: h=1->2.37e-3, 1/2->7.13e-4, 1/4->1.91e-4, 1/8->4.67e-5 (weak order 2)")
    prob = problem_noncommutative(); C = rs_coeffs("RS1"); exact = 0.1 * np.exp(0.5 * T)
    prev = None
    for ns in [1, 2, 4, 8]:
        h = T / ns; X = np.tile(np.array(prob["u0"]), (M, 1))
        for _ in range(ns):
            Ih, It = draw(prob["m"], h); X = rs_step(X, prob["a"], prob["bcols"], Ih, It, h, C)
        err = abs(X[:, 0].mean() - exact)
        rate = "" if prev is None else f"  rate={np.log2(prev / err):.2f}"
        print(f"    ns={ns} h={h:.4f} weak_err(E[X1])={err:.3e}{rate}")
        prev = err


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "validate":
        validate_paper_tables()
    else:
        generate()
