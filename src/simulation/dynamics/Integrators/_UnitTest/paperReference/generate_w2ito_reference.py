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
Paper-faithful reference generator for the Tang & Xiao W2Ito1/W2Ito2 weak-order-2 SRK
methods (Ito), from the primary source:

    Tang, X., Xiao, A. "Efficient weak second-order stochastic Runge-Kutta methods for
    Ito stochastic differential equations", BIT Numer. Math. 57, 241-260 (2017).
    https://doi.org/10.1007/s10543-016-0618-9

The StochasticDiffEq.jl reference library only provides W2Ito1 (there is no W2Ito2 in
it), so W2Ito2's multi-noise / non-commutative behaviour cannot be checked against Julia.
This module implements the general SRK scheme (paper eq. 3.1) directly from the extended
Butcher tableau (Tables 2 and 3), using the paper's iterated-integral definitions (eq.
3.3), and writes trajectories for prescribed Gaussian increments. The Basilisk test
replays the identical increments and must match.

The generator can also self-validate: ``python generate_w2ito_reference.py validate``
shows weak-order-2 convergence of E[X_T] on a linear Ito SDE with a known mean.
"""
import json
import os
import sys

import numpy as np

from paperCommon import three_point, two_point

# Extended Butcher tableaux (paper Tables 2 and 3). Each is (alpha, beta0, beta1, A0, B0,
# A1, B1, B2) with strictly-lower-triangular s x s matrices.
sqrt6 = np.sqrt(6.0)
TABLEAUX = {
    "W2Ito1": dict(
        alpha=[1 / 6, 2 / 3, 1 / 6],
        beta0=[-1.0, 1.0, 1.0],
        beta1=[2.0, 0.0, -2.0],
        A0=[[0, 0, 0], [1 / 2, 0, 0], [-1, 2, 0]],
        B0=[[0, 0, 0], [(6 - sqrt6) / 10, 0, 0], [(3 + 2 * sqrt6) / 5, 0, 0]],
        A1=[[0, 0, 0], [1 / 4, 0, 0], [1 / 4, 0, 0]],
        B1=[[0, 0, 0], [1 / 2, 0, 0], [-1 / 2, 0, 0]],
        B2=[[0, 0, 0], [1.0, 0, 0], [0, 0, 0]],
    ),
    "W2Ito2": dict(
        alpha=[1 / 6, 1 / 3, 1 / 3, 1 / 6],
        beta0=[0.0, -1.0, 1.0, 1.0],
        beta1=[0.0, 2.0, 0.0, -2.0],
        A0=[[0, 0, 0, 0], [1 / 2, 0, 0, 0], [0, 1 / 2, 0, 0], [0, 0, 1, 0]],
        B0=[[0, 0, 0, 0], [0, 0, 0, 0], [0, 1, 0, 0], [0, 1, 0, 0]],
        A1=[[0, 0, 0, 0], [1 / 2, 0, 0, 0], [1 / 2, 0, 0, 0], [1 / 2, 0, 0, 0]],
        B1=[[0, 0, 0, 0], [0, 0, 0, 0], [0, 1 / 2, 0, 0], [0, -1 / 2, 0, 0]],
        B2=[[0, 0, 0, 0], [0, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0]],
    ),
}


def w2ito_step(X, a, bcols, dWg, dZg, h, T):
    """One W2Ito step (paper eq. 3.1) for a single sample.

    X: (n,) state; a(x)->(n,) drift; bcols(x)->list of m columns (n,);
    dWg,dZg: (m,) Gaussian increments ~N(0,h); T: tableau dict.
    """
    alpha = T["alpha"]; beta0 = T["beta0"]; beta1 = T["beta1"]
    A0 = np.array(T["A0"]); B0 = np.array(T["B0"])
    A1 = np.array(T["A1"]); B1 = np.array(T["B1"]); B2 = np.array(T["B2"])
    s = len(alpha)
    m = len(dWg)
    sqh = np.sqrt(h)

    _dW = three_point(dWg, h)              # (m,)
    eta1 = two_point(dZg[0])
    eta2 = two_point(dZg[1]) if m > 1 else 0.0
    xi = sqh * eta1
    Ikk = (_dW ** 2 / xi - xi) / 2.0       # diagonal iterated integral (m,)

    def Ikl(k, l):
        if k < l:
            return 0.5 * (_dW[l] - eta2 * _dW[l])
        return 0.5 * (_dW[l] + eta2 * _dW[l])  # k > l

    # Stage function evaluations. fH0[i] = f(H0[i]); gHk[k][i] = b^k(Hk[i]).
    H0 = [None] * s
    Hk = [[None] * s for _ in range(m)]
    fH0 = [None] * s
    gHk = [[None] * s for _ in range(m)]

    H0[0] = X.copy()
    for k in range(m):
        Hk[k][0] = X.copy()
    fH0[0] = a(X)
    b0 = bcols(X)
    for k in range(m):
        gHk[k][0] = b0[k]

    for i in range(1, s):
        # H0[i] = X + h sum_j A0[i,j] f(H0[j]) + sum_k Ihat_k sum_j B0[i,j] b^k(Hk[j])
        acc = X.copy()
        for j in range(i):
            acc = acc + h * A0[i, j] * fH0[j]
        for k in range(m):
            col = np.zeros_like(X)
            for j in range(i):
                col = col + B0[i, j] * gHk[k][j]
            acc = acc + _dW[k] * col
        H0[i] = acc
        fH0[i] = a(H0[i])

        # Hk[i] = X + h sum_j A1[i,j] f(H0[j]) + xi sum_j B1[i,j] b^k(Hk[j])
        #            + sum_{l!=k} Ihat_(k,l) sum_j B2[i,j] b^l(Hl[j])
        for k in range(m):
            acc = X.copy()
            for j in range(i):
                acc = acc + h * A1[i, j] * fH0[j]
            colk = np.zeros_like(X)
            for j in range(i):
                colk = colk + B1[i, j] * gHk[k][j]
            acc = acc + xi * colk
            for l in range(m):
                if l == k:
                    continue
                coll = np.zeros_like(X)
                for j in range(i):
                    coll = coll + B2[i, j] * gHk[l][j]
                acc = acc + Ikl(k, l) * coll
            Hk[k][i] = acc
            gHk[k][i] = bcols(Hk[k][i])[k]

    # State update
    u = X.copy()
    for i in range(s):
        u = u + h * alpha[i] * fH0[i]
    for k in range(m):
        colb0 = np.zeros_like(X)
        colb1 = np.zeros_like(X)
        for i in range(s):
            colb0 = colb0 + beta0[i] * gHk[k][i]
            colb1 = colb1 + beta1[i] * gHk[k][i]
        u = u + _dW[k] * colb0 + Ikk[k] * colb1
    return u


# ---- Test problems -----------------------------------------------------------------
# Non-commutative linear Ito problem (matches the paper test wiring in the Basilisk test):
#   dX = A X dt + B0 X dW0 + B1 X dW1,  [B0,B1] != 0.
A_MAT = np.array([[-0.5, 0.2], [0.1, -0.4]])
B0_MAT = np.array([[0.3, 0.0], [0.0, 0.1]])
B1_MAT = np.array([[0.0, 0.15], [0.25, 0.0]])


def a_noncomm(X):
    return A_MAT @ X


def bcols_noncomm(X):
    return [B0_MAT @ X, B1_MAT @ X]


# Diagonal 2D linear Ito problem: dX_i = a_i X_i dt + b_i X_i dW_i.
A_DIAG = np.array([1.01, -0.5])
B_DIAG = np.array([0.3, 0.2])


def a_diag(X):
    return A_DIAG * X


def bcols_diag(X):
    return [np.array([B_DIAG[0] * X[0], 0.0]), np.array([0.0, B_DIAG[1] * X[1]])]


# Scalar linear Ito problem (geometric Brownian motion): dX = a X dt + b X dW.
def a_scalar(X):
    return np.array([1.01 * X[0]])


def bcols_scalar(X):
    return [np.array([0.87 * X[0]])]


PROBLEMS = {
    "noncommutative2d": dict(a=a_noncomm, bcols=bcols_noncomm, u0=[0.5, 1.0], n=2, m=2),
    "diag2d": dict(a=a_diag, bcols=bcols_diag, u0=[0.5, 1.2], n=2, m=2),
    "scalar": dict(a=a_scalar, bcols=bcols_scalar, u0=[0.5], n=1, m=1),
}


def generate():
    dt = 1 / 8
    tf = 1.0
    ts = np.arange(0.0, tf + dt / 2, dt)
    nsteps = len(ts) - 1
    cases = []
    seed = 71
    for method in ["W2Ito1", "W2Ito2"]:
        for probname, prob in PROBLEMS.items():
            m = prob["m"]
            n = prob["n"]
            rng = np.random.default_rng(seed)
            seed += 1
            # dZ needs at least 2 entries per step (xi and eta2 use dZ[0], dZ[1]).
            mZ = max(m, 2)
            dW = np.sqrt(dt) * rng.standard_normal((nsteps, m))
            dZ = np.sqrt(dt) * rng.standard_normal((nsteps, mZ))
            X = np.array(prob["u0"], dtype=float)
            traj = [X.tolist()]
            for step in range(nsteps):
                X = w2ito_step(X, prob["a"], prob["bcols"], dW[step], dZ[step], dt,
                               TABLEAUX[method])
                traj.append(X.tolist())
            cases.append(dict(
                method=method, problem=f"{method.lower()}_{probname}", dt=dt, tf=tf,
                n_states=n, m=m, n_steps=nsteps, u0=prob["u0"],
                dW=[dW[i].tolist() for i in range(nsteps)],
                dZ=[dZ[i].tolist() for i in range(nsteps)],
                times=ts.tolist(), trajectory=traj))
            print(f"generated {method} {probname}, final={traj[-1]}")
    out = dict(generator="paper-faithful Tang & Xiao W2Ito1/W2Ito2", cases=cases)
    path = os.path.join(os.path.dirname(__file__), "w2ito_reference_trajectories.json")
    with open(path, "w") as f:
        json.dump(out, f)
        f.write("\n")  # trailing newline for the end-of-file-fixer pre-commit hook
    print(f"Wrote {len(cases)} cases to {path}")


def validate(M=2_000_000):
    import scipy.linalg as sla
    T = 1.0
    x0 = np.array([0.5, 1.0])
    exact = sla.expm(A_MAT * T) @ x0  # E[X_T] for a linear Ito SDE
    print(f"[B0,B1] max = {np.max(np.abs(B0_MAT @ B1_MAT - B1_MAT @ B0_MAT)):.3f} (non-commutative)")
    print(f"exact E[X_T] = {exact}")
    for method in ["W2Ito1", "W2Ito2"]:
        print(f"-- {method} --")
        prev = None
        for ns in [1, 2, 4, 8]:
            h = T / ns
            rng = np.random.default_rng(7)
            acc = np.zeros(2)
            for _ in range(M):
                X = x0.copy()
                for _ in range(ns):
                    dW = np.sqrt(h) * rng.standard_normal(2)
                    dZ = np.sqrt(h) * rng.standard_normal(2)
                    X = w2ito_step(X, a_noncomm, bcols_noncomm, dW, dZ, h, TABLEAUX[method])
                acc += X
            mean = acc / M
            err = abs(mean[0] - exact[0])
            rate = "" if prev is None else f"  rate={np.log2(prev / err):.2f}"
            print(f"  ns={ns} h={h:.4f} weak_err(E[X1])={err:.3e}{rate}")
            prev = err


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "validate":
        validate()
    else:
        generate()
