# Paper-faithful references for multi-noise / non-commutative integrators

This directory holds ground-truth references used by
`../test_stochasticIntegratorsPaper.py` to check native Basilisk weak-order-2
integrators with **multiple** (m > 1), possibly **non-commutative**, noise sources:

- `generate_rs_reference.py` → RS1 / RS2 (Roessler 2007, Stratonovich).
- `generate_dri1_reference.py` → DRI1 (Debrabant–Roessler, Ito), coupled noise.

Both target integrators whose multi-noise path StochasticDiffEq.jl cannot reference
(RS: an aliasing defect in the in-place cache; DRI1: its non-diagonal branch raises an
`UndefVarError`). The sections below describe RS in detail; DRI1 is analogous, with its
oracle validated by weak-order-2 convergence on a non-commutative linear Ito SDE whose
mean is known exactly (`E[X_T] = expm(A·T)·X0`); run
`python3 generate_dri1_reference.py validate`.

## Why this is separate from the Julia reference

The other integrators are checked against StochasticDiffEq.jl (`../juliaReference`).
RS is the exception for m > 1: StochasticDiffEq.jl's in-place RS cache builds its
cross-noise stage states with `H22[k] = uprev` / `H23[k] = uprev`, which **aliases**
(rather than copies) the working state and then mutates it in place. For more than one
noise source this makes the library's trajectory deviate from the method as published,
so it is not a valid reference. (For a single noise source the cross-terms vanish and
StochasticDiffEq.jl agrees with the paper; that case is covered by the Julia test.)

We therefore generate the reference directly from the primary source:

> A. Rößler, *Second Order Runge–Kutta Methods for Stratonovich Stochastic
> Differential Equations*, BIT Numer. Math. 47 (2007), 657–680.

`generate_rs_reference.py` implements eq. (5.1)–(5.2) with the Table 5.2 (RS1) and
Table 5.3 (RS2) coefficients, using vector-valued diffusion columns `b^l` (so genuine
non-commutative noise is handled correctly).

## Validation of the reference itself

Run `python3 generate_rs_reference.py validate` to reproduce Rößler's own published
weak-error tables by Monte Carlo:

- **Commutative SDE (6.1)**, RS1: matches Table 6.1 (`8.57e-2, 3.56e-2, 1.18e-2,
  3.40e-3`) to ~3 significant figures, with the errors quartering per halving of `h`
  (weak order 2).
- **Non-commutative SDE (6.2)**, RS1: matches Table 6.3 to ~1–2 significant figures and
  shows weak order ≈ 2 until the systematic error drops below the Monte-Carlo noise
  floor (the paper used 5×10⁷ samples; the quick validation here uses far fewer).

This is what justifies treating the generator as ground truth.

## How the equivalence test works

Like the Julia generator, this **prescribes** the underlying Gaussian increments
`(dW, dZ)` and applies exactly the transforms the Basilisk integrator uses
(`stochasticWeakRandomVariables`: three-point for `Î`, two-point for `Ĩ`). The Basilisk
test installs a `PrescribedGaussianNoiseGenerator` with the identical increments and
requires the trajectory to match to `atol=1e-11`. Because the increments are identical,
agreement is an exact algorithmic-equivalence check (the Basilisk RS1/RS2 match this
oracle to machine precision, ~1e-17, on the non-commutative problem).

## Regenerating

```bash
cd this_directory
python3 generate_rs_reference.py           # writes data/rs_reference_trajectories.json
python3 generate_rs_reference.py validate  # optional: reproduce the paper's tables
```

Only NumPy is required (no Julia). If you change a problem definition here, keep the
matching drift/diffusion functions in `../test_stochasticIntegratorsRSpaper.py` in sync.
