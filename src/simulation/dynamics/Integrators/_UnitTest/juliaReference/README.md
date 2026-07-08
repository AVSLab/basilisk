# Julia reference trajectories for the native stochastic integrators

This directory holds the ground-truth reference used by
`../test_stochasticIntegratorsJulia.py` to check that the native Basilisk stochastic
integrators reproduce the corresponding
[StochasticDiffEq.jl](https://github.com/SciML/StochasticDiffEq.jl) (SciML) algorithms
exactly. It covers every integrator with a StochasticDiffEq.jl counterpart (SRIW1, SOSRI,
SRA1, SOSRA, EulerHeun, RKMil, the SIEA/SMEA/SIEB/SMEB family, RDI1WM, DRI1/DRI1NM, the
RI1/RI3/RI5/RI6 family, RS1/RS2, and W2Ito1) across scalar, diagonal, additive,
non-diagonal (coupled), and time-dependent noise structures.

## What is here

- `generate_reference.jl` — the Julia script that produces the reference data.
- `Project.toml` / `Manifest.toml` — the (pinned) Julia environment for the generator.
- `reference_trajectories.json` — the generated reference, committed so the test can run
  without Julia. **This is the file the Python test reads.**

## How the comparison works

Matching a Julia RNG stream in C++ is not feasible, so instead of trying to draw the
*same* random numbers in both languages we make the random numbers an **input**:

1. `generate_reference.jl` draws a fixed sequence of Wiener increments `dW` (and, for the
   Roessler SRI/SRA and weak methods, a second independent increment `dZ`).
2. It hands those increments to the solver via a `NoiseGrid`, so the solve is a
   deterministic function of the prescribed increments (bit-reproducible).
3. It solves each toy SDE with each method at a fixed step and records the increments and
   the resulting trajectory into `reference_trajectories.json`.

The Python test loads that JSON, installs a `PrescribedGaussianNoiseGenerator` on the
Basilisk integrator preloaded with the identical `dW`/`dZ`, runs the simulation, and
asserts the trajectory matches to floating-point tolerance. Because both integrators
consume the same increments, agreement demonstrates that the Basilisk implementation
reproduces the reference algorithm exactly — and therefore its convergence order.

## The SDE is defined once

Each problem is described by a **coefficient spec** (a `drift` matrix + time factor and a
per-source `diffusion` spec — see `ProblemSpec` in `generate_reference.jl`). The generator
builds its `f`/`g` from that spec *and* serializes the spec into each JSON case. The Python
test rebuilds `f`/`g` from the serialized spec via `../referenceSDE.py`, so the SDE lives
in exactly one place. Adding or changing a problem is a spec edit in the generator only —
there is no matching Python drift/diffusion to keep in sync.

### A note on the time-dependent test problems

Most problems use time-independent coefficients, but a few exercise explicit `t`
dependence. Those use only **rational** stage nodes (e.g. `dt/2`, `dt`): the Basilisk
MuJoCo harness quantizes the evaluation time to integer nanoseconds, so a coefficient
evaluated at an *irrational* Runge-Kutta stage node (e.g. `t + 0.692*dt`) would pick up a
~1e-10 error relative to Julia's exact `Float64` stage time. Restricting the
time-dependent cases to rational-node methods removes that confound and keeps the
equivalence assertion at floating-point tolerance.

## Regenerating the reference

The reference is **not** regenerated in CI. Regenerate it locally after changing an
integrator or adding a problem:

```bash
cd this_directory
julia --project=. -e 'using Pkg; Pkg.instantiate()'   # first time only
julia --project=. generate_reference.jl
```

This was last generated with Julia 1.10 and StochasticDiffEq.jl v6 (the exact version is
recorded in the `stochasticdiffeq_version` field of the JSON).
