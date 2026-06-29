# Running the stochastic-efficiency study on CU Boulder Alpine

This is a **precision-targeted, staged, resumable** pipeline for the
[Alpine](https://curc.readthedocs.io/en/latest/clusters/alpine/) HPC cluster
(Slurm). It guarantees the two accuracy requirements the study needs:

1. **The reference is precise enough to resolve every integrator's bias.** Its
   sample count is computed so its Monte-Carlo error is below the smallest bias
   we choose to resolve (it is the *shared* anchor for every comparison).
2. **The Monte-Carlo error on each config's bias is ≤ `REL_TARGET` (default 10%)
   of that bias** — so the discretization bias is actually *resolved*, not buried
   in sampling noise. Each config gets its **own** target sample count.

Both fall out of `planBudget.py`, which reads a cheap pilot and emits a
per-config plan (target `N` + shard counts, including a **sharded reference**).

## TL;DR

```bash
cd <basilisk>/examples/dynamicsComparison/stochasticEfficiency/slurm
nano env.sh        # set STOCHEFF_ACCOUNT and BASILISK_ROOT  (one time)
nano config.sh     # set REL_TARGET / NEGLIGIBLE_FRAC / PER_TASK_SECONDS (optional)

./run_study.sh --dry-run   # show the chain of submissions
./run_study.sh             # pilot -> plan -> production -> analyze (auto-chained)

squeue --me                # watch
./watch_analyze.sh 300     # OPTIONAL: refresh figures every 5 min as data lands
```

When the final analyze job finishes, the Pareto / bias / cost JSON and SVGs are
in `$STOCHEFF_RESULTS_DIR` (default `/scratch/alpine/$USER/stocheff/results`).

## The five stages (auto-chained by `run_study.sh`)

| stage | script | Slurm shape | purpose |
|-------|--------|-------------|---------|
| 1. pilot | `submit_pilot.sbatch` + `submit_pilot_ref.sbatch` | array + 1 job | cheap uniform sweep to **measure** each bias and per-run cost |
| 2. plan | `submit_plan.sbatch` | 1 job (`afterok` pilot) | `planBudget.py` → per-config target `N` + shards → `plan.json` |
| 3. launch | `launch_production.sbatch` | 1 job (`afterok` plan) | reads the plan, submits the production array sized to it + a final analyze |
| 4. production | `submit_production.sbatch` | array | the real run: per-config `N`, **reference sharded** |
| 5. analyze | `submit_analyze.sbatch` | 1 job (`afterok` production) | final figures + JSON |

Stage 3 exists because a Slurm `--array` size can't reference a file (the plan)
that doesn't exist at submit time; the launch job runs *after* the plan and
submits the correctly-sized production array itself.

## Why precision drives the cost (read this before tuning)

For the std estimand, the bootstrap SE scales as `SE ≈ σ/√(2N)` (σ ≈ the FoM
spread, ≈146 m here). The bias is `σ̂_cfg − σ̂_ref`, so its error is
`√(SE_cfg² + SE_ref²)`. To get `SE_bias ≤ REL_TARGET·|bias|`, planBudget gives
each config `N ≈ σ²/(REL_TARGET·|bias|)²` and the reference enough to match the
**smallest** resolved bias. Consequences, all handled automatically:

- **Per-config N spans ~100 → ~millions.** Huge-bias configs (coarse `dt`,
  Euler-Maruyama) need almost nothing; the near-converged weak-SRK configs at the
  smallest `dt` dominate. A uniform N would be absurd — hence the plan.
- **The reference needs ~10⁶ samples** to resolve a ~1 m bias, so it is
  **sharded** across array tasks (the earlier single-job reference could not fit
  the walltime).
- **Profile arm is cheap** regardless of bias: it shares the OU path with the
  reference (common random numbers), so a few thousand samples pin its bias.
- **Negligible biases are upper-bounded, not resolved.** A bias below
  `NEGLIGIBLE_FRAC·σ` (default 0.5%, ≈0.7 m) can't change any ranking and would
  cost millions of samples to pin to 10%; planBudget instead sizes those configs
  just to *demonstrate* "bias < floor" (far cheaper) and labels them
  `upper-bound`. Lower `NEGLIGIBLE_FRAC` to chase smaller biases at higher cost.

Representative plan for the default scenario (`REL_TARGET=0.10`,
`PER_TASK_SECONDS=36000`): reference ≈1.2 M samples in ~6 shards, ~51 array
tasks, **≈205 core-hours total (~3 h on 64 cores)** — dominated by the two
W2Ito1/W2Ito2 `dt=1` configs (biases ≈1.3–1.7 m). Inspect your own plan before
launching production:

```bash
# after the pilot+plan stages (or locally on a pilot cache):
python ../planBudget.py --relTarget 0.10 --perTaskSeconds 36000 --out /tmp/plan.json
```

It prints the full per-config N / shards / core-hours table.

## Analyse as results stream in (re-runnable, anytime)

The analysis reads **partial** results — `sampleStore.loadMerged` concatenates
the consolidated file and every shard and de-duplicates by seed — so it never
needs the run to be finished:

- **Live, on a login node:** `./watch_analyze.sh 300` re-plots every 300 s.
- **As a job, anytime:** `sbatch --account=… --qos=normal --export=ALL --time=01:00:00 submit_analyze.sbatch`.

The headline accuracy figure is `stochasticEfficiency_biasPareto.svg` (bias vs
per-run cost); `stochasticEfficiency_pareto.svg` is the sampling-budget view.

## Resuming / re-running

Everything is resumable at **shard granularity**: each shard skips seeds it
already has. So after a timeout, preemption, or to add samples:

- re-submit the same stage, or just `./run_study.sh` again (pilot/plan stages
  are cheap and idempotent), or
- `./run_study.sh --from-plan` to (re)launch production from an existing
  `plan.json` (e.g. after you hand-edit target N's or lower `NEGLIGIBLE_FRAC`).

To push precision further later, edit `plan.json` (raise some `nSamples`) and
`--from-plan`; only the missing samples are computed.

## Prerequisites & Alpine notes

- **Build Basilisk on Alpine** under `/projects/$USER` (on the `acompile`
  partition, never the login node), as a `.venv` with MuJoCo (default) or a
  conda env. Set `STOCHEFF_PYMODE` in `env.sh` accordingly.
- **Account:** `sacctmgr -p show assoc user=$USER format=account` → put it in
  `env.sh` as `STOCHEFF_ACCOUNT`.
- **Storage:** results default to `/scratch/alpine/$USER` (large/fast, **purged
  ~90 days**); copy to `/projects/$USER` to keep. Set `STOCHEFF_RESULTS_DIR` to
  change.
- **QOS:** `normal` (24 h, ≤1000 jobs) is plenty since per-task walltime is
  bounded by `PER_TASK_SECONDS`. Use `QOS=long` only if you raise
  `PER_TASK_SECONDS` past ~20 h.
- Single-threaded per task (`OMP_NUM_THREADS=1` etc.); parallelism is across
  array tasks. `NUMBA_CACHE_DIR` is set so the profile arm's JIT compiles once.
- Override any knob from the shell, e.g.
  `REL_TARGET=0.05 NEGLIGIBLE_FRAC=0.002 ./run_study.sh`.
