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

# =============================================================================
# Reference trajectory generator for the native Basilisk stochastic integrators.
#
# This script uses StochasticDiffEq.jl (SciML) as the ground-truth reference. It solves a
# set of toy SDEs with each integrator under a prescribed sequence of Wiener increments
# (a NoiseGrid, so the increments are deterministic inputs rather than RNG-stream matched
# across languages) and writes the increments plus the resulting trajectory to a JSON.
#
# The Basilisk unit test (test_stochasticIntegratorsJulia.py) loads that JSON, replays the
# identical increments through the corresponding native integrator, and checks the
# trajectories agree to floating-point tolerance. Because the increments are identical,
# agreement demonstrates the Basilisk implementation reproduces the reference algorithm
# exactly (including its convergence order).
#
# SINGLE SOURCE OF TRUTH FOR THE SDE: each problem is described by a coefficient spec
# (drift + per-source diffusion, see ProblemSpec). Both this generator's f/g and the
# Python test's f/g are built from that spec by a generic evaluator, so the SDE is defined
# once (here, and serialized into the JSON) rather than mirrored by hand in Python.
#
# This script is NOT run in CI. It is checked in with the JSON it produces. To regenerate:
#
#   cd this_directory
#   julia --project=. -e 'using Pkg; Pkg.instantiate()'   # first time only
#   julia --project=. generate_reference.jl
#
# =============================================================================

using StochasticDiffEq
using DiffEqNoiseProcess
using Random

# ----------------------------------------------------------------------------
# Minimal JSON writer (avoids a JSON package dependency; we only emit numbers,
# strings, arrays and objects). Floats are written with full round-trip precision.
# ----------------------------------------------------------------------------
jsonval(x::AbstractString) = "\"" * x * "\""
jsonval(x::Bool) = x ? "true" : "false"
jsonval(x::Integer) = string(x)
jsonval(x::AbstractFloat) = begin
    if isnan(x) || isinf(x)
        error("Cannot serialize non-finite value")
    end
    # 17 significant digits round-trips a Float64 exactly.
    repr(x)
end
jsonval(x::AbstractVector) = "[" * join((jsonval(v) for v in x), ",") * "]"
function jsonval(x::AbstractDict)
    parts = String[]
    for (k, v) in x
        push!(parts, jsonval(string(k)) * ":" * jsonval(v))
    end
    return "{" * join(parts, ",") * "}"
end

# ----------------------------------------------------------------------------
# Time factors. A tiny fixed vocabulary shared verbatim with the Python side
# (see _referenceSDE.py TIME_FACTORS). Coefficients are multiplied by the named
# factor evaluated at the stage time t.
# ----------------------------------------------------------------------------
timefactor(name::String, t::Float64) =
    name == "one"                ? 1.0 :
    name == "1+t"                ? 1.0 + t :
    name == "1+0.5t"             ? 1.0 + 0.5 * t :
    name == "0.05_over_sqrt_1pt" ? 0.1 * 0.05 / sqrt(1.0 + t) :
    error("unknown time factor $name")

# ----------------------------------------------------------------------------
# Problem coefficient spec (the single source of truth for each SDE).
#
#   drift_kind:   "linear"        -> f = driftFactor(t) .* (A * x)
#                 "ou_additive_td"-> f = 0.05/sqrt(1+t) - x ./ (2*(1+t))   (special OU)
#   A:            n x n drift matrix (used by "linear")
#   drift_tf:     drift time-factor name
#   diff_kind:    "state_linear"  -> column k is B[k] * x
#                 "additive"      -> column k is the constant vector c[k]
#   B:            for state_linear, a vector of m (n x n) matrices; for additive,
#                 a vector of m length-n constant vectors
#   diff_tf:      diffusion time-factor name
# ----------------------------------------------------------------------------
struct ProblemSpec
    name::String
    u0::Vector{Float64}
    dt::Float64
    tf::Float64
    drift_kind::String
    A::Matrix{Float64}
    drift_tf::String
    diff_kind::String
    B::Vector             # Vector{Matrix} (state_linear) or Vector{Vector} (additive)
    diff_tf::String
    diagonal_flag::Bool   # true -> m == n and source k drives only state k
end

nstates(p::ProblemSpec) = length(p.u0)
nsources(p::ProblemSpec) = length(p.B)

# Build the out-of-place drift f(u,p,t) from the spec.
function make_f(p::ProblemSpec)
    if p.drift_kind == "linear"
        A = p.A; tf = p.drift_tf
        return (u, _p, t) -> timefactor(tf, t) .* (A * u)
    elseif p.drift_kind == "ou_additive_td"
        return (u, _p, t) -> [0.05 / sqrt(1.0 + t) - u[1] / (2.0 * (1.0 + t))]
    else
        error("unknown drift kind $(p.drift_kind)")
    end
end

# Build the out-of-place diffusion g(u,p,t). Returns a length-n vector for diagonal noise
# (m == n and each column drives its own state), otherwise an n x m matrix.
function make_g(p::ProblemSpec)
    n = nstates(p); m = nsources(p); tf = p.diff_tf
    diagonal = is_diagonal(p)
    if p.diff_kind == "state_linear"
        B = p.B
        return (u, _p, t) -> begin
            s = timefactor(tf, t)
            if diagonal
                return [s * (B[k] * u)[k] for k in 1:m]
            else
                M = zeros(n, m)
                for k in 1:m
                    M[:, k] = s * (B[k] * u)
                end
                return M
            end
        end
    elseif p.diff_kind == "additive"
        B = p.B
        return (u, _p, t) -> begin
            s = timefactor(tf, t)
            if diagonal
                return [s * B[k][k] for k in 1:m]
            else
                M = zeros(n, m)
                for k in 1:m
                    M[:, k] = s .* B[k]
                end
                return M
            end
        end
    else
        error("unknown diffusion kind $(p.diff_kind)")
    end
end

# A problem is "diagonal noise" when m == n and each source k drives only state k. The
# basic problems are diagonal; the advanced coupled/scalar ones are not.
is_diagonal(p::ProblemSpec) = p.diagonal_flag

# Serialize the spec into the JSON case dict (the schema the Python side reads).
function spec_json(p::ProblemSpec)
    Bser = p.diff_kind == "state_linear" ?
        [ [collect(row) for row in eachrow(p.B[k])] for k in 1:length(p.B) ] :
        [ collect(p.B[k]) for k in 1:length(p.B) ]
    return Dict{String,Any}(
        "drift" => Dict{String,Any}(
            "kind" => p.drift_kind,
            "matrix" => [collect(row) for row in eachrow(p.A)],
            "timeFactor" => p.drift_tf,
        ),
        "diffusion" => Dict{String,Any}(
            "kind" => p.diff_kind,
            "columns" => Bser,
            "timeFactor" => p.diff_tf,
        ),
    )
end

# ----------------------------------------------------------------------------
# Helper constructors for the two diffusion shapes.
# ----------------------------------------------------------------------------
# Build an n x n diagonal matrix from a vector.
dmat(v::Vector{Float64}) = Matrix{Float64}([i == j ? v[i] : 0.0 for i in 1:length(v), j in 1:length(v)])

const DT = 1 / 8
const TF = 1.0

# ----------------------------------------------------------------------------
# BASIC problems (diagonal / scalar / additive noise). Paired against method
# families in the basic loop below. m == n; each source drives its own state.
# ----------------------------------------------------------------------------

# 1) scalar linear (geometric Brownian motion):  du = a u dt + b u dW
prob_linear = ProblemSpec("linear", [0.5], DT, TF,
    "linear", reshape([1.01], 1, 1), "one",
    "state_linear", Any[reshape([0.87], 1, 1)], "one", true)

# 2) 2D diagonal linear system:  du_i = a_i u_i dt + b_i u_i dW_i
prob_2dlinear = ProblemSpec("diagonal2d", [0.5, 1.2], DT, TF,
    "linear", dmat([1.01, -0.73]), "one",
    "state_linear", Any[dmat([0.87, 0.0]), dmat([0.0, 0.35])], "one", true)

# 3) scalar additive noise (Ornstein-Uhlenbeck):  du = -theta u dt + sigma dW
prob_additive = ProblemSpec("additive", [1.0], DT, TF,
    "linear", reshape([-0.5], 1, 1), "one",
    "additive", Any[[0.3]], "one", true)

# 4) 2D additive system with constant diffusion:  du_i = -theta_i u_i dt + sigma_i dW_i
prob_additive2d = ProblemSpec("additive2d", [0.4, -0.6], DT, TF,
    "linear", dmat([-0.5, -0.2]), "one",
    "additive", Any[[0.3, 0.0], [0.0, 0.15]], "one", true)

const BASIC_PROBLEMS = [prob_linear, prob_2dlinear, prob_additive, prob_additive2d]

# A method is "additive-only" (SRA family) or "diagonal" (SRI/EulerHeun/RKMil/weak).
# EulerHeun and RS use the Stratonovich interpretation; the others are Ito.
const BASIC_METHODS = [
    ("SRIW1",     SRIW1(),     :diagonal),
    ("SOSRI",     SOSRI(),     :diagonal),
    ("SRA1",      SRA1(),      :additive),
    ("SOSRA",     SOSRA(),     :additive),
    ("EulerHeun", EulerHeun(), :diagonal),
    ("RKMil",     RKMil(),     :diagonal),
    ("SIEA",      SIEA(),      :diagonal),
    ("SMEA",      SMEA(),      :diagonal),
    ("SIEB",      SIEB(),      :diagonal),
    ("SMEB",      SMEB(),      :diagonal),
    ("RDI1WM",    RDI1WM(),    :diagonal),
    ("DRI1",      DRI1(),      :diagonal),
    ("DRI1NM",    DRI1NM(),    :diagonal),
    ("RI1",       RI1(),       :diagonal),
    ("RI3",       RI3(),       :diagonal),
    ("RI5",       RI5(),       :diagonal),
    ("RI6",       RI6(),       :diagonal),
    ("RS1",       RS1(),       :diagonal),
    ("RS2",       RS2(),       :diagonal),
    ("W2Ito1",    W2Ito1(),    :diagonal),
]

# ----------------------------------------------------------------------------
# ADVANCED cases (one method per case): non-diagonal / coupled noise, m > 1
# diagonal noise, and time-dependent coefficients. Each carries its own spec.
#
# Time-dependent cases use only rational stage nodes: the Basilisk MuJoCo harness
# quantizes the evaluation time to integer nanoseconds, which would inject a ~1e-10
# error into a time-dependent coefficient evaluated at an irrational stage node.
# ----------------------------------------------------------------------------
struct AdvCase
    method::String
    alg::Any
    spec::ProblemSpec
end

# Full 2x2 coupled diffusion used by several coupled cases: b^1 = [0.2 x1; 0.15 x1],
# b^2 = [0.1 x2; 0.25 x2]. As state_linear columns B[1]*x, B[2]*x:
coupled_B() = Any[[0.2 0.0; 0.15 0.0], [0.0 0.1; 0.0 0.25]]

const ADV_CASES = [
    # ----- non-diagonal / coupled noise -----
    AdvCase("EulerHeun", EulerHeun(), ProblemSpec("eulerheun_nondiag", [0.5, 1.0], DT, TF,
        "linear", dmat([-0.5, -0.3]), "one",
        "state_linear", coupled_B(), "one", false)),
    # scalar noise driving 2 states (single source, d x 1): b^1 = [0.2 x1; 0.15 x2]
    AdvCase("EulerHeun", EulerHeun(), ProblemSpec("eulerheun_scalar", [0.5, 1.0], DT, TF,
        "linear", dmat([-0.5, -0.3]), "one",
        "state_linear", Any[[0.2 0.0; 0.0 0.15]], "one", false)),
    # SOSRA / SRA1 non-diagonal ADDITIVE: constant 2x2 diffusion as two const columns.
    AdvCase("SOSRA", SOSRA(), ProblemSpec("sosra_nondiag_additive", [0.4, -0.6], DT, TF,
        "linear", dmat([-0.5, -0.2]), "one",
        "additive", Any[[0.3, 0.05], [0.1, 0.25]], "one", false)),
    AdvCase("SRA1", SRA1(), ProblemSpec("sra1_nondiag_additive", [0.4, -0.6], DT, TF,
        "linear", dmat([-0.5, -0.2]), "one",
        "additive", Any[[0.3, 0.05], [0.1, 0.25]], "one", false)),
    # ----- time-dependent coefficients (rational stage nodes only) -----
    AdvCase("SRIW1", SRIW1(), ProblemSpec("sriw1_timedep", [0.5], DT, TF,
        "linear", reshape([-0.5], 1, 1), "1+t",
        "state_linear", Any[reshape([0.3], 1, 1)], "1+0.5t", true)),
    AdvCase("RKMil", RKMil(), ProblemSpec("rkmil_timedep", [0.5], DT, TF,
        "linear", reshape([-0.5], 1, 1), "1+t",
        "state_linear", Any[reshape([0.3], 1, 1)], "1+0.5t", true)),
    AdvCase("EulerHeun", EulerHeun(), ProblemSpec("eulerheun_timedep", [0.5], DT, TF,
        "linear", reshape([-0.5], 1, 1), "1+t",
        "state_linear", Any[reshape([0.3], 1, 1)], "1+0.5t", true)),
    AdvCase("SRA1", SRA1(), ProblemSpec("sra1_timedep_additive", [1.0], DT, TF,
        "ou_additive_td", reshape([0.0], 1, 1), "one",
        "additive", Any[[1.0]], "0.05_over_sqrt_1pt", true)),
    # ----- W2Ito1 m = 2 -----
    AdvCase("W2Ito1", W2Ito1(), ProblemSpec("w2ito1_diag2d", [0.5, 1.0], DT, TF,
        "linear", dmat([1.01, -0.5]), "one",
        "state_linear", Any[dmat([0.3, 0.0]), dmat([0.0, 0.2])], "one", true)),
    AdvCase("W2Ito1", W2Ito1(), ProblemSpec("w2ito1_nondiag", [0.5, 1.0], DT, TF,
        "linear", dmat([-0.5, -0.3]), "one",
        "state_linear", coupled_B(), "one", false)),
]

# ----------------------------------------------------------------------------
# Solve one (spec, method) combination with prescribed noise; return the JSON case.
# `nsrc` is the number of noise sources m (== n for diagonal problems).
# ----------------------------------------------------------------------------
function run_case(method_name::String, alg, spec::ProblemSpec; seed::Int)
    n = nstates(spec)
    m = is_diagonal(spec) ? n : nsources(spec)
    ts = collect(0.0:spec.dt:spec.tf)
    nsteps = length(ts) - 1

    rng = MersenneTwister(seed)
    dW = sqrt(spec.dt) .* randn(rng, nsteps, m)
    dZ = sqrt(spec.dt) .* randn(rng, nsteps, m)

    Wcum = vcat(zeros(1, m), cumsum(dW, dims = 1))
    Zcum = vcat(zeros(1, m), cumsum(dZ, dims = 1))
    Wgrid = [collect(Wcum[i, :]) for i in 1:(nsteps + 1)]
    Zgrid = [collect(Zcum[i, :]) for i in 1:(nsteps + 1)]
    NG = NoiseGrid(ts, Wgrid, Zgrid)

    f = make_f(spec)
    g = make_g(spec)
    # W2Ito1's in-place path throws for a single scalar source, so it uses the
    # out-of-place SDEProblem; every other method uses the universally-supported
    # in-place form. Non-diagonal noise needs a noise_rate_prototype (n x m matrix).
    if method_name == "W2Ito1" && is_diagonal(spec)
        sdeprob = SDEProblem(f, g, copy(spec.u0), (0.0, spec.tf), noise = NG)
    else
        f! = (du, u, p, t) -> (du .= f(u, p, t))
        g! = (du, u, p, t) -> (du .= g(u, p, t))
        if is_diagonal(spec)
            sdeprob = SDEProblem(f!, g!, copy(spec.u0), (0.0, spec.tf), noise = NG)
        else
            sdeprob = SDEProblem(f!, g!, copy(spec.u0), (0.0, spec.tf), noise = NG,
                                 noise_rate_prototype = zeros(n, m))
        end
    end
    sol = solve(sdeprob, alg; dt = spec.dt, adaptive = false, save_noise = false,
                saveat = ts)

    traj = [collect(sol.u[i]) for i in 1:length(sol.u)]

    case = Dict{String,Any}(
        "method" => method_name,
        "problem" => spec.name,
        "dt" => spec.dt,
        "tf" => spec.tf,
        "n_states" => n,
        "m" => m,
        "n_steps" => nsteps,
        "seed" => seed,
        "u0" => spec.u0,
        "diagonal" => is_diagonal(spec),
        "dW" => [collect(dW[i, :]) for i in 1:nsteps],
        "dZ" => [collect(dZ[i, :]) for i in 1:nsteps],
        "times" => collect(ts),
        "trajectory" => traj,
    )
    merge!(case, spec_json(spec))
    return case
end

function main()
    cases = Dict{String,Any}[]

    # --- basic method x problem pairings (seed base 20260706) ---
    seed = 20260706
    for prob in BASIC_PROBLEMS
        for (method_name, alg, method_noise) in BASIC_METHODS
            s = seed
            seed += 1
            # Keep the pairing physically sensible: additive problems -> SRA methods,
            # diagonal problems -> everything else.
            probIsAdditive = prob.diff_kind == "additive"
            if method_noise == :additive && !probIsAdditive
                continue
            end
            if method_noise == :diagonal && probIsAdditive
                continue
            end
            # RS1/RS2 and W2Ito1 are only valid Julia references for a single noise
            # source here (m > 1 RS aliases in StochasticDiffEq; W2Ito1 in-place needs
            # m = 1). Their m > 1 behaviour is covered by the paper/advanced suites.
            if (method_name in ("RS1", "RS2", "W2Ito1")) && length(prob.u0) != 1
                continue
            end
            result = run_case(method_name, alg, prob; seed = s)
            push!(cases, result)
            println("generated: ", method_name, " / ", prob.name,
                    "  u_final=", result["trajectory"][end])
        end
    end

    # --- advanced one-method-per-case (seed base 90210) ---
    seed = 90210
    for c in ADV_CASES
        result = run_case(c.method, c.alg, c.spec; seed = seed)
        seed += 1
        push!(cases, result)
        println("generated: ", c.method, " / ", c.spec.name,
                "  u_final=", result["trajectory"][end])
    end

    out = Dict{String,Any}(
        "generator" => "StochasticDiffEq.jl",
        "stochasticdiffeq_version" => string(pkgversion(StochasticDiffEq)),
        "cases" => cases,
    )
    outpath = joinpath(@__DIR__, "reference_trajectories.json")
    open(outpath, "w") do io
        write(io, jsonval(out))
    end
    println("\nWrote ", length(cases), " cases to ", outpath)
end

main()
