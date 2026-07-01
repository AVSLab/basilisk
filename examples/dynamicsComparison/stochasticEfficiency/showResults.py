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
r"""
Print a compact, human-readable summary of the study result from pareto_<moment>.json.

Text only (no plotting), so it works anywhere the JSON is readable -- including a
login node -- and can't be tripped by a matplotlib/backend issue. Usage:

    python showResults.py                 # std, from $STOCHEFF_RESULTS_DIR
    python showResults.py --moment mean
    python showResults.py --json /path/to/pareto_std.json
"""
from __future__ import annotations

import argparse
import json
import os


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--moment", choices=["std", "mean"], default="std")
    p.add_argument("--json", default=None, help="explicit path to a pareto_*.json")
    args = p.parse_args(argv)

    resultsDir = os.environ.get(
        "STOCHEFF_RESULTS_DIR",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
    path = args.json or os.path.join(resultsDir, f"pareto_{args.moment}.json")
    with open(path) as f:
        d = json.load(f)

    r = d.get("reference") or {}
    print(f"=== {os.path.basename(path)} ===")
    if r:
        print(f"REFERENCE {r.get('key','?')}: {args.moment}(FoM) = "
              f"{r['moment']:.3f} +/- {r['momentSE']:.4f} m   (N={r['nSamples']:,})")
    print()
    hdr = f"{'config':26s} {'arm':8s} {'|bias| m':>9s} {'95% CI [lo, hi]':>22s} {'method':>10s} {'~0?':>4s}"
    print(hdr)
    print("-" * len(hdr))
    for c in sorted(d["configs"], key=lambda x: (x["arm"], x["method"], -x["dt"])):
        ci = f"[{c['biasLo']:.3f}, {c['biasHi']:.3f}]"
        z = "yes" if c.get("biasConsistentZero") else ""
        print(f"{c['key']:26s} {c['arm']:8s} {c['bias']:9.3f} {ci:>22s} "
              f"{c.get('biasMethod',''):>10s} {z:>4s}")

    xo = d.get("crossover")
    if xo and xo.get("rows"):
        print("\nStochastic-vs-profile speedup at matched total-error target:")
        for row in xo["rows"]:
            print(f"  err<={row['targetError']:.3g} m: "
                  f"wall_sde={row['wallSde']:.3g}s  wall_profile={row['wallProfile']:.3g}s  "
                  f"speedup={row['speedup']:.2f}x")

    fronts = d.get("fronts", {})
    for arm in ("sde", "profile"):
        fr = fronts.get(arm) or []
        if fr:
            best = min(fr, key=lambda pt: pt["error"])
            print(f"\n{arm} frontier: {len(fr)} points; "
                  f"best error {best['error']:.3f} m at wall {best['wall']:.3g} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
