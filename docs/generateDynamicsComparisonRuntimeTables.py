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
#

"""Generate optional local dynamics-comparison runtime tables."""

import os
import subprocess
import sys


repoRoot = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
buildRoot = os.path.join(repoRoot, "dist3")
runner = os.path.join(
    repoRoot, "examples", "dynamicsComparison", "runAllComparisons.py"
)


def _gitOutput(*args):
    """Run one read-only Git query against the source tree."""
    return subprocess.run(
        ("git", "-C", repoRoot, *args),
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _loadBuildInfo():
    """Load build metadata from this checkout's default build directory."""
    if buildRoot not in sys.path:
        sys.path.insert(0, buildRoot)
    from Basilisk import getBuildInfo
    return getBuildInfo()


def validateBuild(buildInfo=None):
    """Require a clean Release build configured from the current source commit."""
    buildInfo = _loadBuildInfo() if buildInfo is None else buildInfo
    head = _gitOutput("rev-parse", "HEAD")
    artifact = buildInfo["artifact"]
    configuration = buildInfo["diagnostics"]["build"]["configuration"]
    if artifact["sourceRevision"] != head or artifact["sourceDirty"]:
        raise RuntimeError(
            "The documentation runtime tables require a build configured from "
            "the clean current commit; reconfigure and rebuild Basilisk."
        )
    if _gitOutput("status", "--porcelain", "--untracked-files=no"):
        raise RuntimeError(
            "The documentation runtime tables require a clean tracked worktree."
        )
    if configuration != "Release":
        raise RuntimeError(
            "The documentation runtime tables require a Release build, not "
            f"{configuration!r}."
        )
    if not buildInfo["features"]["mujoco"]:
        raise RuntimeError(
            "The documentation runtime tables require a MuJoCo-enabled build."
        )
    return buildInfo


def runtimeEnvironment():
    """Return the environment used to launch the comparison runner."""
    environment = os.environ.copy()
    environment.setdefault("MPLBACKEND", "agg")
    pythonPaths = [buildRoot]
    if environment.get("PYTHONPATH"):
        pythonPaths.append(environment["PYTHONPATH"])
    environment["PYTHONPATH"] = os.pathsep.join(pythonPaths)
    return environment


def main():
    """Validate the build and regenerate every embedded timing table."""
    validateBuild()
    subprocess.run(
        [sys.executable, runner, "--timing-only"],
        check=True,
        cwd=repoRoot,
        env=runtimeEnvironment(),
    )


if __name__ == "__main__":
    main()
