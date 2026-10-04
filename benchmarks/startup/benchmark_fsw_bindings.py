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

"""Measure native binding groups in an existing Ninja build.

The full benchmark removes owned generated outputs and touches one source at a
time. It restores source timestamps and rebuilds removed outputs before exiting.
``--smoke`` only imports one existing binding and does not modify the build.
"""

import argparse
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import tempfile
import time


REPOSITORY = Path(__file__).resolve().parents[2]
MTIME_MARGIN_SECONDS = 1.1  # [s]
NATIVE_SUFFIXES = {".so", ".pyd", ".dll", ".dylib"}
GROUPS = {
    "fsw": ("fswCoreNative", "fswCoreBindings.txt", "fswAlgorithms", "FswObjects",
            "fswAlgorithms/attControl/mrpFeedback/mrpFeedback.c"),
    "simulation": ("simulationCoreNative", "simulationCoreNativeBindings.txt",
                   "simulation", "GroupedObjects", "simulation/dynamics/spacecraft/spacecraft.cpp"),
    "mujoco": ("mujocoNative", "mujocoNativeBindings.txt", "simulation", "GroupedObjects",
               "simulation/mujocoDynamics/thrOnTimeToForce/thrOnTimeToForce.cpp"),
    "opnav": ("opNavNative", "opNavNativeBindings.txt", "", "GroupedObjects",
              "simulation/sensors/camera/camera.cpp"),
}
IMPORT_PROBE = """
import importlib
import json
from pathlib import Path
import sys
import time
sys.path.insert(0, sys.argv[1])
import Basilisk
assert Path(Basilisk.__file__).resolve().is_relative_to(Path(sys.argv[1]).resolve())
started = time.perf_counter()
for name in json.loads(sys.argv[2]):
    importlib.import_module('Basilisk.' + name)
print(json.dumps({'seconds': time.perf_counter() - started,
                  'python_version': sys.version}))
"""


def read_cache(build_dir):
    """Read CMake cache values without depending on the caller's environment."""
    values = {}
    for line in (build_dir / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if separator and ":" in key and not key.startswith(("//", "#")):
            values[key.split(":", 1)[0]] = value
    return values


def remove_outputs(build_dir, paths):
    """Remove an explicit list of generated files confined to the build directory."""
    paths = list(paths)
    for path in paths:
        if not path.resolve().is_relative_to(build_dir.resolve()):
            raise ValueError(f"Refusing to remove an output outside the build directory: {path}")
    for path in paths:
        path.unlink(missing_ok=True)


def linker_outputs(build_dir, target_listing):
    """Select project native outputs from Ninja linker rules, excluding import libraries."""
    outputs = set()
    for line in target_listing.splitlines():
        match = re.fullmatch(
            r"(Basilisk/[^:]+): (?:C|CXX)_(?:MODULE|SHARED)_LIBRARY_LINKER.*", line
        )
        if match:
            path = build_dir / match[1]
            if path.suffix in NATIVE_SUFFIXES:
                outputs.add(path)
    return sorted(outputs)


class Benchmark:
    """Manage sequential timing runs and restoration of an existing build."""

    def __init__(self, args):
        self.args = args
        self.group = getattr(args, "group", "fsw")
        self.native_target, self.manifest_name, package, self.object_suffix, source = GROUPS[self.group]
        self.representative_source = Path(source)
        self.representative_module = source.split("/")[0] + "." + Path(source).stem
        self.build_dir = args.build_dir.resolve()
        self.cache = read_cache(self.build_dir)
        self.source_dir = Path(self.cache["CMAKE_HOME_DIRECTORY"]).resolve()
        if self.source_dir != REPOSITORY / "src":
            raise ValueError("The build directory must belong to this Basilisk checkout.")
        if not args.smoke and self.cache["CMAKE_GENERATOR"] != "Ninja":
            raise ValueError("Build measurements require the single-configuration Ninja generator.")
        self.python = self.cache.get("Python3_EXECUTABLE", self.cache.get("_Python3_EXECUTABLE", sys.executable))
        self.cmake = self.cache["CMAKE_COMMAND"]
        self.restore_all_targets = False
        self.package_dir = self.build_dir / "Basilisk" / package
        self.names = []
        self.proxies = {}
        self.mode = "combined"
        self.results = {
            "schema_version": 2,
            "group": self.group,
            "platform": platform.platform(),
            "machine": platform.machine(),
            "parallel": args.parallel,
            "layout": self.mode,
            "cache_settings": {key: self.cache.get(key) for key in (
                "CMAKE_GENERATOR", "CMAKE_BUILD_TYPE", "PY_LIMITED_API", "BSK_STRICT_WARNINGS",
                "BUILD_OPNAV", "BUILD_MUJOCO", "BUILD_VIZINTERFACE", "BUILD_RUST_MODULES",
            )},
            "measurements": [],
        }
        self.output = None
        if not args.smoke:
            if args.output:
                self.output = args.output.resolve()
                self.output.mkdir(parents=True, exist_ok=False)
            else:
                parent = self.build_dir / "benchmarks"
                parent.mkdir(exist_ok=True)
                self.output = Path(tempfile.mkdtemp(prefix=f"{self.group}-startup-", dir=parent))

    def record(self, label, **values):
        """Append a measurement and save partial results after each completed step."""
        row = {"layout": self.mode, "label": label, **values}
        self.results["measurements"].append(row)
        print(json.dumps({key: value for key, value in row.items() if key != "profiles"}), flush=True)
        self.save()
        return row

    def save(self):
        """Write results using portable JSON, including partial runs on failure."""
        if self.output is not None:
            (self.output / "results.json").write_text(
                json.dumps(self.results, indent=2) + "\n", encoding="utf-8"
            )

    def run_logged(self, label, command, **kwargs):
        """Run a command with a separate log and expose failures to the caller."""
        log_path = self.output / f"{self.mode}-{label}.log"
        with log_path.open("w", encoding="utf-8") as log:
            result = subprocess.run(command, cwd=REPOSITORY, stdout=log,
                                    stderr=subprocess.STDOUT, **kwargs)
        if result.returncode:
            raise RuntimeError(f"Command exited {result.returncode}; see {log_path}")
        return log_path

    def configure(self):
        """Refresh group manifests without changing configured build options."""
        self.run_logged("configure", [self.cmake, "-S", str(self.source_dir),
                        "-B", str(self.build_dir)])

    def build(self, label, all_targets=False):
        """Time a build, retaining its log and compilation, SWIG, and link counts."""
        command = [self.cmake, "--build", str(self.build_dir),
                   "--parallel", str(self.args.parallel)]
        if not all_targets:
            command.extend(["--target", self.native_target])
        started = time.perf_counter()
        log = self.run_logged(label, command)
        elapsed = time.perf_counter() - started
        content = log.read_text(encoding="utf-8")
        return self.record(label, seconds=elapsed,
                           compiles=content.count("Building C object")
                           + content.count("Building CXX object"),
                           swig=content.count("Swig compile"), links=content.count("Linking "))

    def native_outputs(self):
        """Return only the native library owned by the selected group."""
        suffix = ".pyd" if os.name == "nt" else ".so"
        return [self.package_dir / f"_{self.native_target}{suffix}"]

    def public_path(self, name):
        """Locate a public proxy, including groups spanning multiple packages."""
        if "." not in name:
            name = "fswAlgorithms." + name
        return self.build_dir.joinpath("Basilisk", *name.split("."))

    def objects(self):
        """Find active Ninja object outputs, ignoring stale files from older builds."""
        listing = subprocess.check_output(
            [self.cache["CMAKE_MAKE_PROGRAM"], "-C", str(self.build_dir), "-t", "targets", "all"],
            text=True,
        )
        targets = {name.split(".")[-1] + self.object_suffix
                   for name in self.names}
        objects = []
        for line in listing.splitlines():
            match = re.fullmatch(r"(CMakeFiles/([^/]+)\.dir/[^:]+): (?:C|CXX)_COMPILER.*", line)
            if match and match[2] in targets:
                objects.append(self.build_dir / match[1])
        return objects

    def import_pair(self, label, names):
        """Measure consecutive fresh processes, isolating group imports after Basilisk."""
        names = [name if "." in name else "fswAlgorithms." + name for name in names]
        names = [self.proxies.get(name, name) for name in names]
        for temperature in ("cold", "warm"):
            result = subprocess.run(
                [self.python, "-c", IMPORT_PROBE, str(self.build_dir), json.dumps(names)],
                cwd=REPOSITORY, capture_output=True, text=True, check=True,
            )
            self.record(f"{label}-{temperature}", **json.loads(result.stdout))

    def relink(self, label, all_targets=False):
        """Reset generated native files and require a build without C/C++ or SWIG work."""
        paths = self.native_outputs()
        if all_targets:
            listing = subprocess.check_output(
                [self.cache["CMAKE_MAKE_PROGRAM"], "-C", str(self.build_dir), "-t", "targets", "all"],
                text=True,
            )
            paths = linker_outputs(self.build_dir, listing)
            if not paths:
                raise RuntimeError("No project native linker outputs were found.")
            # A failure or interruption after deletion must also recover native
            # libraries outside FSW, including simulation and optional modules.
            self.restore_all_targets = True
        remove_outputs(self.build_dir, paths)
        row = self.build(label, all_targets)
        if all_targets:
            self.restore_all_targets = False
        if row["compiles"] or row["swig"]:
            raise RuntimeError(f"Unexpected compilation during a link-only measurement: {row}")

    def incremental(self, suffix, trial):
        """Touch one representative input and restore its timestamps even after failure."""
        representative = getattr(self, "representative_source", Path("fswAlgorithms/attControl/mrpFeedback/mrpFeedback.c"))
        source = self.source_dir / representative.with_suffix("." + suffix)
        original = source.stat()
        newest = max(path.stat().st_mtime for path in self.native_outputs())
        time.sleep(max(0.0, newest - time.time()) + MTIME_MARGIN_SECONDS)
        try:
            source.touch()
            row = self.build(f"change-{suffix}-{trial}")
            if row["compiles"] != 1 or row["swig"] != int(suffix == "i") or row["links"] != 1:
                raise RuntimeError(f"Unexpected work after a single-module edit: {row}")
        finally:
            os.utime(source, ns=(original.st_atime_ns, original.st_mtime_ns))

    def collection(self, temperature):
        """Collect the CI-selected suite in workers without executing its tests."""
        output = self.output / f"{self.mode}-pytest-{temperature}"
        output.mkdir()
        env = os.environ.copy()
        python_paths = [str(self.build_dir), str(Path(__file__).parent), env.get("PYTHONPATH")]
        env.update(
            MPLBACKEND="Agg", BSK_STARTUP_OUTPUT=str(output),
            BSK_STARTUP_STARTED=str(time.time()),
            PYTHONPATH=os.pathsep.join(filter(None, python_paths)),
        )
        command = [self.python, "-m", "pytest", "-n", str(self.args.pytest_workers),
                   "-m", "not ciSkip", "-rs", "-q", "-p", "_collection_profile", str(self.source_dir)]
        with (output / "pytest.log").open("w", encoding="utf-8") as log:
            result = subprocess.run(command, cwd=REPOSITORY, env=env,
                                    stdout=log, stderr=subprocess.STDOUT)
        profiles = [json.loads(path.read_text(encoding="utf-8"))
                    for path in output.glob("worker-*.json")]
        ready = [profile["ready_seconds"] for profile in profiles
                 if profile["ready_seconds"] is not None]
        expected_profiles = self.args.pytest_workers or 1
        if result.returncode not in (0, 5) or len(ready) != expected_profiles:
            raise RuntimeError(f"Collection failed; see {output / 'pytest.log'}")
        self.record(f"pytest-{temperature}", seconds=max(ready),
                    workers=self.args.pytest_workers, profiles=profiles)

    def measure_group(self):
        """Measure compilation, loading, incremental edits, and optional collection."""
        self.build("settle")
        expected_objects = len(self.objects())
        for trial in range(self.args.trials):
            outputs = self.objects() + self.native_outputs()
            outputs.extend(self.public_path(self.proxies.get(name, name)).with_suffix(".py")
                           for name in self.names)
            outputs.extend(self.public_path(name).with_name(name.split(".")[-1] + "PYTHON_wrap.cxx")
                           for name in self.names)
            remove_outputs(self.build_dir, outputs)
            row = self.build(f"rebuild-{trial}")
            if row["compiles"] != expected_objects or row["swig"] != len(self.names):
                raise RuntimeError(f"Unexpected work during a full binding-group rebuild: {row}")
            self.import_pair(f"all-{self.group}-{trial}", self.names)
        self.relink("single-import-relink")
        self.import_pair(f"one-{self.group}", [self.representative_module])
        for trial in range(self.args.trials):
            self.relink(f"relink-{trial}")
            for suffix in (self.representative_source.suffix[1:], "i"):
                self.incremental(suffix, trial)
        row = self.build("no-change")
        if row["compiles"] or row["swig"] or row["links"]:
            raise RuntimeError(f"An unchanged build performed unexpected work: {row}")
        self.record("native-size", bytes=sum(path.stat().st_size for path in self.native_outputs()))
        if self.args.pytest_workers is not None:
            self.build("prepare-collection", all_targets=True)
            self.relink("all-native-relink", all_targets=True)
            self.collection("cold")
            self.collection("warm")

    def run(self):
        """Measure the selected group and recover removed outputs in all cases."""
        if self.args.smoke:
            result = subprocess.run(
                [self.python, "-B", "-c", IMPORT_PROBE, str(self.build_dir), json.dumps([self.representative_module])],
                capture_output=True, text=True, check=True,
            )
            print(result.stdout.strip())
            return
        try:
            self.configure()
            manifest = self.build_dir / "autoSource" / self.manifest_name
            self.names = sorted(manifest.read_text(encoding="utf-8").split())
            proxy_manifest = self.build_dir / "autoSource" / f"{self.native_target}Proxies.txt"
            if self.group != "fsw":
                self.proxies = dict(line.split("=", 1) for line in proxy_manifest.read_text().splitlines() if line)
            representative = self.representative_module if self.group != "fsw" else "mrpFeedback"
            if representative not in self.names or any(
                    re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*(?:\.[A-Za-z_][A-Za-z_0-9]*)?", name) is None for name in self.names):
                raise ValueError("The binding manifest is empty or invalid; enable the selected group's build feature.")
            self.results["modules"] = self.names
            self.measure_group()
        except BaseException as error:
            self.results["error"] = str(error)
            raise
        finally:
            self.results["restored_build"] = False
            try:
                if self.names or self.restore_all_targets:
                    self.build("restore", all_targets=self.restore_all_targets)
                self.results["restored_build"] = True
            finally:
                self.save()
                print(f"Results: {self.output / 'results.json'}", flush=True)


def main():
    """Parse portable paths and benchmark controls, then measure the selected group."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--group", choices=GROUPS, default="fsw")
    parser.add_argument("--build-dir", type=Path, default=REPOSITORY / "dist3")
    parser.add_argument("--output", type=Path, help="New output directory; defaults under the build tree")
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--parallel", type=int, default=min(os.cpu_count() or 1, 12))
    parser.add_argument("--pytest-workers", type=int, help="Also measure CI collection with this worker count")
    parser.add_argument("--smoke", action="store_true", help="Only import one existing binding; do not rebuild")
    args = parser.parse_args()
    if args.trials < 1 or args.parallel < 1 or (args.pytest_workers is not None and args.pytest_workers < 0):
        parser.error("Trials and parallel jobs must be positive; pytest workers must be nonnegative.")
    Benchmark(args).run()


if __name__ == "__main__":
    main()
