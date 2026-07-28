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

r"""
Print-quality figures for the dynamics-engine comparison paper.

The scenario scripts emit documentation figures sized for web pages; this script
re-renders the figures placed in the paper from the stored ``results/*.json``
metrics, with fonts and legends sized for two-column print inclusion and one
consistent color scheme (Paul Tol high-contrast: BSM blue, MuJoCo red, third
series yellow).

Run after ``runAllComparisons.py`` and the sweep drivers. The input hashes in
``results/provenance.json`` are checked before any figure is rendered::

    python3 paperFigures.py [outputDir]

To regenerate every publication input, render the print figures, and replace
the manifest with hashes from one clean, matching Release build, run::

    python3 paperFigures.py --update-provenance [outputDir]

The update command reruns every scenario and sweep that supplies a paper figure,
table, or quantitative claim. It also regenerates the deep-space full-slosh and
near-rigid controls; no pre-existing result file is certified.

``outputDir`` defaults to ``results/paper`` next to this script.
"""

import argparse
import glob
import hashlib
import importlib
import importlib.util
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

thisFolder = os.path.dirname(os.path.abspath(__file__))
resultsPath = os.path.join(thisFolder, "results")
repoRoot = os.path.abspath(os.path.join(thisFolder, "..", ".."))
BUILD_RECEIPT_ENV = "BSK_DYNAMICS_COMPARISON_BUILD_RECEIPT"

PAPER_RENDER_INPUT_FILES = (
    "scenarioCompareParetoRwPanels.json",
    "scenarioCompareParetoFlexPanels.json",
    "scenarioCompareFlexPanels.json",
    "sweepOrbitDt.json",
    "sweepTorqueArtifact.json",
)
PAPER_INPUT_FILES = PAPER_RENDER_INPUT_FILES
PAPER_EVIDENCE_FILES = (
    "scenarioCompareOrbit.json",
    "scenarioCompareTorque.json",
    "scenarioCompareRwPanels.json",
    "scenarioCompareFlexPanels.json",
    "scenarioCompareOrbitMultibody.json",
    "scenarioCompareVariableMass.json",
    "scenarioCompareVariableMass_deepSpace.json",
    "scenarioCompareVariableMass_deepSpace_rigid.json",
    "scenarioCompareParetoRwPanels.json",
    "scenarioCompareParetoFlexPanels.json",
    "sweepOrbitDt.json",
    "sweepRwBookkeepingMass.json",
    "sweepSloshDisplacement.json",
    "sweepTorqueArtifact.json",
    "sweepTorqueMechanismChecks.json",
    "sweepVariableMassPendulumInertia.json",
    "scenarioCompareOrbit_runtime.csv",
    "scenarioCompareTorque_runtime.csv",
    "scenarioCompareRwPanels_runtime.csv",
    "scenarioCompareOrbitMultibody_runtime.csv",
    "scenarioCompareVariableMass_runtime.csv",
    "scenarioCompareRwPanels_attError.svg",
    "scenarioCompareOrbitMultibody_attError.svg",
    "scenarioCompareOrbitMultibody_regimes.svg",
    "scenarioCompareVariableMass_deepSpace_attError.svg",
)
PAPER_GENERATOR_FILES = (
    "examples/dynamicsComparison/_comparePlots.py",
    "examples/dynamicsComparison/_comparisonValidation.py",
    "examples/dynamicsComparison/_runtimeTable.py",
    "examples/dynamicsComparison/paperFigures.py",
    "examples/dynamicsComparison/runAllComparisons.py",
    "examples/dynamicsComparison/scenarioCompareFlexPanels.py",
    "examples/dynamicsComparison/scenarioCompareOrbit.py",
    "examples/dynamicsComparison/scenarioCompareOrbitMultibody.py",
    "examples/dynamicsComparison/scenarioCompareParetoFlexPanels.py",
    "examples/dynamicsComparison/scenarioCompareParetoRwPanels.py",
    "examples/dynamicsComparison/scenarioCompareRwPanels.py",
    "examples/dynamicsComparison/scenarioCompareTorque.py",
    "examples/dynamicsComparison/scenarioCompareVariableMass.py",
    "examples/dynamicsComparison/sweepOrbitDt.py",
    "examples/dynamicsComparison/sweepRwBookkeepingMass.py",
    "examples/dynamicsComparison/sweepSloshDisplacement.py",
    "examples/dynamicsComparison/sweepTorqueArtifact.py",
    "examples/dynamicsComparison/sweepTorqueMechanismChecks.py",
    "examples/dynamicsComparison/sweepVariableMassPendulumInertia.py",
)
PAPER_SCENARIOS = (
    "scenarioCompareParetoRwPanels",
    "scenarioCompareParetoFlexPanels",
    "scenarioCompareFlexPanels",
)
PAPER_SWEEPS = (
    "sweepOrbitDt",
    "sweepTorqueArtifact",
)
PUBLICATION_SWEEPS = (
    "sweepOrbitDt",
    "sweepRwBookkeepingMass",
    "sweepSloshDisplacement",
    "sweepTorqueArtifact",
    "sweepTorqueMechanismChecks",
    "sweepVariableMassPendulumInertia",
)
PAPER_OUTPUT_FILES = (
    "scenarioCompareParetoRwPanels_frontier.pdf",
    "scenarioCompareParetoRwPanels_frontierPosition.pdf",
    "scenarioCompareParetoFlexPanels_frontier.pdf",
    "scenarioCompareParetoFlexPanels_frontierPosition.pdf",
    "scenarioCompareFlexPanels_runtime.pdf",
    "sweepOrbitDt.pdf",
    "sweepTorqueArtifact.pdf",
)
PDF_METADATA = {
    "Creator": "Basilisk dynamics-engine comparison",
    "CreationDate": None,
    "ModDate": None,
}

BLUE = "#004488"    # BSM
RED = "#BB5566"     # MuJoCo
YELLOW = "#DDAA33"  # third series (spinningBodyNDOF, at-rest cross-engine, ...)
GRAY = "#666666"

STYLE = {
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 9,
    "legend.fontsize": 7.0,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "lines.linewidth": 1.3,
    "lines.markersize": 4.0,
    "legend.framealpha": 0.9,
    "figure.constrained_layout.use": True,
}

# Marker per integrator family, shared by both Pareto panels and stated in the
# paper's figure caption -- keep the two in sync.
INTEGRATOR_MARKERS = (
    ("Euler", "v"),
    ("RK2", "s"),
    ("RK4", "o"),
    ("RKF45", "D"),
    ("RKF78", "^"),
)


def _resultsDirectory(path=None):
    """Return an explicit results directory or the scenario default."""
    return path or resultsPath


def loadJson(name, resultsDir=None):
    with open(os.path.join(_resultsDirectory(resultsDir), name + ".json")) as f:
        return json.load(f)


def sha256File(path):
    """Return the hexadecimal SHA-256 digest of one artifact."""
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _gitOutput(*args):
    """Run a read-only Git query against the Basilisk source tree."""
    return subprocess.run(
        ("git", "-C", repoRoot, *args),
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _basiliskPackageRoot():
    """Locate the configured Basilisk package without importing native modules."""
    specification = importlib.util.find_spec("Basilisk")
    locations = (
        None if specification is None else specification.submodule_search_locations
    )
    if not locations:
        raise RuntimeError(
            "Cannot locate the configured Basilisk package; put its build "
            "directory on PYTHONPATH."
        )
    return os.path.abspath(next(iter(locations)))


def _buildRoot():
    """Return the CMake build directory containing the Basilisk package."""
    return os.path.dirname(_basiliskPackageRoot())


def _getBuildInfo():
    """Load build metadata only after any requested clean rebuild is complete."""
    from Basilisk import getBuildInfo
    return getBuildInfo()


def _matchingSourceRevision(requireClean):
    """Return HEAD and build metadata after validating the publication build."""
    head = _gitOutput("rev-parse", "HEAD")
    buildInfo = _getBuildInfo()
    artifact = buildInfo["artifact"]
    configuration = buildInfo["diagnostics"]["build"]["configuration"]
    if artifact["sourceRevision"] != head or artifact["sourceDirty"]:
        raise RuntimeError(
            "The Basilisk package was not configured from the clean current "
            "commit; reconfigure and rebuild it before publishing figures."
        )
    if requireClean and _gitOutput("status", "--porcelain", "--untracked-files=no"):
        raise RuntimeError(
            "Tracked source changes are present; commit them and rebuild before "
            "refreshing publication provenance."
        )
    if configuration != "Release":
        raise RuntimeError(
            "Publication artifacts require a Release build, not "
            f"{configuration!r}."
        )
    if not buildInfo["features"]["mujoco"]:
        raise RuntimeError(
            "Publication artifacts require a MuJoCo-enabled Basilisk build."
        )
    return head, buildInfo


def _nativeBuildFingerprint(packageRoot=None):
    """Hash every native extension in the imported Basilisk package."""
    packageRoot = packageRoot or _basiliskPackageRoot()
    nativeFiles = []
    for root, _, files in os.walk(packageRoot):
        for name in files:
            if name.endswith((".so", ".dylib", ".pyd")):
                nativeFiles.append(os.path.join(root, name))
    nativeFiles.sort()
    if not nativeFiles:
        raise RuntimeError(
            f"No native Basilisk extensions were found under {packageRoot}."
        )

    digest = hashlib.sha256()
    for path in nativeFiles:
        relativePath = os.path.relpath(path, packageRoot).replace(os.sep, "/")
        digest.update(relativePath.encode("utf-8"))
        digest.update(b"\0")
        with open(path, "rb") as stream:
            for block in iter(lambda: stream.read(1024*1024), b""):
                digest.update(block)
        digest.update(b"\0")
    return {
        "algorithm": "sha256-path-content-v1",
        "fileCount": len(nativeFiles),
        "sha256": digest.hexdigest(),
    }


def _mujocoDependencyFingerprint(buildRoot=None, configuration="Release"):
    """Hash the MuJoCo binary selected by the configured Conan toolchain."""
    buildRoot = buildRoot or _buildRoot()
    generatorRoot = os.path.join(buildRoot, configuration, "generators")
    searchRoot = generatorRoot if os.path.isdir(generatorRoot) else buildRoot
    configFiles = glob.glob(
        os.path.join(searchRoot, "**", "mujoco-config.cmake"),
        recursive=True,
    )
    dataFiles = glob.glob(
        os.path.join(searchRoot, "**", "mujoco-*-data.cmake"),
        recursive=True,
    )
    if not configFiles or not dataFiles:
        raise RuntimeError(
            "Cannot locate the configured MuJoCo package metadata under "
            f"{buildRoot}."
        )

    with open(configFiles[0]) as stream:
        configText = stream.read()
    versionMatch = re.search(
        r'set\(mujoco_VERSION_STRING "([^"]+)"\)', configText)
    packageRoot = None
    for dataFile in dataFiles:
        with open(dataFile) as stream:
            dataText = stream.read()
        packageMatch = re.search(
            r'set\(mujoco_PACKAGE_FOLDER_[^ ]+ "([^"]+)"\)', dataText)
        if packageMatch:
            packageRoot = packageMatch.group(1)
            break
    if versionMatch is None or packageRoot is None:
        raise RuntimeError("Configured MuJoCo package metadata is incomplete.")

    binaryFiles = []
    for subdirectory in ("lib", "bin"):
        root = os.path.join(packageRoot, subdirectory)
        if not os.path.isdir(root):
            continue
        for directory, _, files in os.walk(root):
            for name in files:
                lowerName = name.lower()
                if (
                    "mujoco" in lowerName
                    and lowerName.endswith(
                        (".so", ".dylib", ".dll", ".a", ".lib")
                    )
                ):
                    binaryFiles.append(os.path.join(directory, name))
    binaryFiles = sorted(set(os.path.realpath(path) for path in binaryFiles))
    if not binaryFiles:
        raise RuntimeError(
            f"No MuJoCo binary was found under configured package {packageRoot}."
        )

    digest = hashlib.sha256()
    for path in binaryFiles:
        relativePath = os.path.relpath(path, packageRoot).replace(os.sep, "/")
        digest.update(relativePath.encode("utf-8"))
        digest.update(b"\0")
        with open(path, "rb") as stream:
            for block in iter(lambda: stream.read(1024*1024), b""):
                digest.update(block)
        digest.update(b"\0")
    manifestPath = os.path.join(packageRoot, "conanmanifest.txt")
    return {
        "version": versionMatch.group(1),
        "algorithm": "sha256-path-content-v1",
        "fileCount": len(binaryFiles),
        "sha256": digest.hexdigest(),
        "conanManifestSha256": (
            sha256File(manifestPath) if os.path.isfile(manifestPath) else None
        ),
    }


def _provenanceFile(path, resultsDir=None):
    return path or os.path.join(
        _resultsDirectory(resultsDir), "provenance.json")


def _outputDirectory(path, resultsDir=None):
    return path or os.path.join(_resultsDirectory(resultsDir), "paper")


def _hashFiles(root, names):
    """Hash a required set of files under one artifact root."""
    hashes = {}
    for name in names:
        artifactPath = os.path.join(root, name)
        if not os.path.isfile(artifactPath):
            raise FileNotFoundError(f"Missing publication artifact {artifactPath}.")
        hashes[name] = sha256File(artifactPath)
    return hashes


def _physicalMemoryBytes():
    """Return installed physical memory in bytes when the host exposes it."""
    if hasattr(os, "sysconf"):
        names = getattr(os, "sysconf_names", {})
        if "SC_PHYS_PAGES" in names and "SC_PAGE_SIZE" in names:
            return int(
                os.sysconf("SC_PHYS_PAGES")*os.sysconf("SC_PAGE_SIZE")
            )
    return None


def _cmakeCacheValue(buildRoot, name):
    """Return one value from the existing build tree's CMake cache."""
    cachePath = os.path.join(buildRoot, "CMakeCache.txt")
    if os.path.isfile(cachePath):
        with open(cachePath) as stream:
            for line in stream:
                if line.startswith(name + ":"):
                    value = line.partition("=")[2].strip()
                    if value:
                        return value
    return None


def _cmakeCommand(buildRoot):
    """Return the CMake executable recorded by the existing build tree."""
    command = _cmakeCacheValue(buildRoot, "CMAKE_COMMAND")
    if command:
        return command
    command = shutil.which("cmake")
    if command is None:
        raise RuntimeError("Cannot locate CMake for the publication rebuild.")
    return command


def _cmakeSourceDirectory(buildRoot):
    """Return the source directory recorded by the existing build tree."""
    sourceDirectory = _cmakeCacheValue(buildRoot, "CMAKE_HOME_DIRECTORY")
    if (
        sourceDirectory is None
        or not os.path.isfile(os.path.join(sourceDirectory, "CMakeLists.txt"))
    ):
        raise RuntimeError(
            "Cannot locate the configured CMake source directory from "
            f"{buildRoot}."
        )
    return sourceDirectory


def _hardwareIdentity():
    """Return non-sensitive host details that identify timing hardware."""
    identity = {
        "model": platform.machine(),
        "processor": platform.processor(),
    }
    if platform.system() != "Darwin":
        return identity

    profiler = shutil.which("system_profiler")
    if profiler is None:
        return identity
    try:
        completed = subprocess.run(
            [profiler, "SPHardwareDataType", "-json"],
            check=True,
            capture_output=True,
            text=True,
        )
        hardware = json.loads(completed.stdout)["SPHardwareDataType"][0]
    except (KeyError, IndexError, json.JSONDecodeError, subprocess.SubprocessError):
        return identity

    identity.update({
        "model": " ".join(filter(None, (
            hardware.get("machine_name"),
            (
                f"({hardware['machine_model']})"
                if hardware.get("machine_model")
                else None
            ),
        ))),
        "processor": hardware.get("chip_type") or identity["processor"],
        "processorConfiguration": hardware.get("number_processors"),
        "memory": hardware.get("physical_memory"),
    })
    return identity


def _currentBuildReceipt(head, buildInfo, cleanRebuild):
    """Describe the exact source, Basilisk binaries, and MuJoCo dependency."""
    return {
        "schemaVersion": 1,
        "generatedAt": datetime.now().astimezone().strftime(
            "%Y-%m-%dT%H:%M:%S%z"
        ),
        "cleanRebuild": bool(cleanRebuild),
        "sourceRevision": head,
        "sourceTree": _gitOutput("rev-parse", f"{head}^{{tree}}"),
        "configuration": buildInfo["diagnostics"]["build"]["configuration"],
        "nativeExtensions": _nativeBuildFingerprint(),
        "mujocoDependency": _mujocoDependencyFingerprint(
            configuration=buildInfo["diagnostics"]["build"]["configuration"]
        ),
    }


def _validateBuildReceipt(receipt, head, buildInfo, requireCleanRebuild):
    """Reject a stale, incomplete, or non-clean publication build receipt."""
    expected = _currentBuildReceipt(
        head, buildInfo, cleanRebuild=receipt.get("cleanRebuild", False)
    )
    for key in (
        "schemaVersion",
        "sourceRevision",
        "sourceTree",
        "configuration",
        "nativeExtensions",
        "mujocoDependency",
    ):
        if receipt.get(key) != expected[key]:
            raise RuntimeError(
                f"Publication build receipt field {key!r} does not match "
                "the current build."
            )
    if requireCleanRebuild and receipt.get("cleanRebuild") is not True:
        raise RuntimeError(
            "Publication provenance requires a clean rebuild receipt."
        )
    return receipt


def _writeBuildReceipt(receipt, path):
    """Write one clean-build receipt outside the source tree."""
    with open(path, "w") as stream:
        json.dump(receipt, stream, indent=2)
        stream.write("\n")


def _requiredBuildReceipt(head, buildInfo):
    """Load the receipt passed by the clean-build parent process."""
    path = os.environ.get(BUILD_RECEIPT_ENV)
    if not path or not os.path.isfile(path):
        raise RuntimeError(
            "Run paperFigures.py --update-provenance as a script so it can "
            "perform and record the required clean rebuild."
        )
    with open(path) as stream:
        receipt = json.load(stream)
    return _validateBuildReceipt(
        receipt, head, buildInfo, requireCleanRebuild=True)


def _cleanBuildAndRerun():
    """Reconfigure and clean-rebuild Basilisk, then rerun this command."""
    if _gitOutput("status", "--porcelain", "--untracked-files=no"):
        raise RuntimeError(
            "Commit tracked source changes before preparing publication artifacts."
        )
    buildRoot = _buildRoot()
    cmakeCommand = _cmakeCommand(buildRoot)
    sourceDirectory = _cmakeSourceDirectory(buildRoot)
    subprocess.run(
        [cmakeCommand, "-S", sourceDirectory, "-B", buildRoot],
        check=True,
        cwd=repoRoot,
    )
    buildCommand = [
        cmakeCommand,
        "--build",
        buildRoot,
        "--config",
        "Release",
        "--clean-first",
        "--parallel",
    ]
    subprocess.run(buildCommand, check=True, cwd=repoRoot)
    head, buildInfo = _matchingSourceRevision(requireClean=True)
    receipt = _currentBuildReceipt(head, buildInfo, cleanRebuild=True)
    descriptor, receiptPath = tempfile.mkstemp(
        prefix="dynamicsComparisonBuildReceipt-",
        suffix=".json",
        dir=buildRoot,
    )
    os.close(descriptor)
    try:
        _writeBuildReceipt(receipt, receiptPath)
        environment = os.environ.copy()
        environment[BUILD_RECEIPT_ENV] = receiptPath
        completed = subprocess.run(
            [sys.executable, os.path.abspath(__file__), *sys.argv[1:]],
            cwd=repoRoot,
            env=environment,
        )
        return completed.returncode
    finally:
        if os.path.exists(receiptPath):
            os.remove(receiptPath)


def regeneratePublicationInputs(scenarioRunKwargs=None, sweepRunKwargs=None,
                                resultsDir=None):
    """Regenerate every JSON file consumed by the print renderer.

    Args:
        scenarioRunKwargs (dict, optional): per-scenario keyword overrides.
            Publication provenance uses the full defaults; reduced overrides are
            useful for exercising the rendering pipeline during development.
        sweepRunKwargs (dict, optional): per-sweep keyword overrides with the
            same development-only purpose.
        resultsDir (str, optional): destination for regenerated JSON inputs.
            Defaults to the scenario ``results`` directory.
    """
    targetResults = _resultsDirectory(resultsDir)
    scenarioOverrides = (
        {} if scenarioRunKwargs is None else scenarioRunKwargs
    )
    sweepOverrides = {} if sweepRunKwargs is None else sweepRunKwargs
    os.makedirs(targetResults, exist_ok=True)
    for name in PAPER_INPUT_FILES:
        inputPath = os.path.join(targetResults, name)
        if os.path.exists(inputPath):
            os.remove(inputPath)

    for moduleName in PAPER_SCENARIOS:
        print("Regenerating publication input: " + moduleName)
        module = importlib.import_module(moduleName)
        try:
            kwargs = dict(scenarioOverrides.get(moduleName, {}))
            kwargs.setdefault("showPlots", False)
            kwargs.setdefault("saveJson", True)
            kwargs.setdefault("resultsDir", targetResults)
            module.run(**kwargs)
        finally:
            plt.close("all")

    for moduleName in PAPER_SWEEPS:
        print("Regenerating publication input: " + moduleName)
        module = importlib.import_module(moduleName)
        try:
            kwargs = dict(sweepOverrides.get(moduleName, {}))
            kwargs.setdefault("saveJson", True)
            kwargs.setdefault("resultsDir", targetResults)
            module.run(**kwargs)
        finally:
            plt.close("all")

    _hashFiles(targetResults, PAPER_INPUT_FILES)


def _saveResultFigures(figureList, targetResults, selectedNames=None):
    """Save selected scenario figures into the publication evidence directory."""
    names = set(figureList) if selectedNames is None else set(selectedNames)
    missing = names.difference(figureList)
    if missing:
        raise ValueError(
            "Scenario did not return required publication figures: "
            + ", ".join(sorted(missing))
        )
    os.makedirs(targetResults, exist_ok=True)
    for name in names:
        figureList[name].savefig(
            os.path.join(targetResults, name + ".svg"),
            transparent=True,
        )


def regeneratePublicationEvidence(resultsDir=None):
    """Regenerate every data and scenario-figure artifact used by the paper."""
    targetResults = _resultsDirectory(resultsDir)
    os.makedirs(targetResults, exist_ok=True)
    for name in PAPER_EVIDENCE_FILES:
        artifactPath = os.path.join(targetResults, name)
        if os.path.exists(artifactPath):
            os.remove(artifactPath)

    runner = importlib.import_module("runAllComparisons")
    runner.run(
        resultsDir=targetResults,
        saveDocumentationFigures=False,
    )

    variableMass = importlib.import_module("scenarioCompareVariableMass")
    try:
        deepSpaceFigures = variableMass.run(
            showPlots=False,
            saveJson=True,
            inOrbit=False,
            resultsDir=targetResults,
        )
        _saveResultFigures(
            deepSpaceFigures,
            targetResults,
            ("scenarioCompareVariableMass_deepSpace_attError",),
        )
    finally:
        plt.close("all")

    try:
        variableMass.run(
            showPlots=False,
            saveJson=True,
            inOrbit=False,
            nearRigid=True,
            resultsDir=targetResults,
        )
    finally:
        plt.close("all")

    for moduleName in PUBLICATION_SWEEPS:
        print("Regenerating publication evidence: " + moduleName)
        module = importlib.import_module(moduleName)
        try:
            module.run(saveJson=True, resultsDir=targetResults)
        finally:
            plt.close("all")

    _hashFiles(targetResults, PAPER_EVIDENCE_FILES)


def _publicationProvenance(head, buildInfo, outputDir, buildReceipt,
                           resultsDir=None, dataFiles=PAPER_EVIDENCE_FILES):
    """Build a fresh manifest for the generated data, binaries, and figures."""
    artifact = buildInfo["artifact"]
    diagnostics = buildInfo["diagnostics"]
    compiler = diagnostics["compilers"]["cxx"]
    target = diagnostics["target"]
    dataFiles = tuple(dataFiles)
    return {
        "schemaVersion": 4,
        "generatedAt": datetime.now().astimezone().strftime(
            "%Y-%m-%dT%H:%M:%S%z"
        ),
        "basilisk": {
            "commit": head,
            "sourceTree": _gitOutput("rev-parse", f"{head}^{{tree}}"),
            "trackedWorktreeClean": True,
        },
        "build": {
            "basiliskVersion": artifact["basiliskVersion"],
            "sourceRevision": head,
            "sourceDirty": False,
            "configuration": diagnostics["build"]["configuration"],
            "target": f"{target['system']} {target['processor']}",
            "compiler": f"{compiler['id']} {compiler['version']}",
            "python": diagnostics["tools"]["python"],
            "receipt": buildReceipt,
        },
        "environment": {
            "platform": platform.platform(),
            "hardware": _hardwareIdentity(),
            "logicalCpuCount": os.cpu_count(),
            "physicalMemoryBytes": _physicalMemoryBytes(),
            "python": platform.python_version(),
            "numpy": np.__version__,
            "matplotlib": matplotlib.__version__,
            "timingProtocol": {
                "warmupRuns": 1,
                "measuredRuns": 5,
                "interleavedEngineOrder": True,
                "reportedStatistic": "median",
            },
        },
        "artifacts": {
            "evidenceFiles": list(dataFiles),
            "basiliskDataSha256": _hashFiles(
                _resultsDirectory(resultsDir), dataFiles
            ),
            "basiliskPrintFiguresSha256": _hashFiles(
                outputDir, PAPER_OUTPUT_FILES
            ),
            "generatorSourceSha256": _hashFiles(
                repoRoot, PAPER_GENERATOR_FILES
            ),
        },
    }


def _writeProvenance(provenance, path, resultsDir=None):
    """Replace one provenance file with the supplied complete manifest."""
    provenanceFile = _provenanceFile(path, resultsDir)
    os.makedirs(os.path.dirname(os.path.abspath(provenanceFile)), exist_ok=True)
    with open(provenanceFile, "w") as stream:
        json.dump(provenance, stream, indent=2)
        stream.write("\n")
    return provenanceFile


def updateInputProvenance(path=None, outputDir=None, resultsDir=None):
    """Regenerate publication inputs and replace their complete provenance."""
    head, buildInfo = _matchingSourceRevision(requireClean=True)
    buildReceipt = _requiredBuildReceipt(head, buildInfo)
    regeneratePublicationEvidence(resultsDir=resultsDir)
    outputDir = _outputDirectory(outputDir, resultsDir)
    _renderFiguresInFreshProcess(outputDir, resultsDir)
    provenance = _publicationProvenance(
        head, buildInfo, outputDir, buildReceipt, resultsDir)
    provenanceFile = _writeProvenance(provenance, path, resultsDir)
    print("Publication provenance written to " + provenanceFile)
    return provenance


def validateInputProvenance(path=None, verifySource=True, resultsDir=None):
    """Reject missing, stale, or source-inconsistent publication inputs."""
    targetResults = _resultsDirectory(resultsDir)
    provenanceFile = _provenanceFile(path, resultsDir)
    if not os.path.isfile(provenanceFile):
        raise FileNotFoundError(
            f"Missing {provenanceFile}; regenerate the data and run "
            "paperFigures.py --update-provenance."
        )
    with open(provenanceFile) as stream:
        provenance = json.load(stream)

    if provenance.get("schemaVersion") != 4:
        raise ValueError(
            "Publication provenance uses an obsolete schema; regenerate it with "
            "paperFigures.py --update-provenance."
        )
    revision = provenance.get("basilisk", {}).get("commit")
    build = provenance.get("build", {})
    if (
        not revision
        or build.get("sourceRevision") != revision
        or build.get("sourceDirty") is not False
    ):
        raise ValueError(
            "Publication provenance does not identify one clean Basilisk source revision."
        )
    if verifySource:
        head, buildInfo = _matchingSourceRevision(requireClean=True)
        if revision != head:
            raise ValueError(
                "Publication inputs were generated from a different Basilisk commit."
            )
        sourceTree = _gitOutput("rev-parse", f"{head}^{{tree}}")
        if provenance["basilisk"].get("sourceTree") != sourceTree:
            raise ValueError(
                "Publication provenance has an inconsistent Git source-tree hash."
            )
        try:
            _validateBuildReceipt(
                build.get("receipt", {}),
                head,
                buildInfo,
                requireCleanRebuild=True,
            )
        except RuntimeError as error:
            raise ValueError(str(error)) from error

    artifacts = provenance.get("artifacts", {})
    evidenceFiles = tuple(artifacts.get("evidenceFiles", ()))
    if not set(PAPER_RENDER_INPUT_FILES).issubset(evidenceFiles):
        raise ValueError(
            "Publication provenance does not cover every print-renderer input."
        )
    if verifySource and evidenceFiles != PAPER_EVIDENCE_FILES:
        raise ValueError(
            "Publication provenance does not cover the complete paper evidence set."
        )
    if verifySource:
        expectedGenerators = _hashFiles(repoRoot, PAPER_GENERATOR_FILES)
        if artifacts.get("generatorSourceSha256") != expectedGenerators:
            raise ValueError(
                "Publication generator source hashes do not match the current checkout."
            )

    hashes = artifacts.get("basiliskDataSha256", {})
    for name in evidenceFiles:
        expected = hashes.get(name)
        if not expected:
            raise ValueError(f"Publication provenance has no SHA-256 hash for {name}.")
        inputPath = os.path.join(targetResults, name)
        if not os.path.isfile(inputPath):
            raise FileNotFoundError(f"Missing paper evidence {inputPath}.")
        actual = sha256File(inputPath)
        if actual != expected:
            raise ValueError(
                f"Publication evidence hash mismatch for {name}; regenerate the "
                "provenance before rendering."
            )
    return provenance


def validateFigureProvenance(provenance, outputDir):
    """Reject missing or changed print figures after deterministic rendering."""
    hashes = provenance.get("artifacts", {}).get(
        "basiliskPrintFiguresSha256", {}
    )
    for name in PAPER_OUTPUT_FILES:
        expected = hashes.get(name)
        if not expected:
            raise ValueError(
                f"Publication provenance has no SHA-256 hash for {name}."
            )
        figurePath = os.path.join(outputDir, name)
        if not os.path.isfile(figurePath):
            raise FileNotFoundError(f"Missing paper figure {figurePath}.")
        if sha256File(figurePath) != expected:
            raise ValueError(
                f"Publication figure hash mismatch for {name}; regenerate all "
                "publication artifacts with --update-provenance."
            )


def markerFor(integratorName):
    """Marker for an svIntegrator name; RKF45 checked before RK4 by order below."""
    for key, marker in reversed(INTEGRATOR_MARKERS):
        if key in integratorName:
            return marker
    return "x"


def paretoPanel(data, errorKey, errorLabel, floorKey, outFile):
    """One work-precision panel: error vs wall-clock, both engines, marker per integrator."""
    fig, ax = plt.subplots(figsize=(3.5, 2.7))

    for engine, color in (("bsm", BLUE), ("mujoco", RED)):
        rows = data[engine]
        families = {}
        for row in rows:
            families.setdefault(row["integrator"], []).append(row)
        for integratorName, familyRows in families.items():
            familyRows.sort(key=lambda r: r["wall"])
            adaptive = "RKF" in integratorName
            ax.loglog([r["wall"] for r in familyRows],
                      [max(r[errorKey], 1e-18) for r in familyRows],
                      linestyle="-" if adaptive else "--",
                      marker=markerFor(integratorName), color=color,
                      markerfacecolor=color if adaptive else "white",
                      markeredgecolor=color, linewidth=1.0)

    ax.axhline(data[floorKey], color=GRAY, linestyle=":", linewidth=1.0)
    ax.text(0.98, data[floorKey]*1.6, "reference floor", color=GRAY, fontsize=6.5,
            ha="right", transform=ax.get_yaxis_transform())

    # Compact custom legend: engine colors + integrator markers.
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], color=BLUE, label="BSM"),
               Line2D([], [], color=RED, label="MuJoCo")]
    present = {r["integrator"] for r in data["bsm"]}
    for key, marker in INTEGRATOR_MARKERS:
        if any(key in name for name in present):
            handles.append(Line2D([], [], color=GRAY, linestyle="none",
                                  marker=marker, label=key))
    ax.legend(handles=handles, ncol=2, loc="best", handlelength=1.4,
              columnspacing=0.8, borderpad=0.4)

    ax.set_xlabel("wall-clock time [s]")
    ax.set_ylabel(errorLabel)
    ax.grid(True, which="both", alpha=0.2)
    fig.savefig(outFile, metadata=PDF_METADATA)
    plt.close(fig)


def paretoFigures(outputDir, resultsDir=None):
    for study, jsonName in (("RwPanels", "scenarioCompareParetoRwPanels"),
                            ("FlexPanels", "scenarioCompareParetoFlexPanels")):
        data = loadJson(jsonName, resultsDir)
        paretoPanel(data, "error", "final attitude error [rad]",
                    "sharedReferenceThreshold",
                    os.path.join(outputDir, jsonName + "_frontier.pdf"))
        paretoPanel(data, "positionError", "final position error [m]",
                    "sharedPositionReferenceThreshold",
                    os.path.join(outputDir, jsonName + "_frontierPosition.pdf"))


def scalingFigure(outputDir, resultsDir=None):
    """Wall-clock vs DOF for the three implementations, with BSM/MuJoCo ratio labels."""
    rows = loadJson("scenarioCompareFlexPanels", resultsDir)["rows"]
    dof = np.array([r["dof"] for r in rows])
    bsm = np.array([r["bsmWall"] for r in rows])
    ndof = np.array([r["ndofWall"] for r in rows])
    mujoco = np.array([r["mujocoWall"] for r in rows])

    fig, ax = plt.subplots(figsize=(4.8, 3.3))
    ax.loglog(dof, bsm, "o-", color=BLUE, label="BSM nHingedRigidBody")
    ax.loglog(dof, ndof, "^-", color=YELLOW, label="BSM spinningBodyNDOF")
    ax.loglog(dof, mujoco, "s-", color=RED, label="MuJoCo")
    for x, yBsm, yMujoco in zip(dof, bsm, mujoco):
        ax.text(x, np.sqrt(yBsm*yMujoco), f"{yBsm/yMujoco:.1f}$\\times$",
                fontsize=7, color=GRAY, ha="center", va="center")
    ax.set_xlabel("system degrees of freedom")
    ax.set_ylabel("wall-clock for 600 steps [s]")
    ax.legend(loc="upper left")
    ax.grid(True, which="both", alpha=0.2)
    fig.savefig(
        os.path.join(outputDir, "scenarioCompareFlexPanels_runtime.pdf"),
        metadata=PDF_METADATA,
    )
    plt.close(fig)


def orbitDtFigure(outputDir, resultsDir=None):
    rows = loadJson("sweepOrbitDt", resultsDir)["rows"]
    dts = np.array([r["dt"] for r in rows])
    analytic = np.array([r["bsmVsAnalyticMax"] for r in rows])
    cross = np.array([r["crossParadigmPosMax"] for r in rows])

    fig, ax = plt.subplots(figsize=(4.2, 3.1))
    ax.loglog(dts, analytic, "o-", color=BLUE, label="engine vs analytic Kepler")
    ax.loglog(dts, cross, "s-", color=RED, label="BSM vs MuJoCo")
    dref = np.array([dts.max(), dts.min()])
    ax.loglog(dref, analytic[0]*(dref/dts.max())**4, ":", color=GRAY,
              label=r"$\propto \Delta t^4$")
    ax.set_xlabel(r"integrator step $\Delta t$ [s]")
    ax.set_ylabel("max position difference [m]")
    # Pad below the flat cross-engine series so the legend sits in empty space.
    positiveCross = cross[cross > 0.0]
    if positiveCross.size:
        ax.set_ylim(bottom=positiveCross.min()/60.0)
    ax.legend(loc="lower right")
    ax.grid(True, which="both", alpha=0.2)
    fig.savefig(
        os.path.join(outputDir, "sweepOrbitDt.pdf"),
        metadata=PDF_METADATA,
    )
    plt.close(fig)


def torqueArtifactFigure(outputDir, resultsDir=None):
    data = loadJson("sweepTorqueArtifact", resultsDir)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 2.9))

    dts = [r["dt"] for r in data["dtSweep"]]
    mujocoArtifact = [r["mujocoMotionAttMax"] for r in data["dtSweep"]]
    bsmArtifact = [r["bsmMotionAttMax"] for r in data["dtSweep"]]
    restCross = [r["crossEngineRestAttMax"] for r in data["dtSweep"]]
    ax1.loglog(dts, mujocoArtifact, "o-", color=RED, label="MuJoCo orbiting-vs-rest")
    ax1.loglog(dts, restCross, "s-", color=YELLOW, label="cross-engine, at rest")
    ax1.loglog(dts, bsmArtifact, "^-", color=BLUE, label="BSM orbiting-vs-rest")
    dref = np.array([max(dts), min(dts)])
    ax1.loglog(dref, restCross[0]*(dref/max(dts))**4, ":", color=GRAY,
               label=r"$\propto \Delta t^4$")
    ax1.set_xlabel(r"integrator step $\Delta t$ [s]")
    ax1.set_ylabel("max principal angle [rad]")
    ax1.legend(loc="lower right")
    ax1.grid(True, which="both", alpha=0.2)

    speeds = np.array([r["speed"] for r in data["velocitySweep"]])/1000.0
    mujocoDrift = [r["mujocoDriftAttMax"] for r in data["velocitySweep"]]
    bsmDrift = [r["bsmDriftAttMax"] for r in data["velocitySweep"]]
    ax2.semilogy(speeds, mujocoDrift, "o-", color=RED, label="MuJoCo drifting-vs-rest")
    ax2.semilogy(speeds, bsmDrift, "^-", color=BLUE, label="BSM drifting-vs-rest")
    ax2.set_xlabel("inertial drift speed [km/s]")
    ax2.set_ylabel("max principal angle [rad]")
    ax2.legend(loc="center right")
    ax2.grid(True, which="both", alpha=0.2)

    fig.savefig(
        os.path.join(outputDir, "sweepTorqueArtifact.pdf"),
        metadata=PDF_METADATA,
    )
    plt.close(fig)


def _renderFigures(outputDir, resultsDir=None):
    """Render every print figure into an existing or new output directory."""
    os.makedirs(outputDir, exist_ok=True)
    for name in PAPER_OUTPUT_FILES:
        figurePath = os.path.join(outputDir, name)
        if os.path.exists(figurePath):
            os.remove(figurePath)
    matplotlib.rcParams.update(STYLE)

    paretoFigures(outputDir, resultsDir)
    scalingFigure(outputDir, resultsDir)
    orbitDtFigure(outputDir, resultsDir)
    torqueArtifactFigure(outputDir, resultsDir)
    print("Paper figures written to " + outputDir)


def _renderFiguresInFreshProcess(outputDir, resultsDir=None):
    """Render PDFs without inheriting Matplotlib state from scenario runs."""
    command = [
        sys.executable,
        os.path.abspath(__file__),
        os.path.abspath(outputDir),
        "--render-only",
    ]
    if resultsDir is not None:
        command.extend(("--results-dir", os.path.abspath(resultsDir)))
    subprocess.run(command, check=True, cwd=repoRoot)


def run(outputDir=None, provenanceFile=None, verifySource=True,
        resultsDir=None):
    """Validate stored inputs, render figures, and verify their exact hashes."""
    provenance = validateInputProvenance(
        provenanceFile, verifySource, resultsDir)
    outputDir = _outputDirectory(outputDir, resultsDir)
    _renderFiguresInFreshProcess(outputDir, resultsDir)
    validateFigureProvenance(provenance, outputDir)
    return provenance


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("outputDir", nargs="?")
    parser.add_argument("--provenance")
    parser.add_argument("--update-provenance", action="store_true")
    parser.add_argument("--render-only", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument("--results-dir", help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    if arguments.render_only:
        _renderFigures(
            _outputDirectory(arguments.outputDir, arguments.results_dir),
            arguments.results_dir,
        )
    elif arguments.update_provenance:
        if not os.environ.get(BUILD_RECEIPT_ENV):
            raise SystemExit(_cleanBuildAndRerun())
        updateInputProvenance(arguments.provenance, arguments.outputDir)
    else:
        run(arguments.outputDir, arguments.provenance)
