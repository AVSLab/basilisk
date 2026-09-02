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

import importlib.util
from functools import cache
from pathlib import Path
from types import SimpleNamespace

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MASTER_CONFIGURATIONS = (
    (
        "BskSim",
        REPOSITORY_ROOT / "examples/BskSim/BSK_masters.py",
        "BSKDynamicModels",
        "BSKFswModels",
        None,
    ),
    (
        "OpNav",
        REPOSITORY_ROOT / "examples/OpNavScenarios/BSK_OpNav.py",
        "BSKDynamicModels",
        "BSKFswModels",
        None,
    ),
    (
        "MuJoCo",
        REPOSITORY_ROOT / "examples/mujoco/BSK_mujocoMasters.py",
        "BSKMujocoDynamicsModels",
        "BSKMujocoFSWModels",
        None,
    ),
    (
        "MultiSat",
        REPOSITORY_ROOT / "examples/MultiSatBskSim/BSK_MultiSatMasters.py",
        "BSKDynamicModels",
        "BSKFswModels",
        2,
    ),
)


@cache
def loadMasterClass(masterName, masterPath):
    """Load and return a ``BSKSim`` class from an example master file."""
    moduleName = f"test_{masterName}_master"
    moduleSpec = importlib.util.spec_from_file_location(moduleName, masterPath)
    module = importlib.util.module_from_spec(moduleSpec)
    moduleSpec.loader.exec_module(module)
    return module.BSKSim


def makeSimulation(masterClass, numberSpacecraft, customRates=False):
    """Create a single- or multi-spacecraft master simulation."""
    rates = {"fswRate": 0.25, "dynRate": 0.5} if customRates else {}  # [s]
    if numberSpacecraft is None:
        return masterClass(**rates)
    if customRates:
        rates["envRate"] = 0.75  # [s]
    return masterClass(numberSpacecraft, **rates)


def makeModelInputs(dynClassName, fswClassName, numberSpacecraft):
    """Create lightweight model inputs for exercising the master classes."""

    class EnvironmentModel:
        def __init__(self, simBase, envRate):
            self.simBase = simBase
            self.rate = envRate

    class DynamicsModel:
        def __init__(self, simBase, dynRate, spacecraftIndex=None):
            self.simBase = simBase
            self.rate = dynRate
            self.spacecraftIndex = spacecraftIndex

    class FswModel:
        def __init__(self, simBase, fswRate, spacecraftIndex=None):
            self.simBase = simBase
            self.rate = fswRate
            self.spacecraftIndex = spacecraftIndex
            self.dynModels = simBase.DynModels

    modelModule = SimpleNamespace(BSKEnvironmentModel=EnvironmentModel)
    setattr(modelModule, dynClassName, DynamicsModel)
    setattr(modelModule, fswClassName, FswModel)
    if numberSpacecraft is None:
        return modelModule, modelModule, modelModule
    modelModules = [modelModule] * numberSpacecraft
    return modelModule, modelModules, modelModules


def setEnvironmentModel(simulation, environmentInput, numberSpacecraft):
    """Set the environment model when exercising the MultiSat master."""
    if numberSpacecraft is not None:
        simulation.set_EnvModel(environmentInput)


@pytest.mark.parametrize(
    "masterName, masterPath, dynClassName, fswClassName, numberSpacecraft",
    MASTER_CONFIGURATIONS,
    ids=[configuration[0] for configuration in MASTER_CONFIGURATIONS],
)
def test_modelAccessRequiresInitialization(
    masterName, masterPath, dynClassName, fswClassName, numberSpacecraft
):
    """Verify that both access styles reject uninitialized model access."""
    del dynClassName, fswClassName
    masterClass = loadMasterClass(masterName, masterPath)
    simulation = makeSimulation(masterClass, numberSpacecraft)

    if numberSpacecraft is not None:
        with pytest.raises(RuntimeError, match="not been added"):
            _ = simulation.EnvModel
        with pytest.raises(RuntimeError, match="not been added"):
            simulation.get_EnvModel()
    with pytest.raises(RuntimeError, match="not been added"):
        _ = simulation.DynModels
    with pytest.raises(RuntimeError, match="not been added"):
        simulation.get_DynModel()
    with pytest.raises(RuntimeError, match="not been added"):
        _ = simulation.FSWModels
    with pytest.raises(RuntimeError, match="not been added"):
        simulation.get_FswModel()


def test_multiSatDynamicsRequiresEnvironment():
    """Verify MultiSat dynamics setup requires the environment to be added first."""
    masterPath = REPOSITORY_ROOT / "examples/MultiSatBskSim/BSK_MultiSatMasters.py"
    masterClass = loadMasterClass("MultiSat", masterPath)
    simulation = makeSimulation(masterClass, 2)
    environmentInput, dynamicsInput, _ = makeModelInputs(
        "BSKDynamicModels", "BSKFswModels", 2
    )

    processCount = len(simulation.procList)
    with pytest.raises(
        RuntimeError,
        match="An environment model must be added before the dynamics models",
    ):
        simulation.set_DynModel(dynamicsInput)
    assert len(simulation.procList) == processCount
    assert simulation.dynamics_added is False

    simulation.set_EnvModel(environmentInput)
    simulation.set_DynModel(dynamicsInput)
    assert simulation.dynamics_added is True


@pytest.mark.parametrize(
    "masterName, masterPath, dynClassName, fswClassName, numberSpacecraft",
    MASTER_CONFIGURATIONS,
    ids=[configuration[0] for configuration in MASTER_CONFIGURATIONS],
)
def test_modelReferencesAreReadOnlyAndSingleUse(
    masterName, masterPath, dynClassName, fswClassName, numberSpacecraft
):
    """Verify model references cannot be replaced or initialized more than once."""
    masterClass = loadMasterClass(masterName, masterPath)
    simulation = makeSimulation(masterClass, numberSpacecraft, customRates=True)
    environmentInput, dynamicsInput, fswInput = makeModelInputs(
        dynClassName, fswClassName, numberSpacecraft
    )

    if numberSpacecraft is not None:
        simulation.set_EnvModel(environmentInput)
        assert simulation.get_EnvModel() is simulation.EnvModel
        assert simulation.EnvModel.rate == simulation.envRate
        assert simulation.environment_added is True
        with pytest.raises(AttributeError):
            simulation.EnvModel = object()
        with pytest.raises(RuntimeError, match="already been added"):
            simulation.set_EnvModel(environmentInput)

    processCount = len(simulation.procList)
    with pytest.raises(RuntimeError, match="must be added before"):
        simulation.set_FswModel(fswInput)
    assert len(simulation.procList) == processCount

    simulation.set_DynModel(dynamicsInput)
    dynModels = simulation.DynModels
    assert simulation.get_DynModel() is dynModels
    dynModelList = dynModels if numberSpacecraft is not None else [dynModels]
    assert all(model.rate == simulation.dynRate for model in dynModelList)
    assert simulation.dynamics_added is True

    simulation.set_FswModel(fswInput)
    fswModels = simulation.FSWModels
    assert simulation.get_FswModel() is fswModels
    fswModelList = fswModels if numberSpacecraft is not None else [fswModels]
    assert all(model.dynModels is dynModels for model in fswModelList)
    assert all(model.rate == simulation.fswRate for model in fswModelList)
    assert simulation.fsw_added is True

    with pytest.raises(AttributeError):
        simulation.DynModels = object()
    with pytest.raises(AttributeError):
        simulation.FSWModels = object()
    with pytest.raises(RuntimeError, match="already been added"):
        simulation.set_DynModel(dynamicsInput)
    with pytest.raises(RuntimeError, match="already been added"):
        simulation.set_FswModel(fswInput)


@pytest.mark.parametrize(
    "masterName, masterPath, dynClassName, fswClassName, numberSpacecraft",
    MASTER_CONFIGURATIONS,
    ids=[configuration[0] for configuration in MASTER_CONFIGURATIONS],
)
def test_failedConstructionIsTerminal(masterName, masterPath, dynClassName, fswClassName, numberSpacecraft):
    """Verify failed constructors leave models unpublished and block unsafe retries."""
    masterClass = loadMasterClass(masterName, masterPath)

    if numberSpacecraft is not None:
        environmentInput, _, _ = makeModelInputs(dynClassName, fswClassName, numberSpacecraft)

        class FailingEnvironmentModel:
            def __init__(self, simBase, envRate):
                del simBase, envRate
                raise ValueError("environment construction failed")

        environmentInput.BSKEnvironmentModel = FailingEnvironmentModel
        simulation = makeSimulation(masterClass, numberSpacecraft)
        processCount = len(simulation.procList)
        with pytest.raises(ValueError, match="environment construction failed"):
            simulation.set_EnvModel(environmentInput)
        assert len(simulation.procList) == processCount + 1
        with pytest.raises(RuntimeError, match="previous environment model setup attempt failed"):
            simulation.set_EnvModel(environmentInput)
        assert len(simulation.procList) == processCount + 1
        assert simulation.environment_added is False
        with pytest.raises(RuntimeError, match="not been added"):
            _ = simulation.EnvModel

    environmentInput, dynamicsInput, _ = makeModelInputs(
        dynClassName, fswClassName, numberSpacecraft
    )

    class FailingDynamicsModel:
        def __init__(self, simBase, dynRate, spacecraftIndex=None):
            del simBase, dynRate
            if spacecraftIndex is None or spacecraftIndex == numberSpacecraft - 1:
                raise ValueError("dynamics construction failed")

    if numberSpacecraft is None:
        setattr(dynamicsInput, dynClassName, FailingDynamicsModel)
    else:
        for modelModule in dynamicsInput:
            setattr(modelModule, dynClassName, FailingDynamicsModel)
    simulation = makeSimulation(masterClass, numberSpacecraft)
    setEnvironmentModel(simulation, environmentInput, numberSpacecraft)
    processCount = len(simulation.procList)
    setupProcessCount = numberSpacecraft or 1
    with pytest.raises(ValueError, match="dynamics construction failed"):
        simulation.set_DynModel(dynamicsInput)
    assert len(simulation.procList) == processCount + setupProcessCount
    with pytest.raises(RuntimeError, match="previous dynamics model setup attempt failed"):
        simulation.set_DynModel(dynamicsInput)
    assert len(simulation.procList) == processCount + setupProcessCount
    assert simulation.dynamics_added is False
    with pytest.raises(RuntimeError, match="not been added"):
        _ = simulation.DynModels

    class FailingFswModel:
        def __init__(self, simBase, fswRate, spacecraftIndex=None):
            del simBase, fswRate
            if spacecraftIndex is None or spacecraftIndex == numberSpacecraft - 1:
                raise ValueError("flight software construction failed")

    environmentInput, dynamicsInput, fswInput = makeModelInputs(
        dynClassName, fswClassName, numberSpacecraft
    )
    if numberSpacecraft is None:
        setattr(fswInput, fswClassName, FailingFswModel)
    else:
        for modelModule in fswInput:
            setattr(modelModule, fswClassName, FailingFswModel)
    simulation = makeSimulation(masterClass, numberSpacecraft)
    setEnvironmentModel(simulation, environmentInput, numberSpacecraft)
    simulation.set_DynModel(dynamicsInput)
    processCount = len(simulation.procList)
    with pytest.raises(ValueError, match="flight software construction failed"):
        simulation.set_FswModel(fswInput)
    assert len(simulation.procList) == processCount + setupProcessCount
    with pytest.raises(RuntimeError, match="previous flight software model setup attempt failed"):
        simulation.set_FswModel(fswInput)
    assert len(simulation.procList) == processCount + setupProcessCount
    assert simulation.fsw_added is False
    with pytest.raises(RuntimeError, match="not been added"):
        _ = simulation.FSWModels
